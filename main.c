//*****************************************************************************
//
// Codigo de partida Practica 1.
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//*****************************************************************************

#include <serialprotocol.h>
#include <serialprotocol.h>
#include<stdbool.h>
#include<stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"
#include "event_groups.h"
#include "sparkfun_apds9960drv.h"

#include "drivers/rgb.h"
#include <tivarpc.h>

//Globales
uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;

xQueueHandle ButtonsQueue;
xQueueHandle ADCQueue;

EventGroupHandle_t events;

extern void vUARTTask( void *pvParameters );

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
// Esta funcion se llama si la biblioteca driverlib o FreeRTOS comprueban la existencia de un error (mediante
// las macros ASSERT(...) y configASSERT(...)
// Los parametros nombrefich y linea contienen informacion de en que punto se encuentra el error...
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea)
{
    UARTprintf("File: %s, Error line: %d\n\n", nombrefich, linea);
    UARTprintf("Debug error!\n");
    while(1)
    {
    }
}
#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//


void SystemInit(void);
void ConfigADC();

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static uint8_t count = 0;

	if (++count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}





//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
	while(1);
}



//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************

// El codigo de esta tarea esta definida en el fichero command.c, es la que se encarga de procesar los comandos del interprete a traves
// del terminal serie (puTTY)
//Aqui solo la declaramos para poderla referenciar en la funcion main
extern void vUARTTask( void *pvParameters );





// Codigo de tarea de ejemplo: Esta tarea no se llega realmente a crear en el ejemplo inicial
/*static portTASK_FUNCTION(LEDTask,pvParameters)
{

	uint8_t estado_led=0;

	//
	// Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
	//
	while(1)
	{
		estado_led=!estado_led;

		if (estado_led)
		{
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 , GPIO_PIN_1);
			vTaskDelay(0.1*configTICK_RATE_HZ);        //Espera del RTOS (eficiente, no gasta CPU)
			//Esta espera es de unos 100ms aproximadamente.
		}
		else
		{
			GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1,0);
			vTaskDelay(2*configTICK_RATE_HZ);        //Espera del RTOS (eficiente, no gasta CPU)
			//Esta espera es de unos 2s aproximadamente.
		}
	}
}*/




//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{


    SystemInit();


	/********************************      Creacion de tareas   ************************************/

	// Crea la tarea que gestiona los comandos UART (definida en el fichero commands.c)
	//
	if((xTaskCreate(vUARTTask, (portCHAR *)"Uart", 512,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
	{
		while(1);
	}

	ConfigADC();


	//Esta funcion crea internamente una tarea para las comunicaciones USB.
	//Ademas, inicializa el USB y configura el perfil USB-CDC
	TivaRPC_Init(); //Inicializo la aplicacion de comunicacion con el PC (Remote). Ver fichero tivarpc.c


	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas
	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.

	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}


void SystemInit(void)
{
    //
        // Set the clocking to run at 40 MHz from the PLL.
        //
        SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                SYSCTL_OSC_MAIN);   //Ponermos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)


        // Get the system clock speed.
        g_ulSystemClock = SysCtlClockGet();


        //Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
        //                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
        ROM_SysCtlPeripheralClockGating(true);

        // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
        // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
        // (y por tanto este no se deberia utilizar para otra cosa).
        CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ/10, 3);

        //
        // Inicializa la UARTy la configura a 115.200 bps, 8-N-1 .
        //se usa para mandar y recibir mensajes y comandos por el puerto serie
        // Mediante un programa terminal como gtkterm, putty, cutecom, etc...
        //
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
        ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
        ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        UARTStdioConfig(0,115200,SysCtlClockGet());

        ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);   //La UART tiene que seguir funcionando aunque el micro este dormido
        ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);   //La UART tiene que seguir funcionando aunque el micro este dormido


        //Inicializa los LEDs usando libreria RGB --> usa Timers 0 y 1 (eliminar si no se usa finalmente)
        RGBInit(1);
        SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
        SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
        SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);  //Redundante porque BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH son el mismo
        SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER5);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);

        //Volvemos a configurar los LEDs en modo GPIO POR Defecto
        ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
        GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4);

         ButtonsQueue = xQueueCreate(1, sizeof(uint8_t));
         if(NULL == ButtonsQueue)
             while(1);

         xQueueReset(ButtonsQueue);

         ADCQueue = xQueueCreate(16, sizeof(SimpleADCSample));
         if(NULL == ADCQueue)
             while(1);

         xQueueReset(ADCQueue);

        ButtonsInit();
        MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_BOTH_EDGES);
        GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4,GPIO_BOTH_EDGES);

        MAP_IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);
        IntPrioritySet(INT_GPIOE,configMAX_SYSCALL_INTERRUPT_PRIORITY);

        MAP_GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);
        GPIOIntEnable(GPIO_PORTE_BASE,GPIO_PIN_4);
        GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_4);

        MAP_IntEnable(INT_GPIOF);
        IntEnable(INT_GPIOE);


        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
        TimerClockSourceSet(TIMER5_BASE,TIMER_CLOCK_SYSTEM);
        TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
        TimerLoadSet(TIMER5_BASE, TIMER_A, SysCtlClockGet() - 1);

}



void ButtonPressed(void)
{
    uint8_t poll;

    poll = ROM_GPIOPinRead(GPIO_PORTF_BASE, ALL_BUTTONS);
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    if(xQueueSendFromISR(ButtonsQueue, &poll, &higherPriorityTaskWoken) == pdTRUE);
        xEventGroupSetBitsFromISR(events, BUTTONS_FLAG, &higherPriorityTaskWoken);

    GPIOIntClear(GPIO_PORTF_BASE, ALL_BUTTONS);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void GSensorProximityInterrupt(void)
{
    signed portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    xEventGroupSetBitsFromISR(events, GSENSOR_FLAG, &higherPriorityTaskWoken);

    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_4);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);

}

void ADC_ISR(void){

   portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

   SimpleADCSample sample;

   ADCIntClear(ADC0_BASE,1);

   ADCSequenceDataGet(ADC0_BASE, 1,(uint32_t *)&sample);

   if(xQueueSendFromISR(ADCQueue, &sample, &higherPriorityTaskWoken) == errQUEUE_FULL){

   }

   portEND_SWITCHING_ISR(higherPriorityTaskWoken);

}


void ConfigADC(){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);

    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

    ADCSequenceConfigure(ADC0_BASE,1,ADC_TRIGGER_TIMER, 0);
    TimerControlTrigger(TIMER5_BASE, TIMER_A, true);

    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END );

    ADCSequenceEnable(ADC0_BASE,1);

    ADCIntClear(ADC0_BASE,1);
    ADCIntEnable(ADC0_BASE,1);
    IntPrioritySet(INT_ADC0SS1, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_ADC0SS1);
}


