/*
 * tivarpc.c
 *
 * Implementa la funcionalidad RPC entre la TIVA y el interfaz de usuario
 */

#include <tivarpc.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
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
#include "drivers/rgb.h"
#include "usb_dev_serial.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "rpc_commands.h"


//Defino a un tipo que es un puntero a funcion con el prototipo que tienen que tener las funciones que definamos
typedef int32_t (*rpc_function_prototype)(uint32_t param_size, void *param);

static uint8_t Rxframe[MAX_FRAME_SIZE];	//Usar una global permite ahorrar pila en la tarea de RX.
static uint8_t Txframe[MAX_FRAME_SIZE]; //Usar una global permite ahorrar pila en las tareas, pero hay que tener cuidado al transmitir desde varias tareas!!!!
static uint32_t gRemoteProtocolErrors=0;

xSemaphoreHandle frame_mutex;
xSemaphoreHandle uart_mutex;

extern xQueueHandle ButtonsQueue;
extern xQueueHandle ADCQueue;

PARAMETERS_LED_PWM_COLOR last_color;
PARAMETERS_SAMPLING_CONFIG sampling_config;


//Funciones "internas//
static int32_t TivaRPC_ReceiveFrame(uint8_t *frame, int32_t maxFrameSize);

// *********************************************************************************************
// ********************* FUNCIONES RPC (se ejecutan en respuesta a comandos RPC recibidos desde el interfaz Qt *********************
// *********************************************************************************************

//Funcion que se ejecuta cuando llega un paquete indicando comando rechazado
static int32_t RPC_RejectedCommand(uint32_t param_size, void *param)
{
    //He recibido "Comando rechazado" desde el PC
    //TODO, por hacer: Tratar dicho error??
    gRemoteProtocolErrors++;
    return 0;
}





//Funcion que se ejecuta cuando llega un PING
static int32_t RPC_Ping(uint32_t param_size, void *param)
{
    int32_t numdatos;

    numdatos = TivaRPC_SendFrame(COMMAND_PING,NULL,0);

    return numdatos;
}



//Funcion que se ejecuta cuando llega el comando que configura los LEDS
static int32_t RPC_LEDGpio(uint32_t param_size, void *param)
{
    PARAMETERS_LED_GPIO parametro;

    if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
    {
        ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,parametro.value);
        return 0;
    }
    else
    {
        return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
    }
}

//Funcion que se ejecuta cuando recibimos un comando que no tenemos aun implementado
static int32_t RPC_UnimplementedCommand(uint32_t param_size, void *param)
{
    return PROT_ERROR_UNIMPLEMENTED_COMMAND;
}





//Funcion que se ejecuta cuando llega el comando que configura el BRILLO
static int32_t RPC_LEDPwmBrightness(uint32_t param_size, void *param)
{
    PARAMETERS_LED_PWM_BRIGHTNESS parametro;


    if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
    {
        RGBEnable();
        RGBColorSet(last_color.colors);
        RGBIntensitySet(parametro.rIntensity);

        return 0;
    }
    else
    {
        return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
    }
}

static int32_t RPC_LEDPwmRGB(uint32_t param_size, void *param)
{
    PARAMETERS_LED_PWM_COLOR parametro;
    TimerDisable(TIMER5_BASE, TIMER_A);

    if(check_and_extract_command_param(param, param_size, sizeof(parametro), &parametro) > 0)
    {
        RGBEnable();

        TimerDisable(TIMER5_BASE, TIMER_A);

        parametro.colors[0] = parametro.colors[0] << 8;
        parametro.colors[1] = parametro.colors[1] << 8;
        parametro.colors[2] = parametro.colors[2] << 8;

        memcpy(&last_color.colors, &parametro.colors, sizeof(PARAMETERS_LED_PWM_COLOR));
        RGBColorSet(parametro.colors);

        return 0;
    }else
    {
        return PROT_ERROR_INCORRECT_PARAM_SIZE;
    }
}

static int32_t RPC_SendButtonsStatus(uint32_t param_size, void *param)
{
    return TivaRPC_SendFrame(COMMAND_BUTTONS_STATUS, (uint8_t *)param, param_size);
}

static int32_t RPC_ButtonsStatusAnswer(uint32_t param_size, void *param)
{
    uint8_t ButtonsPoll = GPIOPinRead(GPIO_PORTF_BASE, ALL_BUTTONS);
    return TivaRPC_SendFrame(COMMAND_BUTTONS_ANSWER, &ButtonsPoll, sizeof(ButtonsPoll));
}


static int32_t RPC_SamplingConfig(uint32_t param_size, void *param)
{
    if(check_and_extract_command_param(param, param_size, sizeof(sampling_config),&sampling_config) > 0)
    {
        ADCIntDisable(ADC0_BASE,ADC_INT_SS1);
        TimerDisable(TIMER5_BASE, TIMER_A);

        TimerLoadSet(TIMER5_BASE,TIMER_A,(uint32_t)(SysCtlClockGet()/sampling_config.config.freq) - 1);

        if(sampling_config.config.mode12)
        {
            ADCHardwareOversampleConfigure(ADC0_BASE,4);
        } else
        {
            ADCHardwareOversampleConfigure(ADC0_BASE,0);
        }

        if(sampling_config.config.active)
        {
          TimerEnable(TIMER5_BASE, TIMER_A);
          ADCIntClear(ADC0_BASE,ADC_INT_SS1);
          ADCIntEnable(ADC0_BASE,ADC_INT_SS1);
        }

        return 0;
    }else{
        return PROT_ERROR_INCORRECT_PARAM_SIZE;
    }

}

// *********************************************************************************************
// ********************* Tabla de  FUnciones RPC
// *********************************************************************************************
/* Array que contiene las funciones que se van a ejecutar en respuesta a cada comando */
static const rpc_function_prototype rpc_function_table[]={
                                            RPC_RejectedCommand, /* Responde al paquete comando rechazado */
                                            RPC_Ping, /* Responde al comando ping */
                                            RPC_LEDGpio, /* Responde al comando LEDS */
                                            RPC_LEDPwmBrightness, /* Responde al comando Brillo */
                                            RPC_LEDPwmRGB,
                                            RPC_SendButtonsStatus,
                                            RPC_ButtonsStatusAnswer,
                                            RPC_UnimplementedCommand,
                                            RPC_SamplingConfig,
                                             /* Este comando no esta implementado aun */
};


// Codigo para procesar los comandos recibidos a traves del canal USB del micro ("conector lateral")
//Esta tarea decodifica los comandos y ejecuta la función que corresponda a cada uno de ellos (por posicion)
//También gestiona posibles errores en la comunicacion




static portTASK_FUNCTION(ButtonRTOSTask, pvParameters)
{
    uint8_t data_received;
    while(1){
        if(xQueueReceive(ButtonsQueue, &data_received, portMAX_DELAY) != pdTRUE){
            while(1);
        }

       xSemaphoreTake(frame_mutex,portMAX_DELAY);
       rpc_function_table[COMMAND_BUTTONS_STATUS](sizeof(data_received), &data_received);
       xSemaphoreGive(frame_mutex);

    }
}

static portTASK_FUNCTION(ADCTask, pvParameters)
{
   static SimpleADCSample sample;
   static PARAMETERS_ADC_12SAMPLES complete_12sample;
   static PARAMETERS_ADC_8SAMPLES complete_8sample;

   char samples_count = 0;

   while(1){
       if(xQueueReceive(ADCQueue, &sample, portMAX_DELAY) != pdTRUE)
           while(1);

       xSemaphoreTake(frame_mutex, portMAX_DELAY);

       MAP_IntDisable(INT_GPIOF);


       if(!sampling_config.config.mode12)
       {
           complete_8sample.channel[0].samples[samples_count] = (uint8_t)((sample.channel_0) >> 4);
           complete_8sample.channel[1].samples[samples_count] = (uint8_t)((sample.channel_1) >> 4);
           complete_8sample.channel[2].samples[samples_count] = (uint8_t)((sample.channel_2) >> 4);
           complete_8sample.channel[3].samples[samples_count] = (uint8_t)((sample.channel_3) >> 4);

           if(samples_count == 7){
             samples_count = 0;

             TivaRPC_SendFrame(COMMAND_ADC_8BITS_SAMPLES, &complete_8sample, sizeof(complete_8sample));

           }else{
              samples_count++;
           }

       } else {
           complete_12sample.channel[0].samples[samples_count] = (uint16_t)sample.channel_0;
           complete_12sample.channel[1].samples[samples_count] = (uint16_t)sample.channel_1;
           complete_12sample.channel[2].samples[samples_count] = (uint16_t)sample.channel_2;
           complete_12sample.channel[3].samples[samples_count] = (uint16_t)sample.channel_3;

           if(samples_count == 7)
           {
               samples_count = 0;
               TivaRPC_SendFrame(COMMAND_ADC_12BITS_SAMPLES, &complete_12sample, sizeof(complete_12sample));
           }else
           {
               samples_count++;
           }
       }

       MAP_IntEnable(INT_GPIOF);
       xSemaphoreGive(frame_mutex);


   }
}




static portTASK_FUNCTION(TivaRPC_ServerTask, pvParameters ){

    //Frame es global en este fichero, se reutiliza en las funciones que envian respuestas ---> CUIDADO!!!

    int32_t numdatos;
    uint8_t command;
    void *ptrtoreceivedparam;

    /* The parameters are not used. (elimina el warning)*/


    ( void ) pvParameters;

    for(;;)
    {
        //Espera hasta que se reciba una trama con datos serializados por el interfaz USB
        numdatos=TivaRPC_ReceiveFrame(Rxframe,MAX_FRAME_SIZE); //Esta funcion es bloqueante

        if (numdatos>0)
        {
            //Si no ha habido errores recibiendo la trama, la intenta decodificar y procesar
            //primero se "desestufa" y se comprueba el checksum
            numdatos=destuff_and_check_checksum(Rxframe,numdatos);
            if (numdatos<0)
            {
                //Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
                gRemoteProtocolErrors++;
                // Procesamiento del error (TODO, POR HACER!!)
            }
            else
            {
                //El paquete esta bien, luego procedo a tratarlo.
                //Obtiene el valor del campo comando
                command=decode_command_type(Rxframe);
                //Obtiene un puntero al campo de parametros y su tamanio.
                numdatos=get_command_param_pointer(Rxframe,numdatos,&ptrtoreceivedparam);

                //Accedemos a la tabla de funciones y ejecutarmos la correspondiente al comando que ha llegado
                //Primero hay que comprobar que el tamanio de dicha tabla es suficiente.

                if (command<(sizeof(rpc_function_table)/sizeof(rpc_function_prototype)))
                {
                    int32_t error_status;
                    //Aqui es donde se ejecuta a funcion de la tabla que corresponde con el valor de comando que ha llegado
                    xSemaphoreTake(frame_mutex,portMAX_DELAY);
                    error_status = rpc_function_table[command](numdatos,ptrtoreceivedparam); //La funcion puede devolver códigos de error.

                    //Una vez ejecutado, se comprueba si ha habido errores.
                    xSemaphoreTake(uart_mutex, portMAX_DELAY);
                    switch(error_status)
                    {

                        //Se procesarían a continuación
                        case PROT_ERROR_NOMEM:
                        {
                            // Procesamiento del error NO MEMORY
                            UARTprintf("RPC Error: not enough memory\n");
                        }
                        break;
                        case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
                        {
                            // Procesamiento del error STUFFED_FRAME_TOO_LONG
                            UARTprintf("RPC Error: Frame too long. Cannot be created\n");
                        }
                        break;
                        case PROT_ERROR_COMMAND_TOO_LONG:
                        {
                            // Procesamiento del error COMMAND TOO LONG
                            UARTprintf("RPC Error: Packet too long. Cannot be allocated\n");
                        }
                        break;
                        case PROT_ERROR_INCORRECT_PARAM_SIZE:
                        {
                            // Procesamiento del error INCORRECT PARAM SIZE
                            UARTprintf("RPC Error: Incorrect parameter size\n");
                        }
                        break;
                        case PROT_ERROR_UNIMPLEMENTED_COMMAND:
                        {
                            PARAMETERS_COMMAND_REJECTED parametro;
                            parametro.command=command;
                            numdatos=TivaRPC_SendFrame(COMMAND_REJECTED,&parametro,sizeof(parametro));
                            UARTprintf("RPC Error: Unexpected command: %x\n",(uint32_t)command);
                            gRemoteProtocolErrors++;
                            //Aqui se podria, ademas, comprobar numdatos....
                        }
                        break;
                        default:
                            break;
                    }

                    xSemaphoreGive(uart_mutex);
                    xSemaphoreGive(frame_mutex);
                }

                else
                {
                    /* El valor del comando que ha llegado es superior al numero de comandos que hay implementados en la tabla  */
                    // Se envia el "Comando Rechazado" hacia el GUI
                    PARAMETERS_COMMAND_REJECTED parametro;
                    parametro.command=command;
                    numdatos = TivaRPC_SendFrame(COMMAND_REJECTED,&parametro,sizeof(parametro));
                    UARTprintf("RPC Error: Unexpected command: %x\n",(uint32_t)command);
                    //Aqui se podria, ademas,  comprobar numdatos....
                    gRemoteProtocolErrors++;
                }
            }
        }
        else
        { // if (numdatos >0)
            //Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
            gRemoteProtocolErrors++;
            // Procesamiento del error (TODO)
        }

    }
}



//Inicializa la tarea que recibe comandos (se debe llamar desde main())
void TivaRPC_Init(void)
{
    USBSerialInit(32,32);   //Inicializo el  sistema USB-serie

    //
    // Crea la tarea que gestiona los comandos USB (definidos en CommandProcessingTask)
    //
    frame_mutex = xSemaphoreCreateMutex();
    if(NULL == frame_mutex)
        while(1);


    uart_mutex= xSemaphoreCreateMutex();
    if(NULL == uart_mutex)
        while(1);





    if(xTaskCreate(TivaRPC_ServerTask, (portCHAR *)"usbser",TIVARPC_TASK_STACK, NULL, TIVARPC_TASK_PRIORITY, NULL) != pdTRUE)
    {
        while(1);
    }

    if((xTaskCreate(ButtonRTOSTask, "ButtonRTOSTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }

    ADCIntDisable(ADC0_BASE,ADC_INT_SS1);
    TimerDisable(TIMER5_BASE, TIMER_A);

    sampling_config.config.active = 0;
    sampling_config.config.mode12 = 0;
    sampling_config.config.freq = 1;

    TimerLoadSet(TIMER5_BASE,TIMER_A,(SysCtlClockGet()/sampling_config.config.freq) - 1);

    if(xTaskCreate(ADCTask, "ADCTask", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdTRUE){
        while(1);
    }


}


// TODO
//Ojo!! TxFrame es global (para ahorrar memoria de pila en las tareas) --> Se deben tomar precauciones al usar esta funcion varias tareas
//IDEM en lo que respecta al envio por la conexion USB serie desde varias tareas....
//Estas precauciones no se han tomado en este codigo de parti1da, pero al realizar la practica se deberian tener en cuenta....
// "TODO" (por hacer)
int32_t TivaRPC_SendFrame(uint8_t comando,void *parameter,int32_t paramsize)
{
    int32_t numdatos;

    numdatos=create_frame(Txframe,comando,parameter,paramsize,MAX_FRAME_SIZE);
    if (numdatos>=0)
    {
        USBSerialWrite(Txframe,numdatos,portMAX_DELAY);
    }

    return numdatos;
}

/* Recibe una trama (sin los caracteres de inicio y fin */
/* Utiliza la funcion bloqueante xSerialGetChar ---> Esta funcion es bloqueante y no se puede llamar desde una ISR !!!*/
// Esta funcion es INTERNA de la biblioteca y solo se utiliza en la rarea TivaRPC_ServerTask
static int32_t TivaRPC_ReceiveFrame(uint8_t *frame, int32_t maxFrameSize)
{
    int32_t i;
    uint8_t rxByte;

    do
    {
        //Elimino bytes de la cola de recepcion hasta llegar a comienzo de nueva trama
        USBSerialGetChar( ( char *)&rxByte, portMAX_DELAY);
    } while (rxByte!=START_FRAME_CHAR);

    i=0;
    do
    {
        USBSerialGetChar( ( char *)frame, portMAX_DELAY);
        i++;
    } while ((*(frame++)!=STOP_FRAME_CHAR)&&(i<=maxFrameSize));

    if (i>maxFrameSize)
    {
        return PROT_ERROR_RX_FRAME_TOO_LONG;    //Numero Negativo indicando error
    }
    else
    {
        return (i-END_SIZE);    //Devuelve el numero de bytes recibidos (quitando el de BYTE DE STOP)
    }
}
