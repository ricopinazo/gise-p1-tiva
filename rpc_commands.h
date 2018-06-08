/*
 * rpccommands.h
 *
 *  Created on: 11 abr. 2018
 *
 */

#ifndef RPCCOMMANDS_H
#define RPCCOMMANDS_H
//C�digos de los comandos y definicion de parametros para el protocolo RPC

// El estudiante debe a�adir aqui cada nuevo comando que implemente. IMPORTANTE el orden de los comandos
// debe SER EL MISMO aqui, y en el codigo equivalente en la parte del microcontrolador (Code Composer)


//#define MASK_FOR_12_BITS 0x0FF0

typedef enum {
    COMMAND_REJECTED,
    COMMAND_PING,
    COMMAND_LED_GPIO,
    COMMAND_LED_PWM_BRIGHTNESS,
    COMMAND_LED_PWM_COLOR,
    COMMAND_BUTTONS_STATUS,
    COMMAND_BUTTONS_REQUEST,
    COMMAND_BUTTONS_ANSWER,
    COMMAND_SAMPLING_CONFIG,
    COMMAND_ADC_8BITS_SAMPLES,
    COMMAND_ADC_12BITS_SAMPLES,
    COMMAND_GSENSOR_COLOR_REQUEST,
    COMMAND_GSENSOR_COLOR_ANSWER,
    COMMAND_GSENSOR_GESTURE,
    COMMAND_GSENSOR_CONFIG_THRESHOLD,
    COMMAND_GSENSOR_THRESHOLD_EXCEED,
    COMMAND_GSENSOR_MODE_CONFIG,
    COMMAND_GSENSOR_FIFO
} commandTypes;

//Estructuras relacionadas con los parametros de los comandos. El estuadiante debera crear las
// estructuras adecuadas a los comandos usados, y asegurarse de su compatibilidad en ambos extremos

#pragma pack(1) //Cambia el alineamiento de datos en memoria a 1 byte.

typedef struct {
    uint8_t command;
} PARAMETERS_COMMAND_REJECTED;

typedef union
{
    struct
    {
         uint8_t padding:1;
         uint8_t red:1;
         uint8_t blue:1;
         uint8_t green:1;
    } leds;
    uint8_t value;
} PARAMETERS_LED_GPIO;

typedef struct
{
    float rIntensity;
} PARAMETERS_LED_PWM_BRIGHTNESS;

typedef struct
{
    uint32_t colors[3];
} PARAMETERS_LED_PWM_COLOR;

typedef union
{
    struct
    {
      uint8_t active : 1;
      uint8_t mode12 : 1;
      uint16_t freq : 14;
    }config;
} PARAMETERS_SAMPLING_CONFIG;

typedef struct
{
    uint32_t channel_0;
    uint32_t channel_1;
    uint32_t channel_2;
    uint32_t channel_3;
} SimpleADCSample;

typedef struct
{
    struct
    {
        uint16_t samples[8];
    } channel [4];
} PARAMETERS_ADC_12SAMPLES;

typedef struct
{
    struct
    {
        uint8_t samples[8];
    } channel [4];
} PARAMETERS_ADC_8SAMPLES;

typedef struct
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t intensity;
} PARAMETERS_GSENSOR_COLOR;

typedef struct
{
    uint8_t gesture;
} PARAMETERS_GSENSOR_GESTURE;

typedef struct
{
    uint8_t threshold;
} PARAMETERS_GSENSOR_CONFIG_THRESHOLD;

typedef struct
{
    bool fifo_mode;
} PARAMETERS_GSENSOR_FIFO_MODE;

typedef struct
{
    uint8_t data_size;
    uint8_t fifo_data[128];
} PARAMETERS_GSENSOR_FIFO;

#pragma pack()  //...Pero solo para los comandos que voy a intercambiar, no para el resto.

#endif // RPCCOMMANDS_H

