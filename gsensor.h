
#ifndef __GSENSOR_H__
#define __GSENSOR_H__

#include <stdbool.h>

#define DBG_PRINT    UARTprintf



static  bool SF_APDS9960_wireReadDataByte(uint8_t reg, uint8_t *val);
static  int SF_APDS9960_wireReadDataBlock(uint8_t ucRegAddr,uint8_t *pucBlkData,uint8_t ucBlkDataSz);
//static bool SF_APDS9960_isGestureAvailable();
//static uint8_t SF_APDS9960_getMode();


static int SF_APDS9960_wireReadDataBlock(uint8_t ucRegAddr,uint8_t *pucBlkData,uint8_t ucBlkDataSz)
{
    //
    // Invoke the readfrom I2C API to get the required bytes
    //
    if(I2C_IF_ReadFrom(SF_APDS9960_I2C_ADDR, &ucRegAddr, 1,
                   pucBlkData, ucBlkDataSz) != 0)
    {
        DBG_PRINT("I2C readfrom failed\n");
        return -1;
    }

    return ucBlkDataSz;
}


static bool SF_APDS9960_wireReadDataByte(uint8_t ucRegAddr, uint8_t *pucRegValue)
{
    //
    // Invoke the readfrom  API to get the required byte
    //
    if(I2C_IF_ReadFrom(SF_APDS9960_I2C_ADDR, &ucRegAddr, sizeof(uint8_t),
                   pucRegValue, sizeof(uint8_t)) != 0)
    {
        DBG_PRINT("I2C readfrom failed\n\r");
        return false;
    }

    return true;
}


//static bool SF_APDS9960_isGestureAvailable()
//{
//    uint8_t val;
//
//    /* Read value from GSTATUS register */
//    if( !SF_APDS9960_wireReadDataByte(SF_APDS9960_GSTATUS, &val) ) {
//        return SF_APDS9960_ERROR;
//    }
//
//    /* Shift and mask out GVALID bit */
//    val &= SF_APDS9960_GVALID;
//
//    /* Return true/false based on GVALID bit */
//    if( val == 1) {
//        return true;
//    } else {
//        return false;
//    }
//}
//
//static uint8_t SF_APDS9960_getMode()
//{
//    uint8_t enable_value;
//
//    /* Read current ENABLE register */
//    if( !SF_APDS9960_wireReadDataByte(SF_APDS9960_ENABLE, &enable_value) ) {
//        return SF_APDS9960_ERROR;
//    }
//
//    return enable_value;
//}

#endif
