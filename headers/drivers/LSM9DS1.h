/*
 * LSM9DS1.h
 *
 *  Created on: Feb 23, 2018
 *      Author: Joshua Russell
 */

#ifndef _LSM9DS1_H_
#define _LSM9DS1_H_

//----Includes----//
#include "headers/drivers/i2c.h"

//----Register Defines----//

/*These were copied from:
LSM9DS1_Registers.h
SFE_LSM9DS1 Library - LSM9DS1 Register Map
Jim Lindblom @ SparkFun Electronics
Original Creation Date: April 21, 2015
https://github.com/sparkfun/LSM9DS1_Breakout
******************************************************************************/

/////////////////////////////////////////
// LSM9DS1 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
#define ACT_THS             0x04
#define ACT_DUR             0x05
#define INT_GEN_CFG_XL      0x06
#define INT_GEN_THS_X_XL    0x07
#define INT_GEN_THS_Y_XL    0x08
#define INT_GEN_THS_Z_XL    0x09
#define INT_GEN_DUR_XL      0x0A
#define REFERENCE_G         0x0B
#define INT1_CTRL           0x0C
#define INT2_CTRL           0x0D
#define WHO_AM_I_XG         0x0F
#define CTRL_REG1_G         0x10
#define CTRL_REG2_G         0x11
#define CTRL_REG3_G         0x12
#define ORIENT_CFG_G        0x13
#define INT_GEN_SRC_G       0x14
#define OUT_TEMP_L          0x15
#define OUT_TEMP_H          0x16
#define STATUS_REG_0        0x17
#define OUT_X_L_G           0x18
#define OUT_X_H_G           0x19
#define OUT_Y_L_G           0x1A
#define OUT_Y_H_G           0x1B
#define OUT_Z_L_G           0x1C
#define OUT_Z_H_G           0x1D
#define CTRL_REG4           0x1E
#define CTRL_REG5_XL        0x1F
#define CTRL_REG6_XL        0x20
#define CTRL_REG7_XL        0x21
#define CTRL_REG8           0x22
#define CTRL_REG9           0x23
#define CTRL_REG10          0x24
#define INT_GEN_SRC_XL      0x26
#define STATUS_REG_1        0x27
#define OUT_X_L_XL          0x28
#define OUT_X_H_XL          0x29
#define OUT_Y_L_XL          0x2A
#define OUT_Y_H_XL          0x2B
#define OUT_Z_L_XL          0x2C
#define OUT_Z_H_XL          0x2D
#define FIFO_CTRL           0x2E
#define FIFO_SRC            0x2F
#define INT_GEN_CFG_G       0x30
#define INT_GEN_THS_XH_G    0x31
#define INT_GEN_THS_XL_G    0x32
#define INT_GEN_THS_YH_G    0x33
#define INT_GEN_THS_YL_G    0x34
#define INT_GEN_THS_ZH_G    0x35
#define INT_GEN_THS_ZL_G    0x36
#define INT_GEN_DUR_G       0x37

///////////////////////////////
// LSM9DS1 Magneto Registers //
///////////////////////////////
#define OFFSET_X_REG_L_M    0x05
#define OFFSET_X_REG_H_M    0x06
#define OFFSET_Y_REG_L_M    0x07
#define OFFSET_Y_REG_H_M    0x08
#define OFFSET_Z_REG_L_M    0x09
#define OFFSET_Z_REG_H_M    0x0A
#define WHO_AM_I_M          0x0F
#define CTRL_REG1_M         0x20
#define CTRL_REG2_M         0x21
#define CTRL_REG3_M         0x22
#define CTRL_REG4_M         0x23
#define CTRL_REG5_M         0x24
#define STATUS_REG_M        0x27
#define OUT_X_L_M           0x28
#define OUT_X_H_M           0x29
#define OUT_Y_L_M           0x2A
#define OUT_Y_H_M           0x2B
#define OUT_Z_L_M           0x2C
#define OUT_Z_H_M           0x2D
#define INT_CFG_M           0x30
#define INT_SRC_M           0x31
#define INT_THS_L_M         0x32
#define INT_THS_H_M         0x33

////////////////////////////////
// LSM9DS1 WHO_AM_I Responses //
////////////////////////////////
#define WHO_AM_I_AG_RSP     0x68
#define WHO_AM_I_M_RSP      0x3D

//I might eventually incorporate the HAL a little more and this could be useful
//! \brief Defines the LSM9DS1 object
//!
typedef struct _LSM9DS1_Obj_
{
  I2C_Handle       i2cHandle;                  //!< the handle for the I2C Interface

} LSM9DS1_Obj;


//! \brief Defines the LSM9DS1 handle
//!
typedef struct _LSM9DS1_Obj_ *LSM9DS1_Handle;


//----Inline Functions----//
static inline void LSM9DS1_Write(I2C_Handle i2cHandle, uint16_t register_addr, uint16_t data ){
    //I2C_Obj *i2c = (I2C_Obj *)i2cHandle;



    while(I2C_isMasterBusy(i2cHandle));
    I2C_clearStopConditionDetection(i2cHandle);
    while(I2C_isMasterStopBitSet(i2cHandle));

    I2C_MasterControl(i2cHandle, I2C_Control_Single_TX, 0, 2);


    while(!((I2C_getStatus(i2cHandle) & (I2C_I2CSTR_XRDY_BITS | I2C_I2CSTR_ARDY_BITS))));
    // If a NACK occurred, SCL is held low and STP bit cleared
    if ( I2C_isNoAck(i2cHandle) )
    {
        I2C_setMasterStopBit(i2cHandle);  // send STP to end transfer
        I2C_clearNoAckBit(i2cHandle);     // clear NACK bit
        return;
    }
    I2C_putData(i2cHandle, register_addr);

    while(!((I2C_getStatus(i2cHandle) & (I2C_I2CSTR_XRDY_BITS | I2C_I2CSTR_ARDY_BITS))));
    // If a NACK occurred, SCL is held low and STP bit cleared
    if ( I2C_isNoAck(i2cHandle) )
    {
        I2C_setMasterStopBit(i2cHandle);  // send STP to end transfer
        I2C_clearNoAckBit(i2cHandle);     // clear NACK bit
        return;
    }
    I2C_putData(i2cHandle, data);

    while(!((I2C_getStatus(i2cHandle) & (I2C_I2CSTR_XRDY_BITS | I2C_I2CSTR_ARDY_BITS))));
    // If a NACK occurred, SCL is held low and STP bit cleared
    if ( I2C_isNoAck(i2cHandle) )
    {
        I2C_setMasterStopBit(i2cHandle);  // send STP to end transfer
        I2C_clearNoAckBit(i2cHandle);     // clear NACK bit
        return;
    }

    I2C_setMasterStopBit(i2cHandle);

    while(!I2C_isStopConditionDetected(i2cHandle));
} // end of accel_Write() function


static inline uint16_t LSM9DS1_Read(I2C_Handle i2cHandle, uint16_t register_add ){

    while(I2C_isMasterBusy(i2cHandle));
    I2C_clearStopConditionDetection(i2cHandle);
    while(I2C_isMasterStopBitSet(i2cHandle));

    I2C_MasterControl(i2cHandle, I2C_Control_Burst_TX_Start, 0, 1);


    while(!((I2C_getStatus(i2cHandle) & (I2C_I2CSTR_XRDY_BITS))));

    I2C_putData(i2cHandle, register_add);

    while(!((I2C_getStatus(i2cHandle) & (I2C_I2CSTR_ARDY_BITS))));
    I2C_MasterControl(i2cHandle, I2C_Control_Single_RX, 0, 1);

    if(I2C_isNoAck(i2cHandle)){
        I2C_clearNoAckBit(i2cHandle);
    }

    I2C_setMasterStopBit(i2cHandle);

    while(!I2C_isStopConditionDetected(i2cHandle));

    return I2C_getData(i2cHandle);

}

//----Functions----//
void LSM9DS1_Init(I2C_Handle i2cHandle);


#endif /* _LSM9DS1_H_ */
