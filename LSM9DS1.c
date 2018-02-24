/*
 * LSM9DS1.c
 *
 *  Created on: Feb 23, 2018
 *      Author: Joshua Russell
 */

//----Includes----//
#include "headers/drivers/LSM9DS1.h"



//----Functions----//
void LSM9DS1_Init(I2C_Handle i2cHandle){

    //  CTRL_REG5_XL (0x1F) (Default value: 0x38)
    //  [DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
    //  DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
    //      00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
    //  Zen_XL - Z-axis output enabled
    //  Yen_XL - Y-axis output enabled
    //  Xen_XL - X-axis output enabled

    uint16_t tempValue = 0x00;
    tempValue |= (1<<5);  //Enable Z
    //tempValue |= (1<<4);  //Enable Y
    tempValue |= (1<<3);  //Enable X
    LSM9DS1_Write(i2cHandle, CTRL_REG5_XL, tempValue );
//
//    // CTRL_REG6_XL (0x20) (Default value: 0x00)
//    // [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
//    // ODR_XL[2:0] - Output data rate & power mode selection
//    // FS_XL[1:0] - Full-scale selection
//    // BW_SCAL_ODR - Bandwidth selection
//    // BW_XL[1:0] - Anti-aliasing filter bandwidth selection
    tempValue = 0x00;
    tempValue |= (0x80);//(6 & 0x07) << 5;  // 6 is the value for 952 Hz - highest sample rate
    // Scale defaults to 2g (0x0 << 3)
    LSM9DS1_Write(i2cHandle, CTRL_REG6_XL, tempValue );

//    // CTRL_REG7_XL (0x21) (Default value: 0x00)
//    // [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
//    // HR - High resolution mode (0: disable, 1: enable)
//    // DCF[1:0] - Digital filter cutoff frequency
//    // FDS - Filtered data selection
//    // HPIS1 - HPF enabled for interrupt function

}






