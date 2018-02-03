#ifndef _MAIN_2MTR_H_
#define _MAIN_2MTR_H_
/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//! \file   solutions/instaspin_foc/src/main_2motors.h
//! \brief Defines the structures, global initialization, and functions used in MAIN 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.

// **************************************************************************
// the includes

// modules
#include "headers/modules/math.h"
#include "headers/modules/memCopy.h"
#include "headers/modules/est/est.h"
#include "headers/modules/svgen_current.h"
#include "headers/modules/fw.h"
#include "headers/modules/fem.h"
#include "headers/modules/cpu_usage.h"

#include "headers/modules/flyingStart_2motors.h"
#include "headers/modules/cpu_time.h"

#include "headers/modules/clarke.h"
#include "headers/modules/park.h"
#include "headers/modules/ipark.h"
#include "headers/modules/motor.h"
#include "headers/modules/offset.h"
#include "headers/modules/pid.h"
#include "headers/modules/svgen.h"
#include "headers/modules/traj.h"
#include "headers/modules/angle_comp.h"


// drivers


// platforms
#include "headers/modules/ctrl_2motors.h"

#include "headers/modules/hal/hal_2motors.h"

#include "headers/user_2motors.h"

// **************************************************************************
// the defines


//! \brief Defines the number of main iterations before global variables are updated
//!
#define NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE  1

//! \brief Defines the speed acceleration scale factor.
//!
#define MAX_ACCEL_KRPMPS_SF_M1  _IQ(USER_MOTOR_NUM_POLE_PAIRS_M1*1000.0/USER_TRAJ_FREQ_Hz_M1/USER_IQ_FULL_SCALE_FREQ_Hz_M1/60.0)

#define MAX_ACCEL_KRPMPS_SF_M2  _IQ(USER_MOTOR_NUM_POLE_PAIRS_M2*1000.0/USER_TRAJ_FREQ_Hz_M2/USER_IQ_FULL_SCALE_FREQ_Hz_M2/60.0)

//! \brief Initialization values of global variables
//!
#define MOTOR_Vars_INIT_Mtr1  {false, \
                               false, \
                               false, \
                               true, \
                               false, \
                               false, \
                               true, \
                               true, \
                               false, \
							   \
                               false, \
							   true, \
							   \
							   false, \
						       false, \
						 	   false, \
                               CTRL_State_Idle, \
                               EST_State_Idle, \
                               USER_ErrorCode_NoError, \
                               {0,CTRL_TargetProc_Unknown,0,0}, \
                               _IQ(0.0), \
                               _IQ(0.0), \
							   _IQ(0.6), \
                               _IQ(0.0), \
                               _IQ(0.6), \
                               _IQ(0.0), \
                               _IQ(0.2), \
                               _IQ(0.0), \
                               _IQ(USER_MAX_VS_MAG_PU_M1), \
                               _IQ(0.1 * USER_MOTOR_MAX_CURRENT_M1), \
                               400, \
                               _IQ(0.0), \
                               _IQ(0.0), \
							   \
							   _IQ(0.0), \
                               _IQ(0.0), \
							   \
                               0.0, \
                               0.0, \
                               0.0, \
                               0.0, \
                               0.0, \
                               0.0, \
                               0.0, \
                               0.0, \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               0.0, \
                               0.0, \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.8 * USER_MAX_VS_MAG_PU_M1), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               {0,0,0}, \
                               {0,0,0}}


#define MOTOR_Vars_INIT_Mtr2  {false, \
                               false, \
                               false, \
                               true, \
                               false, \
                               false, \
                               true, \
                               true, \
                               false, \
							   \
                               false, \
							   true, \
							   \
							   false, \
						 	   false, \
						 	   false, \
                               CTRL_State_Idle, \
                               EST_State_Idle, \
                               USER_ErrorCode_NoError, \
                               {0,CTRL_TargetProc_Unknown,0,0}, \
                               _IQ(0.0), \
                               _IQ(0.0), \
							   _IQ(0.6), \
                               _IQ(0.0), \
                               _IQ(0.9), \
                               _IQ(0.0), \
                               _IQ(0.2), \
                               _IQ(0.0), \
                               _IQ(USER_MAX_VS_MAG_PU_M2), \
                               _IQ(0.1 * USER_MOTOR_MAX_CURRENT_M2), \
                               400, \
                               _IQ(0.0), \
                               _IQ(0.0), \
							   \
							   _IQ(0.0), \
                               _IQ(0.0), \
							   \
                               0.0, \
                               0.0, \
                               0.0, \
                               0.0, \
                               0.0, \
                               0.0, \
                               0.0, \
                               0.0, \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               0.0, \
                               0.0, \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.8 * USER_MAX_VS_MAG_PU_M2), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               _IQ(0.0), \
                               {0,0,0}, \
                               {0,0,0}, \
							   0, 0, 0, \
							   0, 0 \
}

// **************************************************************************
// the typedefs

typedef struct _MOTOR_Vars_t_
{
  bool Flag_enableSys;
  bool Flag_Run_Identify;
  bool Flag_MotorIdentified;
  bool Flag_enableForceAngle;
  bool Flag_enableFieldWeakening;
  bool Flag_enableRsRecalc;
  bool Flag_enableUserParams;
  bool Flag_enableOffsetcalc;
  bool Flag_enablePowerWarp;

  bool Flag_enableSpeedCtrl;
  bool Flag_enableDcBusComp;

  bool Flag_enableRun;
  bool Flag_RunState;
  bool Flag_enableFlyingStart;

  CTRL_State_e CtrlState;
  EST_State_e EstState;

  USER_ErrorCode_e UserErrorCode;

  CTRL_Version CtrlVersion;

  _iq IdRef_A;
  _iq IqRef_A;
  _iq SpeedSet_krpm;
  _iq SpeedRef_pu;
  _iq SpeedRef_krpm;
  _iq SpeedTraj_krpm;
  _iq MaxAccel_krpmps;
  _iq Speed_krpm;
  _iq OverModulation;
  _iq RsOnLineCurrent_A;
  _iq SvgenMaxModulation_ticks;
  _iq Flux_Wb;
  _iq Torque_Nm;

  _iq angle_est_pu;
  _iq speed_est_pu;

  float_t MagnCurr_A;
  float_t Rr_Ohm;
  float_t Rs_Ohm;
  float_t RsOnLine_Ohm;
  float_t Lsd_H;
  float_t Lsq_H;
  float_t Flux_VpHz;

  float_t ipd_excFreq_Hz;
  _iq     ipd_Kspd;
  _iq     ipd_excMag_coarse_pu;
  _iq     ipd_excMag_fine_pu;
  float   ipd_waitTime_coarse_sec;
  float   ipd_waitTime_fine_sec;

  _iq Kp_spd;
  _iq Ki_spd;

  _iq Kp_Idq;
  _iq Ki_Idq;

  _iq Vd;
  _iq Vq;
  _iq Vs;
  _iq VsRef;
  _iq VdcBus_kV;

  _iq Id_A;
  _iq Iq_A;
  _iq Is_A;

  MATH_vec3 I_bias;
  MATH_vec3 V_bias;

  _iq voltage_pu_to_kv_sf;
  _iq current_pu_to_A_sf;
  _iq speed_pu_to_krpm_sf;

  _iq current_A_to_pu_sf;
  _iq speed_krpm_to_pu_sf;
}MOTOR_Vars_t;


//------------------------------------------------------------------------
#define SYSTEM_Vars_INIT  { \
		false, \
        false, \
        false, \
		_IQ(0.6), \
		_IQ(0.30) \
}

typedef struct _SYSTEM_Vars_t_
{
	bool Flag_enableSystem;
	bool Flag_enableSynControl;
	bool Flag_enableRun;

	_iq SpeedSet_krpm;
	_iq MaxAccelSet_krpmps;
}SYSTEM_Vars_t;

// **************************************************************************
// the globals

extern volatile MOTOR_Vars_t gMotorVars[2];
extern volatile SYSTEM_Vars_t gSystemVars;

// **************************************************************************
// the function prototypes

//! \brief The main interrupt service (ISR) routine
//!
interrupt void motor1_ISR(void);
interrupt void motor2_ISR(void);

void pidSetup(HAL_MtrSelect_e mtrNum);


void runCurrentIgnore(void);


void runCurrentReconstruction(void);


void runFieldWeakening(void);


void runOffsetsCalculation(HAL_MtrSelect_e mtrNum);


void runSetTrigger(void);


//! \brief     Updates the global motor variables 
//! 
void updateGlobalVariables_motor(CTRL_Handle handle, const uint_least8_t mtrNum);


//! \brief     Updates the global variables 
//! 
void updateGlobalVariables(EST_Handle handle, const uint_least8_t mtrNum);


//! \brief     Updates version 1p6 of library
//!
void softwareUpdate1p6(EST_Handle handle,USER_Params *pUserParams);


//! \brief     Reset Ls Q format to a higher value when Ls identification starts
//!
void CTRL_resetLs_qFmt(CTRL_Handle handle, const uint_least8_t qFmt);


//! \brief     Recalculate Kp and Ki gains to fix the R/L limitation of 2000.0 and Kp limitation of 0.11
//! \brief     as well as recalculates gains based on estimator state to allow low inductance pmsm to id
//!
void recalcKpKiPmsm(CTRL_Handle handle);


//! \brief     Calculates the maximum qFmt value for Ls identification, to get a more accurate Ls per unit
//!
void CTRL_calcMax_Ls_qFmt(CTRL_Handle handle, uint_least8_t *p_qFmt);


//! \brief     Updates Iq reference and also sets the right sign to the speed reference for correct force angle
//!
void updateIqRef(CTRL_Handle handle);


//! \brief     Updates Kp and Ki gains in the controller object
//!
void updateKpKiGains(CTRL_Handle handle, const uint_least8_t mtrNum);


//! \brief     Recalculate Kp and Ki gains to fix the R/L limitation of 2000.0 and Kp limitation of 0.11
//!
void recalcKpKi(CTRL_Handle handle, const uint_least8_t mtrNum);

//! \brief     Runs Rs online
//!
void runRsOnLine(CTRL_Handle handle);


//! \brief     Updates CPU usage
//!
void updateCPUusage(void);


//! \brief     Set electrical frequency limit to zero while identifying an induction motor
//!
void setFeLimitZero(CTRL_Handle handle);


//! \brief     Calculates Dir_qFmt for ACIM
//!
void acim_Dir_qFmtCalc(CTRL_Handle handle);


//! \brief     Sets up the Clarke transform for current
//!
void setupClarke_I(CLARKE_Handle,const uint_least8_t);


//! \brief     Sets up the Clarke transform for voltage
//!
void setupClarke_V(CLARKE_Handle,const uint_least8_t);


CTRL_Handle CTRL_init(void *pMemory,const size_t numBytes);

void EST_getIdq_pu(EST_Handle handle,MATH_vec2 *pIdq_pu);

#if !defined(FAST_ROM_V1p6)
void EST_setEstParams(EST_Handle handle,USER_Params *pUserParams);
void EST_setupEstIdleState(EST_Handle handle);
#endif

EST_Handle EST_init(void *pMemory,const size_t numBytes);

//! \brief     Runs PowerWarp
//! \param[in] handle     The estimator (EST) handle
//! \param[in] Id_int_pu  The intermediate value along the Id trajectory in pu
//! \param[in] Iq_pu      The measured Iq value in pu
//! \return    The target value for the Id trajectory in pu
_iq EST_runPowerWarp(EST_Handle handle,const _iq Id_int_pu,const _iq Iq_pu);

_iq getAbsElecAngle(const _iq angle_pu);

_iq getAbsMechAngle(_iq *pAngle_mech_poles, _iq *pAngle_z1_pu, const _iq angle_pu);

void motor_RunCtrl(CTRL_Handle ctrlshandle, const uint_least8_t mtrNum);

//@} //defgroup
#endif // end of _MAIN_H_ definition



