#ifndef _USER_2MOTORS_H_
#define _USER_2MOTORS_H_
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

//! \file   solutions/instaspin_foc/boards/boostxldrv8301_revB/f28x/f2806xF/src/user.h
//! \brief  Contains the public interface for user initialization data for the CTRL, HAL, and EST modules 
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
#include "headers/modules/types.h"
#include "headers/modules/motor.h"
#include "headers/modules/est/est.h"
#include "headers/modules/est/est_states.h"
#include "headers/modules/est/est_Flux_states.h"
#include "headers/modules/est/est_Ls_states.h"
#include "headers/modules/est/est_Rs_states.h"
#include "headers/modules/ctrl_obj_2motors.h"

// platforms
#include "headers/modules/userParams_2motors.h"

#include "headers/modules/ctrl_2motors.h"

#include "headers/user_motor1.h"
#include "headers/user_motor2.h"

//!
//!
//! \defgroup USER USER
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines


//! \brief      Recalculates Inductances with the correct Q Format
//! \param[in]  handle       The controller (CTRL) handle
extern void USER_softwareUpdate1p6(CTRL_Handle handle, USER_Params *pUserParams);


//! \brief      Updates Id and Iq PI gains
//! \param[in]  handle       The controller (CTRL) handle
extern void USER_calcPIgains(CTRL_Handle handle, USER_Params *pUserParams);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _USER_H_ definition

