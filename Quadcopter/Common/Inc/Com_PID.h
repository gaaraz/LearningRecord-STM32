#ifndef __COM_PID_H__
#define __COM_PID_H__


/* Includes */
#include "main.h"
#include "Com_Debug.h"
#include "Com_Config.h"

void Com_PID_ComputePID(PIDStruct *pid);

void Com_PID_CascadePID(PIDStruct *out, PIDStruct *in);

#endif