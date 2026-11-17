#ifndef __COMMAND_H
#define __COMMAND_H

#include "comm.h"

void Command_ProcessInput(CommSource source);
void Command_UpdateCalibration(uint16_t elapsed_ms);

#endif
