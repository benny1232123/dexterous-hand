#ifndef GRIPPER_TASK_H
#define GRIPPER_TASK_H

#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

void GripperTask_Entry(void const *argument);

/* 1 kHz control step, called from the TIM1 update interrupt */
void GripperControl_OnTimer(void);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_TASK_H */
