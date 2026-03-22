#ifndef __AS5600_PWM_H__
#define __AS5600_PWM_H__
#include "main.h"
#include "adc.h"

#define MAX_VALUE 4095  // 输入范围 0~4095，所以最大值为 4096

// 各电机1024点位定义
#define MOTOR1_MIDPOINT 3584    // 电机1 1024点编码器值
//#define MOTR0_MIDPOINT 3761    // 电机2 1024点编码器值  
#define MOTOR3_MIDPOINT 412    // 电机3 1024点编码器值
//#define MOTR0_MIDPOINT 3159   // 电机4 1024点编码器值



#define PROCESS_VALUE(raw, zero) \
    (((raw) + 4096u - (((zero) + 3072u) & 0x0FFFu)) & 0x0FFFu)



extern void StarAndGetResult(void);

#endif
