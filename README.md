# 文件说明

# 代码逻辑
## 当switch0,mode0

​```
      motor_1_pid.Kp = 5;
			motor_3_pid.Kp = 5;
			// 目标角度
			const int16_t ago_targetL		=	1024;
			const int16_t ago_targetR		=	1024;

			
			// 从当前值缓慢移动到目标值
			static uint32_t last_smooth_time_mode0 = 0;
			const uint32_t SMOOTH_INTERVAL = 10; // 平滑间隔10ms
			const int16_t MAX_STEP = 70;         // 最大步进值
			
			if ((int32_t)(now - last_smooth_time_mode0) >= SMOOTH_INTERVAL)
			{
				// 电机0（左前）平滑移动
				int16_t current0 = Wings_Data.Wings_motor[0].Corrective_Angle;
				if (current0 < ago_targetL) {
					current0 += MIN(MAX_STEP, ago_targetL - current0);
				} else if (current0 > ago_targetL) {
					current0 -= MIN(MAX_STEP, current0 - ago_targetL);
				}
				Wings_Data.Wings_motor[0].Target_Angle = 2095;

	
				// 电机2（右前）平滑移动
				int16_t current2 = Wings_Data.Wings_motor[2].Corrective_Angle;
				if (current2 < ago_targetR) {
					current2 += MIN(MAX_STEP, ago_targetR - current2);
				} else if (current2 > ago_targetR) {
					current2 -= MIN(MAX_STEP, current2 - ago_targetR);
				}
				Wings_Data.Wings_motor[2].Target_Angle = 2005;

				
				last_smooth_time_mode0 = now;
			}
			// if(abs16_fast(ago_targetL-Wings_Data.Wings_motor[0].Corrective_Angle)>200&abs16_fast(ago_targetL-Wings_Data.Wings_motor[0].Corrective_Angle)>200)
			// {
			// motor_1_pid.Kp=12*(float)abs16_fast(ago_targetL-Wings_Data.Wings_motor[0].Corrective_Angle)/(float)abs16_fast(ago_targetR-Wings_Data.Wings_motor[2].Corrective_Angle);
			// motor_3_pid.Kp=12*(float)abs16_fast(ago_targetR-Wings_Data.Wings_motor[2].Corrective_Angle)/(float)abs16_fast(ago_targetL-Wings_Data.Wings_motor[0].Corrective_Angle);	
			// }
			StarAndGetResult();
			Motor_PID_Control();

			sm_idx = 0;						// 0..14
			sm_dir = 1;          			// +1 正向 / -1 反向
			sm_next_tick = HAL_GetTick();	// 下次步进的时刻（ms）
​```


