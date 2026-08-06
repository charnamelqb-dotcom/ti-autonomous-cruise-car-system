#include "motor.h"
#include "pid.h"
int L1,L2,L3,L4,R1,R2,R3,R4;
int add,stop,stop2;
int turn=0;
extern PID PosionPID;
extern PID cicriPID;
extern PID PosionPIDR;
extern PID cicriPIDR;
void xunjiopen()
{//白色为0
		if (L4==0&&L3==0&&L2==0&&L1 ==1 && R1 == 1&&R2 == 0 && R3 == 0&& R4 == 0)
		{
            add=0;
		}

		if (L4==0&&L3==0&&L2 ==0  && L1 == 1&& R1== 0&& R2 == 0&& R3 == 0&& R4 == 0)
        {
			add=2;			
		}	
		if (L4==0&&L3==0&&L2 == 1 && L1 == 1&& R1== 0&& R2 == 0&& R3 == 0&& R4 == 0)
        {   
            add=3;
		}	
		
		if (L4==0&&L3==0&&L2 == 1 && L1 == 0&& R1== 0&& R2 == 0&& R3 == 0&& R4 == 0)
        {
            add=4;
		}

        if (L4==0&&L3==1&&L2 == 1 && L1 == 1&& R1==0 && R2 == 0&& R3== 0&& R4 == 0)//多加
        {
            add=5;
		}	
		
		if (L4==0&&L3==1&&L2 == 1 && L1 == 0&& R1== 0&& R2 == 0&& R3 == 0&& R4 == 0)
        {   
            add=6; 
        }	
        


		
		if (L4==0&&L3==1&&L2 == 0 && L1 == 0&& R1==0 && R2 == 0&& R3== 0&& R4 == 0)
        {
            add=7;
		}	



        		//再写多种情况
		if (L4==0&&L3==0&&L2 == 0 && L1 == 0&& R1== 1&& R2 == 0&& R3 == 0&& R4 == 0)
        {
			add=-2;
		}	
		if (L4==0&&L3==0&&L2 == 0 && L1 == 0&& R1== 1&& R2 == 1&& R3 == 0&& R4 == 0)
        {
			add=-3;	
		}	
		if (L4==0&&L3==0&&L2 == 0 && L1 == 0&& R1== 0&& R2 == 1&& R3 == 0&& R4 == 0)
        {
        	add=-4;

		}

        if (L4==0&&L3==0&&L2 == 0 && L1 == 0&& R1== 1&& R2 == 1&& R3 == 1&& R4 == 0)
        {
			add=-5;
		}		
		


		if (L4==0&&L3==0&&L2 == 0 && L1 == 0&& R1== 0&& R2 == 1&& R3 == 1&& R4 == 0)
        {
			add=-6;
		}		
		
		if (L4==0&&L3==0&&L2== 0 && L1 == 0&& R1== 0&& R2 == 0&& R3 == 1&& R4 == 0)
        {
			add=-7;
		}		
		
		if (L4==1&&L3==1&&L2 == 1 && L1 == 1&& R1== 1&& R2 == 1&& R3 == 1&& R4 == 1)
        {
            // PosionPID.target_val=0;
            // PosionPIDR.target_val=0;
            
		}

        if (L4==1&&L3==1&&L2== 1 && L1 == 1&& R1== 0&& R2 == 0&& R3 == 0&& R4 == 0)//左转
        {   
            stop=1;
			turn=1;
		}	
        if (L4==0&&L3==0&&L2== 0 && L1 == 0&& R1== 1&& R2 == 1&& R3 == 1&& R4 == 1)//右转
        {
            stop=1;
			turn=2;
		}			



}
void read_sensors()
{       

    if (DL_GPIO_readPins(GPIO_Traking_left3_PORT,GPIO_Traking_left3_PIN)) {//最左PA28/35
        L3=1;
    }
    else {
        L3=0;
    }
    if (DL_GPIO_readPins(GPIO_Traking_left2_PORT,GPIO_Traking_left2_PIN)) {//PA31
        L2=1;
    }
    else {
        L2=0;
    }
    if (DL_GPIO_readPins(GPIO_Traking_left1_PORT,GPIO_Traking_left1_PIN)) {//PA8
        L1=1;
    }
    else {
        L1=0;
    }
    if (DL_GPIO_readPins(GPIO_Traking_right1_PORT,GPIO_Traking_right1_PIN)) {//PB26
        R1=1;
    }
    else {
        R1=0;
    }
    if (DL_GPIO_readPins(GPIO_Traking_right2_PORT,GPIO_Traking_right2_PIN)) {//PB27
        R2=1;
    }
    else {
        R2=0;
    }
    if (DL_GPIO_readPins(GPIO_Traking_right3_PORT,GPIO_Traking_right3_PIN)) {//最右PA29
        R3=1;
    }
    else {
        R3=0;
    }
	// L0 = DL_GPIO_readPins(GPIO_Traking_left3_PORT,GPIO_Traking_left3_PIN);
    // L1 = DL_GPIO_readPins(GPIO_Traking_left2_PORT,GPIO_Traking_left2_PIN);
    // L2 = DL_GPIO_readPins(GPIO_Traking_left1_PORT,GPIO_Traking_left1_PIN);
	// R0 = DL_GPIO_readPins(GPIO_Traking_right1_PORT,GPIO_Traking_right1_PIN);
    // R1 = DL_GPIO_readPins(GPIO_Traking_right2_PORT,GPIO_Traking_right2_PORT);
    // R2 = DL_GPIO_readPins(GPIO_Traking_right3_PORT,GPIO_Traking_right3_PORT);
}






void motor_speed(uint8_t side,float duty)
{
    uint32_t CompareValue = 0;
    if(side==0)//left
    {
        if(duty<0)
        {
            CompareValue = 3199 - 3199 *(-duty/100.0);
            DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,CompareValue,GPIO_PWM_MOTOR_C0_IDX);
            DL_GPIO_setPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Left1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Left2_PIN);
        }
        else if(duty>0)
        {
            CompareValue = 3199 - 3199 *(duty/100.0);
            DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,CompareValue,GPIO_PWM_MOTOR_C0_IDX);
            DL_GPIO_setPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Left2_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Left1_PIN);
        }
        else 
        {
            DL_GPIO_clearPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Left1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Left2_PIN);
        }
    }
    else if(side==1)//right
    {
        if(duty<0)
        {
            CompareValue = 3199 - 3199 *(-duty/100.0);
            DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,CompareValue,GPIO_PWM_MOTOR_C1_IDX);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Righ2_mirror_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Right1_PIN);
        }
        else if(duty>0)
        {
            CompareValue = 3199 - 3199 *(duty/100.0);
            DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,CompareValue,GPIO_PWM_MOTOR_C1_IDX);
            // DL_GPIO_setPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Right2_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Righ2_mirror_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Right1_PIN);
        }
        else 
        {
            // DL_GPIO_clearPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Right2_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Right1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Righ2_mirror_PIN);
        }
    }
    else if (side==3) {
        if(duty<0)
        {
            CompareValue = 3199 - 3199 *(-duty/100.0);
            DL_TimerA_setCaptureCompareValue(PWM_SEROR_NEW_INST ,CompareValue,GPIO_PWM_SEROR_NEW_C0_IDX);
            DL_GPIO_clearPins(GPIO_MOTOR_NEW_PORT , GPIO_MOTOR_NEW_PIN_NEW_1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_NEW_PORT , GPIO_MOTOR_NEW_PIN_NEW_2_PIN);
        }
        else if(duty>0)
        {
            CompareValue = 3199 - 3199 *(duty/100.0);
            DL_TimerA_setCaptureCompareValue(PWM_SEROR_NEW_INST,CompareValue,GPIO_PWM_SEROR_NEW_C0_IDX);
            // DL_GPIO_setPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Right2_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_NEW_PORT , GPIO_MOTOR_NEW_PIN_NEW_1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_NEW_PORT , GPIO_MOTOR_NEW_PIN_NEW_2_PIN);
        }
        else 
        {
            // DL_GPIO_clearPins(GPIO_MOTOR_PORT , GPIO_MOTOR_Right2_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_NEW_PORT , GPIO_MOTOR_NEW_PIN_NEW_1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_NEW_PORT , GPIO_MOTOR_NEW_PIN_NEW_2_PIN);
        }
    }
}
void Set_Angle(uint8_t side, float duty)//2.5__-90//5__-45//7.5__0//10__45//12.5__90
{
    uint32_t compareValue = 0;
    if(side == 0)
    {

            compareValue = 20000 * (duty/100.0);
            DL_TimerG_setCaptureCompareValue(PWM_SEROR_INST, compareValue, GPIO_PWM_SEROR_C0_IDX);          
    }
}

