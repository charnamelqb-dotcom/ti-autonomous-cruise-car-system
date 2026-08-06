#include "encoder.h"
extern volatile int32_t Left_Count ;
extern volatile int32_t Right_Count ;
extern volatile int32_t NEW_Count;
void GROUP1_IRQHandler(void)
{
    uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(GPIOA, GPIO_Encoder_PIN_Left_A_PIN|GPIO_Encoder_PIN_Right_A_PIN);
    uint32_t gpioB2 = DL_GPIO_getEnabledInterruptStatus(GPIOB, GPIO_Encoder_PIN_NEW_A_PIN);

    if (gpioA & GPIO_Encoder_PIN_Left_A_PIN) 
    {
        if(DL_GPIO_readPins(GPIO_Encoder_PIN_Left_B_PORT, GPIO_Encoder_PIN_Left_B_PIN))
            Left_Count--;
        else 
            Left_Count++;
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_PIN_Left_A_PORT, GPIO_Encoder_PIN_Left_A_PIN);
    }

    if (gpioA & GPIO_Encoder_PIN_Right_A_PIN) 
    {
        if(DL_GPIO_readPins(GPIO_Encoder_PIN_Right_B_PORT, GPIO_Encoder_PIN_Right_B_PIN))
            Right_Count--;
        else 
            Right_Count++;
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_PIN_Right_A_PORT, GPIO_Encoder_PIN_Right_A_PIN);
    }

    if (gpioB2 & GPIO_Encoder_PIN_NEW_A_PIN) {

         if(DL_GPIO_readPins(GPIO_Encoder_PIN_NEW_B_PORT, GPIO_Encoder_PIN_NEW_B_PIN))
            NEW_Count--;
        else 
            NEW_Count++;
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_PIN_NEW_A_PORT, GPIO_Encoder_PIN_NEW_A_PIN);


    }

}
