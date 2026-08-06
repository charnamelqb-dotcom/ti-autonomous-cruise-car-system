#include "uart.h"
#include "wit.h"
#include <math.h>
volatile unsigned char uart_data,uart_data2;
volatile unsigned char uart_data_temp[40];
extern volatile int32_t Left_Count ;
//串口发送单个字符
void uart0_send_char(char ch)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_0_INST) == true );
    //发送单个字符
    DL_UART_Main_transmitDataBlocking(UART_0_INST, ch);
}
//串口发送字符串
void uart0_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart0_send_char(*str++);
    }
}

//串口的中断服务函数
void UART_0_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_0_INST) ) 
    {
        case DL_UART_IIDX_RX://如果是接收中断
            //将发送过来的数据保存在变量中
            uart_data = DL_UART_Main_receiveData(UART_0_INST);

            //将保存的数据再发送出去
            uart0_send_char(uart_data);

            // uart0_send_char(uart_data[1]);
            // uart0_send_char(Left_Count);
            break;

        default://其他的串口中断
            break;
    }
}

// 全局变量需要调整为浮点型及辅助变量
float angle[3] = {0.0f};       // 改为浮点型数组
uint8_t index2 = 0;
uint8_t inFraction = 0;        // 是否处于小数部分(0:整数,1:小数)
uint8_t decimalPlaces = 0;     // 小数位数计数
uint8_t dataReceived = 0;      // 数据接收完成标志
uint8_t rxData=0;
void UART_OpenMv_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_OpenMv_INST) ) 
    {
        case DL_UART_IIDX_RX://如果是接收中断
            rxData = DL_UART_Main_receiveData(UART_OpenMv_INST);
        
        // 限制索引范围，防止数组越界
        if (index2 >= 3) {
            
            return;
        }
        
        if (rxData == ',') {
            // 切换到下一个数据，重置小数相关标志
            index2++;
            inFraction = 0;
            decimalPlaces = 0;
        } else if (rxData == '\n') {
            // 接收完成，重置所有标志
            dataReceived = 1;  // 新增：标记数据接收完成
            index2 = 0;
            inFraction = 0;
            decimalPlaces = 0;
        } else if (rxData == '.') {
            // 遇到小数点，切换到小数模式
            inFraction = 1;
        } else if (rxData >= '0' && rxData <= '9') {
            // 处理数字字符
            if (!inFraction) {
                // 整数部分处理
                angle[index2] = angle[index2] * 10 + (rxData - '0');
            } else {
                // 小数部分处理
                decimalPlaces++;
                angle[index2] += (rxData - '0') / pow (10, decimalPlaces);
            }
        }
            break;
        default://其他的串口中断
            break;
    }
}










void uartOpenMv_send_char(char ch)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_OpenMv_INST) == true );
    //发送单个字符
    DL_UART_Main_transmitDataBlocking(UART_OpenMv_INST,ch);
}
//串口发送字符串
void uartOpenMv_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uartOpenMv_send_char(*str++);
    }
}

//串口的中断服务函数
// void UART_OpenMv_INST_IRQHandler(void)
// {
//     //如果产生了串口中断
//     switch( DL_UART_getPendingInterrupt(UART_OpenMv_INST) ) 
//     {
//         case DL_UART_IIDX_RX://如果是接收中断
//             //将发送过来的数据保存在变量中
//             uart_data2 = DL_UART_Main_receiveData(UART_OpenMv_INST);
//             DL_GPIO_togglePins(GPIO_LED_PORT,GPIO_LED_LED_PIN);
//             //将保存的数据再发送出去
//             uart0_send_char(uart_data2);//用串口发给上位机

//             // uart0_send_char(uart_data[1]);
//             // uart0_send_char(Left_Count);
//             break;

//         default://其他的串口中断
//             break;
//     }
// }
// int fputc(int c, FILE* restrict stream)
// {
//     DL_UART_Main_transmitDataBlocking(UART_0_INST, c);
//     return c;

// }
void UART_WIT_INST_IRQHandler(void)
{
    uint8_t checkSum, packCnt = 0;
    extern uint8_t wit_dmaBuffer[33];

    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
        wit_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_WIT_INST);

    while(rxSize >= 11)
    {
        checkSum=0;
        for(int i=packCnt*11; i<(packCnt+1)*11-1; i++)
            checkSum += wit_dmaBuffer[i];

        if((wit_dmaBuffer[packCnt*11] == 0x55) && (checkSum == wit_dmaBuffer[packCnt*11+10]))
        {
            if(wit_dmaBuffer[packCnt*11+1] == 0x51)
            {
                wit_data.ax = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 2.048; //mg
                wit_data.ay = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 2.048; //mg
                wit_data.az = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 2.048; //mg
                wit_data.temperature =  (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]) / 100.0; //°C
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x52)
            {
                wit_data.gx = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 16.384; //°/S
                wit_data.gy = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 16.384; //°/S
                wit_data.gz = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 16.384; //°/S
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x53)
            {
                wit_data.roll  = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 32768.0 * 180.0; //°
                wit_data.pitch = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 32768.0 * 180.0; //°
                wit_data.yaw   = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 32768.0 * 180.0; //°
                wit_data.version = (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]);
            }
        }

        rxSize -= 11;
        packCnt++;
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_WIT_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}