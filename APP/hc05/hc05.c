#include "SysTick.h"             
#include "usart.h"              
#include "usart3.h"              
#include "hc05.h" 
#include "led.h" 
#include "string.h"     
#include "math.h"




//3?ê??ˉHC05?￡?é
//·μ???μ:0,3é1|;1,ê§°ü.
u8 HC05_Init(void)
{
    u8 retry=10,t;               
    u8 temp=1;
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);    //ê1?üPORTA
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;                 // ???ú????
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          //é?à-ê?è?
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //IO?ú?ù?è?a50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                     //?ù?Yéè?¨2?êy3?ê??ˉA15
     
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;                 // ???ú????
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;          //í?íìê?3?
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //IO?ú?ù?è?a50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                     //?ù?Yéè?¨2?êy3?ê??ˉGPIOA4

    GPIO_SetBits(GPIOA,GPIO_Pin_4);
     GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    HC05_KEY=1;
    HC05_LED=1; 
    
    USART3_Init(9600); 
        //3?ê??ˉ′??ú3?a:9600,2¨ì??ê.
    while (retry--)
        {
    HC05_KEY = 1; // KEY ??,?? AT ??
    delay_ms(10);
    u3_printf("AT\r\n"); // ?? AT ????
    HC05_KEY = 0; // KEY ??,?? AT ??
    for (t = 0; t < 10; t++) // ???? 50ms,??? HC05 ?????
    {
        if (USART3_RX_STA & 0X8000)
            break;
        delay_ms(5);
    }
    if (USART3_RX_STA & 0X8000) // ????????
    {
        temp = USART3_RX_STA & 0X7FFF; // ??????
        USART3_RX_STA = 0;
        if (temp == 4 && USART3_RX_BUF[0] == 'O' && USART3_RX_BUF[1] == 'K')
        {
            temp = 0; // ??? OK ??
            break;
        }
    }
}
if (retry == 0)
    temp = 1; // ????
return temp;
}
// ?? ATK-HC05 ?????
// ???:0,??;1,??;0XFF,????
u8 HC05_Get_Role(void)
{
    u8 retry = 0X0F;
    u8 temp, t;
    while (retry--)
    {
        HC05_KEY = 1; // KEY ??,?? AT ??
        delay_ms(10);
        u3_printf("AT+ROLE?\r\n"); // ????
        for (t = 0; t < 20; t++) // ???? 200ms,??? HC05 ?????
        {
            delay_ms(10);
            if (USART3_RX_STA & 0X8000)
                break;
        }
        HC05_KEY = 0; // KEY ??,?? AT ??
        if (USART3_RX_STA & 0X8000) // ????????
        {
            temp = USART3_RX_STA & 0X7FFF; // ??????
            USART3_RX_STA = 0;
            if (temp == 13 && USART3_RX_BUF[0] == '+') // ?????????
            {
                temp = USART3_RX_BUF[6] - '0'; // ???????
                break;
            }
        }
    }
    if (retry == 0)
        temp = 0XFF; // ????
    return temp;
}
// HC05 ????
// ??????? HC05,?????? OK ??? AT ??
// atstr:AT ???.??:"AT+RESET"/"AT+UART=9600,0,0"/"AT+ROLE=0"????
// ???:0,????;??,????
u8 HC05_Set_Cmd(u8 *atstr)
{
    u8 retry = 0X0F;
    u8 temp, t;
    while (retry--)
    {
        HC05_KEY = 1; // KEY ??,?? AT ??
        delay_ms(10);
        u3_printf("%s\r\n", atstr); // ?? AT ???
        HC05_KEY = 0; // KEY ??,?? AT ??
        for (t = 0; t < 20; t++) // ???? 100ms,??? HC05 ?????
        {
            if (USART3_RX_STA & 0X8000)
                break;
            delay_ms(5);
        }
        if (USART3_RX_STA & 0X8000) // ????????
        {
            temp = USART3_RX_STA & 0X7FFF; // ??????
            USART3_RX_STA = 0;
            if (temp == 4 && USART3_RX_BUF[0] == 'O') // ?????????
            {
                temp = 0;
                break;
            }
        }
    }
    if (retry == 0)
        temp = 0XFF; // ????
    return temp;
}
///////////////////////////////////////////////////////////////////////////////////////////////////
// str:???(??????????????)
void HC05_CFG_CMD(u8 *str)
{
    u8 temp;
    u8 t;
    HC05_KEY = 1; // KEY ??,?? AT ??
    delay_ms(10);
    u3_printf("%s\r\n", (char *)str); // ????
    for (t = 0; t < 50; t++) // ???? 500ms,??? HC05 ?????
    {
        if (USART3_RX_STA & 0X8000)
            break;
        delay_ms(10);
    }
    HC05_KEY = 0; // KEY ??,?? AT ??
    if (USART3_RX_STA & 0X8000) // ????????
    {
        temp = USART3_RX_STA & 0X7FFF; // ??????
        USART3_RX_STA = 0;
        USART3_RX_BUF[temp] = 0; // ????
        printf("\r\n%s", USART3_RX_BUF); // ?????????1
    }
}