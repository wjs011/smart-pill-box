#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "tftlcd.h"
#include "time.h"
#include "key.h"
#include "rtc.h"
#include "stdlib.h"
#include "stdio.h"  // 添加stdio.h支持printf
#include "string.h" // 添加string.h支持memcpy和memset
#include "dht11.h"  // 温湿度传感器驱动
#include "beep.h"   // 蜂鸣器驱动
#include "hc05.h"
#include "usart3.h"
#include "stm32f10x_exti.h" // 外部中断相关
#include "misc.h"           // NVIC相关
#include "hwjs.h"
// 系统状态定义
#define STATE_NORMAL 0
#define STATE_ALARM 1
#define STATE_MED_TAKEN 2
#define STATE_ENV_ALERT 3

// 蓝牙控制命令定义
#define BT_CMD_LED1_ON "LED1_ON"
#define BT_CMD_LED1_OFF "LED1_OFF"
#define BT_CMD_LED2_ON "LED2_ON"
#define BT_CMD_LED2_OFF "LED2_OFF"
#define BT_CMD_BEEP_ON "BEEP_ON"
#define BT_CMD_BEEP_OFF "BEEP_OFF"
#define BT_CMD_STATUS "STATUS"
#define BT_CMD_MED_CHECK "MED_CHECK"
#define BT_CMD_ENV_CHECK "ENV_CHECK"


// 红外遥控按键编码
#define IR_KEYa 0x00FFA25D // 按键开关的编码
#define IR_KEYb 0x00FF629D // 按键mode的编码
#define IR_KEYc 0x00FFE21D // 按键静音的编码
#define IR_KEYd 0x00FF22DD // 按键快进的编码
#define IR_KEYe 0x00FF02FD // 按键上一首的编码
#define IR_KEYf 0x00FFC23D // 按键下一首的编码
#define IR_KEYg 0x00FFE01F // 按键EQ的编码
#define IR_KEYh 0x00FFA857 // 按键VOL-的编码
#define IR_KEYi 0x00FF906F // 按键VOL+的编码
#define IR_KEY0 0x00FF6897 // 按键0的编码
#define IR_KEYj 0x00FF9867 // 按键RPT的编码
#define IR_KEYk 0x00FFB04F // 按键U/SD的编码
#define IR_KEY1 0x00FF30CF // 按键1的编码（根据实际遥控器可能需要修改）
#define IR_KEY2 0x00FF18E7 // 按键2的编码（根据实际遥控器可能需要修改）
#define IR_KEY3 0x00FF7A85 // 按键3的编码
#define IR_KEY4 0x00FF10EF // 按键4的编码
#define IR_KEY5 0x00FF38C7 // 按键5的编码
#define IR_KEY6 0x00FF5AA5 // 按键6的编码
#define IR_KEY7 0x00FF42BD // 按键7的编码 
#define IR_KEY8 0x00FF4AB5 // 按键8的编码 
#define IR_KEY9 0x00FF52AD // 按键9的编码

// 药物信息结构体
typedef struct
{
    char name[16]; // 药物名称
    u8 hour;       // 服药小时(24小时制)
    u8 minute;     // 服药分钟
    u8 taken;      // 是否已服药
} Medicine;

// 系统状态结构体
struct SystemState
{
    u8 current_state;  // 当前系统状态
    u8 env_alert;      // 环境警报标志
    u8 med_count;      // 药物数量
    u8 next_med_index; // 下一个要服用的药物索引
    float temperature; // 当前温度
    float humidity;    // 当前湿度
    u8 bt_led1_ctrl;   // 蓝牙控制LED1状态
    u8 bt_led2_ctrl;   // 蓝牙控制LED2状态
    u8 bt_beep_ctrl;   // 蓝牙控制蜂鸣器状态
} system_state;

// 药物时间表(可根据需要修改)
Medicine medicines[] = {
    {"Vitamins", 8, 0, 0},
    {"Anti_drugs", 12, 30, 0},
    {"Calcium_tablets", 19, 0, 0}};

// 红外传感器计数器(用于检测取药动作)
u32 ir_sensor_count = 0;

// 全局变量声明(添加在文件顶部变量声明区域)
// 屏幕状态管理变量
u8 last_screen = 0xFF;         // 上一次显示的屏幕
u8 last_state = 0xFF;          // 上一次的系统状态
u8 last_minute_display = 0xFF; // 上一次显示更新的分钟数
u8 force_refresh = 1;          // 强制刷新标志

// 蓝牙相关外部变量声明
extern u8 USART3_RX_BUF[USART3_MAX_RECV_LEN];
extern vu16 USART3_RX_STA;

// HC05状态变量
u8 bt_send_mask = 0; // 蓝牙发送状态标志
u8 bt_send_cnt = 0;  // 蓝牙发送计数器

// 函数声明
void show_home_screen(void);
void show_medication_screen(void);
void show_environment_screen(void);
void show_alert_screen(u8 alert_type);
void bluetooth_data_process(void);     // 蓝牙数据处理函数
void bluetooth_cmd_handler(char *cmd); // 蓝牙命令处理函数
void send_device_status(void);         // 发送设备状态函数
void simple_bluetooth_test(void);      // 简单蓝牙测试函数
void HC05_Role_Show(void);             // 显示HC05主从状态
void HC05_Sta_Show(void);              // 显示HC05连接状态

// 文件顶部声明外部函数
void USART1_Init(u32 bound);
void USART3_Init(u32 bound);
void BEEP_Init(void);
u8 HC05_Init(void);
void Bluetooth_Send(const char *msg);

// 系统初始化函数
void system_init(void)
{
    system_state.current_state = STATE_NORMAL;
    system_state.env_alert = 0;
    system_state.med_count = sizeof(medicines) / sizeof(Medicine);
    system_state.next_med_index = 0;
    system_state.temperature = 0.0;
    system_state.humidity = 0.0;
    system_state.bt_led1_ctrl = 0; // 初始化蓝牙LED控制状态
    system_state.bt_led2_ctrl = 0;
    system_state.bt_beep_ctrl = 0;
}

// 蓝牙数据处理函数
void bluetooth_data_process(void)
{
    static u32 check_counter = 0;
    static u16 last_rx_len = 0;
    u16 len;
    char recv_data[USART3_MAX_RECV_LEN];

    check_counter++; // 简单计数器，每次调用递增

    // 检查是否接收到一帧完整数据（方法1：检查完成标志）
    if (USART3_RX_STA & (1 << 15)) // 接收到一帧数据
    {
        len = USART3_RX_STA & 0x7FFF; // 获取数据长度

        // 复制接收到的数据并添加字符串结束符
        memcpy(recv_data, USART3_RX_BUF, len);
        recv_data[len] = '\0';

        // 串口调试输出 - 显示接收到的原始命令
        printf("BT Received (Method1): %s\r\n", recv_data);

        // 处理蓝牙命令
        bluetooth_cmd_handler(recv_data);

        // 清除接收标志和缓冲区
        USART3_RX_STA = 0;
        memset(USART3_RX_BUF, 0, USART3_MAX_RECV_LEN);
        last_rx_len = 0;
    }
    // 方法2：检查数据长度变化和超时（备用方案）
    else if ((USART3_RX_STA & 0x7FFF) > 0) // 有数据但未完成
    {
        len = USART3_RX_STA & 0x7FFF;

        // 如果数据长度变化，重置计数器
        if (len != last_rx_len)
        {
            check_counter = 0;
            last_rx_len = len;
        }
        // 如果数据长度超过500次检查没有变化，认为接收完成（约50ms，因为主循环100ms调用一次）
        else if (len > 0 && check_counter > 5)
        {
            // 查找换行符位置，确保命令完整
            u8 i;
            for (i = 0; i < len; i++)
            {
                if (USART3_RX_BUF[i] == '\r' || USART3_RX_BUF[i] == '\n')
                {
                    // 找到换行符，处理数据
                    memcpy(recv_data, USART3_RX_BUF, i);
                    recv_data[i] = '\0';

                    if (i > 0) // 确保不是空命令
                    {
                        printf("BT Received (Method2): %s\r\n", recv_data);
                        bluetooth_cmd_handler(recv_data);
                    }

                    // 清除接收标志和缓冲区
                    USART3_RX_STA = 0;
                    memset(USART3_RX_BUF, 0, USART3_MAX_RECV_LEN);
                    last_rx_len = 0;
                    break;
                }
            }
        }
    }
}

// 蓝牙命令处理函数
void bluetooth_cmd_handler(char *cmd)
{
    char response[100];

    // 去除命令中的换行符和回车符
    char *p = cmd;
    while (*p)
    {
        if (*p == '\r' || *p == '\n')
        {
            *p = '\0';
            break;
        }
        p++;
    }

    // LED1控制命令
    if (strcmp(cmd, BT_CMD_LED1_ON) == 0)
    {
        system_state.bt_led1_ctrl = 1;
        LED1 = 0; // LED低电平点亮
        Bluetooth_Send("LED1_ON_OK");
        printf("LED1 ON\r\n"); // 串口调试输出
    }
    else if (strcmp(cmd, BT_CMD_LED1_OFF) == 0)
    {
        system_state.bt_led1_ctrl = 0;
        LED1 = 1; // LED高电平熄灭
        Bluetooth_Send("LED1_OFF_OK");
        printf("LED1 OFF\r\n"); // 串口调试输出
    }
    // LED2控制命令
    else if (strcmp(cmd, BT_CMD_LED2_ON) == 0)
    {
        system_state.bt_led2_ctrl = 1;
        LED2 = 0; // LED低电平点亮
        Bluetooth_Send("LED2_ON_OK");
        printf("LED2 ON\r\n"); // 串口调试输出
    }
    else if (strcmp(cmd, BT_CMD_LED2_OFF) == 0)
    {
        system_state.bt_led2_ctrl = 0;
        LED2 = 1; // LED高电平熄灭
        Bluetooth_Send("LED2_OFF_OK");
        printf("LED2 OFF\r\n"); // 串口调试输出
    }
    // 蜂鸣器控制命令
    else if (strcmp(cmd, BT_CMD_BEEP_ON) == 0)
    {
        system_state.bt_beep_ctrl = 1;
        BEEP = 1; // 蜂鸣器响
        Bluetooth_Send("BEEP_ON_OK");
        printf("BEEP ON\r\n"); // 串口调试输出
    }
    else if (strcmp(cmd, BT_CMD_BEEP_OFF) == 0)
    {
        system_state.bt_beep_ctrl = 0;
        BEEP = 0; // 蜂鸣器关
        Bluetooth_Send("BEEP_OFF_OK");
        printf("BEEP OFF\r\n"); // 串口调试输出
    }
    // 状态查询命令
    else if (strcmp(cmd, BT_CMD_STATUS) == 0)
    {
        send_device_status();
        printf("STATUS QUERY\r\n"); // 串口调试输出
    }
    // 药物检查命令
    else if (strcmp(cmd, BT_CMD_MED_CHECK) == 0)
    {
        sprintf(response, "MED_INFO:%s,%02d:%02d,%s",
                medicines[system_state.next_med_index].name,
                medicines[system_state.next_med_index].hour,
                medicines[system_state.next_med_index].minute,
                medicines[system_state.next_med_index].taken ? "taken" : "not_taken");
        Bluetooth_Send(response);
        printf("MEDICINE CHECK\r\n"); // 串口调试输出
    }
    // 环境检查命令
    else if (strcmp(cmd, BT_CMD_ENV_CHECK) == 0)
    {
        sprintf(response, "ENV_INFO:%.1f,%.1f,%s",
                system_state.temperature, system_state.humidity,
                system_state.env_alert ? "alert" : "normal");
        Bluetooth_Send(response);
        printf("ENVIRONMENT CHECK\r\n"); // 串口调试输出
    }
    // 兼容原始格式的LED2控制命令
    else if (strcmp(cmd, "+LED2 ON") == 0)
    {
        system_state.bt_led2_ctrl = 1;
        LED2 = 0; // LED低电平点亮
        Bluetooth_Send("LED2_ON_OK");
        printf("LED2 ON (original format)\r\n"); // 串口调试输出
    }
    else if (strcmp(cmd, "+LED2 OFF") == 0)
    {
        system_state.bt_led2_ctrl = 0;
        LED2 = 1; // LED高电平熄灭
        Bluetooth_Send("LED2_OFF_OK");
        printf("LED2 OFF (original format)\r\n"); // 串口调试输出
    }
    else
    {
        // 未知命令
        sprintf(response, "UNKNOWN_CMD:%s", cmd);
        Bluetooth_Send(response);
        printf("UNKNOWN CMD: %s\r\n", cmd); // 串口调试输出
    }
}

// 发送设备状态函数
void send_device_status(void)
{
    char status_msg[200];
    sprintf(status_msg, "STATUS:LED1_%s,LED2_%s,BEEP_%s,TEMP_%.1f,HUMI_%.1f,STATE_%d",
            system_state.bt_led1_ctrl ? "ON" : "OFF",
            system_state.bt_led2_ctrl ? "ON" : "OFF",
            system_state.bt_beep_ctrl ? "ON" : "OFF",
            system_state.temperature,
            system_state.humidity,
            system_state.current_state);
    Bluetooth_Send(status_msg);
}

// 简单蓝牙测试函数
void simple_bluetooth_test(void)
{
    u16 len = USART3_RX_STA & 0x7FFF;
    u8 i;

    // 如果有数据且包含换行符，直接处理
    if (len > 0)
    {
        for (i = 0; i < len; i++)
        {
            if (USART3_RX_BUF[i] == '\r' || USART3_RX_BUF[i] == '\n')
            {
                if (i > 0) // 确保命令不为空
                {
                    char cmd[USART3_MAX_RECV_LEN];
                    memcpy(cmd, USART3_RX_BUF, i);
                    cmd[i] = '\0';

                    printf("Simple BT Test - Received: %s\r\n", cmd);
                    bluetooth_cmd_handler(cmd);
                }

                // 清除缓冲区
                USART3_RX_STA = 0;
                memset(USART3_RX_BUF, 0, USART3_MAX_RECV_LEN);
                break;
            }
        }
    }
}

// 显示HC05模块的主从状态
void HC05_Role_Show(void)
{
    if (HC05_Get_Role() == 1)
    {
        LCD_ShowString(10, 140, 200, 16, 16, (u8 *)"ROLE:Master"); // 主机
    }
    else
    {
        LCD_ShowString(10, 140, 200, 16, 16, (u8 *)"ROLE:Slave "); // 从机
    }
}

// 显示HC05模块的连接状态
void HC05_Sta_Show(void)
{
    if (HC05_LED)
    {
        LCD_ShowString(110, 140, 120, 16, 16, (u8 *)"STA:Connected "); // 连接成功
    }
    else
    {
        LCD_ShowString(110, 140, 120, 16, 16, (u8 *)"STA:Disconnect"); // 未连接
    }
}

// 检查服药时间函数
void check_medication_time(void)
{
    u8 current_hour = calendar.hour;
    u8 current_minute = calendar.min;
    u8 i;
    for (i = 0; i < system_state.med_count; i++)
    {
        if (!medicines[i].taken &&
            current_hour == medicines[i].hour &&
            current_minute == medicines[i].minute)
        {
            system_state.next_med_index = i;
            system_state.current_state = STATE_ALARM;
            break;
        }
    }
}

// 检查环境状态函数
void check_environment(void)
{
    u8 temp = 0, humi = 0;
    if (DHT11_Read_Data(&temp, &humi) == 0)
    {
        system_state.temperature = temp;
        system_state.humidity = humi;
        if (system_state.temperature > 30.0 || system_state.temperature < 10.0 ||
            system_state.humidity > 70.0)
        {
            system_state.env_alert = 1;
            system_state.current_state = STATE_ENV_ALERT;
        }
        else
        {
            system_state.env_alert = 0;
        }
    }
}

// 处理红外遥控器输入
void Process_IR_Command(void)
{
	char buf[32]; // 将变量声明移到函数开头

	if (hw_jsbz)
	{ // 有红外数据接收到
		printf("IR Code: 0x%08X\r\n", hw_jsm); // 打印接收到的红外代码

		// 根据红外代码控制LED2
		if (hw_jsm == IR_KEY1)
		{ // 按键1
			LED2 = 0; // 开LED2
			printf("IR Key 1: LED2 ON\r\n");

			// 更新显示
			LCD_Fill(10, 320, 220, 336, WHITE);
			sprintf(buf, "LED2: %s", LED2 ? "OFF" : "ON");
			LCD_ShowString(10, 320, 220, 16, 16, (u8 *)buf);
		}
		else if (hw_jsm == IR_KEY2)
		{ // 按键2
			LED2 = 1; // 关LED2
			printf("IR Key 2: LED2 OFF\r\n");

			// 更新显示
			LCD_Fill(10, 320, 220, 336, WHITE);
			sprintf(buf, "LED2: %s", LED2 ? "OFF" : "ON");
			LCD_ShowString(10, 320, 220, 16, 16, (u8 *)buf);
		}
		
		hw_jsbz = 0; // 处理完成，清除标志
	}
}

// 优化显示主界面函数
void show_home_screen(void)
{
    char time_str[20];
    char med_info[40];
    char env_status[30];

    // 只在需要时清屏和重绘
    if (force_refresh || last_screen != 0)
    {
        LCD_Clear(WHITE);
        FRONT_COLOR = BLUE;
        BACK_COLOR = WHITE;
        LCD_ShowString(60, 10, 200, 30, 24, (u8 *)"Smart Medicine Box");

        FRONT_COLOR = BLACK;
        LCD_ShowString(20, 200, 200, 16, 16, (u8 *)"KEY0:Medicine KEY1:Env");
        LCD_ShowString(20, 220, 200, 16, 16, (u8 *)"KEY2:Send KEY_UP:Role/Return");

        // 显示HC05状态信息
        FRONT_COLOR = BLUE;
        LCD_ShowString(10, 160, 200, 16, 16, (u8 *)"Send:");
        LCD_ShowString(10, 180, 200, 16, 16, (u8 *)"Receive:");
        HC05_Role_Show();
        HC05_Sta_Show();
        FRONT_COLOR = BLACK;

        last_screen = 0;
    }

    // 只在时间变化时更新时间显示（只在分钟变化时更新）
    if (force_refresh || last_minute_display != calendar.min)
    {
        sprintf(time_str, "%04d-%02d-%02d %02d:%02d", calendar.w_year, calendar.w_month, calendar.w_date, calendar.hour, calendar.min);
        // 先清除时间显示区域
        LCD_Fill(50, 50, 250, 80, WHITE);
        FRONT_COLOR = BLACK;
        LCD_ShowString(50, 50, 200, 30, 16, (u8 *)time_str);
        last_minute_display = calendar.min;
    }

    // 更新下一个服药信息（只在需要时）
    if (force_refresh)
    {
        if (system_state.next_med_index < system_state.med_count)
        {
            sprintf(med_info, "Next: %s %02d:%02d", medicines[system_state.next_med_index].name, medicines[system_state.next_med_index].hour, medicines[system_state.next_med_index].minute);
            LCD_Fill(30, 80, 230, 110, WHITE);
            LCD_ShowString(30, 80, 200, 30, 16, (u8 *)med_info);
        }

        // 更新环境状态
        if (system_state.env_alert)
        {
            sprintf(env_status, "Env Alert!");
            FRONT_COLOR = RED;
        }
        else
        {
            sprintf(env_status, "Env Normal");
            FRONT_COLOR = GREEN;
        }
        LCD_Fill(80, 110, 180, 140, WHITE);
        LCD_ShowString(80, 110, 200, 30, 16, (u8 *)env_status);
        FRONT_COLOR = BLACK;
    }
}

// 优化药物信息显示函数
void show_medication_screen(void)
{
    u8 y_pos = 50;
    u8 i;
    char med_info[50];
    u8 *status_str;

    // 只在屏幕切换时重新绘制
    if (force_refresh || last_screen != 1)
    {
        LCD_Clear(LGRAY);
        FRONT_COLOR = BLUE;
        BACK_COLOR = LGRAY;
        LCD_ShowString(80, 10, 200, 30, 24, (u8 *)"Medicine Info");

        for (i = 0; i < system_state.med_count; i++)
        {
            if (medicines[i].taken)
                status_str = (u8 *)"Taken";
            else
                status_str = (u8 *)"Not taken";
            sprintf(med_info, "%s %02d:%02d %s", medicines[i].name, medicines[i].hour, medicines[i].minute, status_str);
            FRONT_COLOR = medicines[i].taken ? GREEN : RED;
            LCD_ShowString(30, y_pos, 200, 30, 16, (u8 *)med_info);
            y_pos += 30;
        }
        FRONT_COLOR = BLACK;
        LCD_ShowString(20, 260, 200, 30, 16, (u8 *)"KEY_UP: Return");
        last_screen = 1;
    }
}

// 优化环境信息显示函数
void show_environment_screen(void)
{
    char temp_str[20];
    char humi_str[20];

    // 只在屏幕切换时重新绘制
    if (force_refresh || last_screen != 2)
    {
        LCD_Clear(LGRAY);
        FRONT_COLOR = BLUE;
        BACK_COLOR = LGRAY;
        LCD_ShowString(80, 10, 200, 30, 24, (u8 *)"Env Info");

        sprintf(temp_str, "Temp: %.1fC", system_state.temperature);
        sprintf(humi_str, "Humi: %.1f%%", system_state.humidity);
        FRONT_COLOR = BLACK;
        LCD_ShowString(80, 60, 200, 30, 24, (u8 *)temp_str);
        LCD_ShowString(80, 90, 200, 30, 24, (u8 *)humi_str);

        if (system_state.env_alert)
        {
            LCD_ShowString(60, 150, 200, 30, 24, (u8 *)"Env Alert!");
            LCD_ShowString(30, 180, 200, 30, 16, (u8 *)"Please check!");
            FRONT_COLOR = RED;
        }
        else
        {
            LCD_ShowString(80, 150, 200, 30, 24, (u8 *)"Env Normal");
            FRONT_COLOR = GREEN;
        }
        FRONT_COLOR = BLACK;
        LCD_ShowString(20, 260, 200, 30, 16, (u8 *)"KEY_UP: Return");
        last_screen = 2;
    }
}

// 优化警报显示函数
void show_alert_screen(u8 alert_type) 
{
    char med_info[50];
    char alert_msg[50];
    char time_str[20];
    
    // 只在状态变化时重新绘制
    if (force_refresh || last_state != alert_type) 
    {
        // 先获取当前时间
        sprintf(time_str, "%02d:%02d", calendar.hour, calendar.min);
        
        if (alert_type == STATE_ALARM) 
        {
            LCD_Clear(YELLOW);
            FRONT_COLOR = RED;
            BACK_COLOR = YELLOW;
            
            // 显示大号警报标题
            LCD_ShowString(30, 40, 240, 50, 36, (u8 *)"MEDICATION ALERT!");
            
            // 显示药物信息
            FRONT_COLOR = BLUE;
            LCD_ShowString(50, 100, 200, 30, 24, (u8 *)"Medicine:");
            FRONT_COLOR = BLACK;
            LCD_ShowString(150, 100, 200, 30, 24, (u8 *)medicines[system_state.next_med_index].name);
            
            // 显示时间信息
            FRONT_COLOR = BLUE;
            LCD_ShowString(50, 140, 200, 30, 24, (u8 *)"Scheduled Time:");
            FRONT_COLOR = BLACK;
            sprintf(med_info, "%02d:%02d", medicines[system_state.next_med_index].hour, 
                                          medicines[system_state.next_med_index].minute);
            LCD_ShowString(200, 140, 200, 30, 24, (u8 *)med_info);
            
            // 显示当前时间
            FRONT_COLOR = BLUE;
            LCD_ShowString(50, 180, 200, 30, 24, (u8 *)"Current Time:");
            FRONT_COLOR = BLACK;
            LCD_ShowString(180, 180, 200, 30, 24, (u8 *)time_str);
            
            // 操作提示
            FRONT_COLOR = RED;
            LCD_ShowString(30, 220, 240, 30, 24, (u8 *)"Take medicine or press UP");
        } 
        else if (alert_type == STATE_MED_TAKEN) 
        {
            LCD_Clear(GREEN);
            FRONT_COLOR = BLUE;
            BACK_COLOR = GREEN;
            
            // 显示确认信息
            LCD_ShowString(80, 80, 200, 50, 36, (u8 *)"MEDICATION TAKEN");
            
            // 显示药物信息
            FRONT_COLOR = BLACK;
            sprintf(med_info, "%s at %s", 
                    medicines[system_state.next_med_index].name, time_str);
            LCD_ShowString(40, 150, 240, 30, 24, (u8 *)med_info);
            
            // 操作提示
            FRONT_COLOR = BLUE;
            LCD_ShowString(60, 200, 200, 30, 24, (u8 *)"Press UP to return");
        } 
        else if (alert_type == STATE_ENV_ALERT) 
        {
            LCD_Clear(RED);
            FRONT_COLOR = WHITE;
            BACK_COLOR = RED;
            
            // 显示警报标题
            LCD_ShowString(60, 40, 240, 50, 36, (u8 *)"ENVIRONMENT ALERT!");
            
            // 显示具体警报信息
            if (system_state.temperature > 30.0) {
                sprintf(alert_msg, "HIGH TEMP: %.1fC", system_state.temperature);
            } else if (system_state.temperature < 10.0) {
                sprintf(alert_msg, "LOW TEMP: %.1fC", system_state.temperature);
            } else {
                sprintf(alert_msg, "HIGH HUMI: %.1f%%", system_state.humidity);
            }
            LCD_ShowString(50, 100, 240, 30, 24, (u8 *)alert_msg);
            
            // 显示当前时间
            LCD_ShowString(50, 140, 240, 30, 24, (u8 *)time_str);
            
            // 操作提示
            LCD_ShowString(30, 200, 240, 30, 24, (u8 *)"Check environment!");
            LCD_ShowString(60, 240, 200, 30, 24, (u8 *)"Press UP to return");
        }
        
        last_state = alert_type;
    }
}

// main函数变量声明提前
int main(void)
{
    u8 last_minute = 0;
    u8 last_second = 0;
    u8 current_screen = 0;
    u8 key;
    char msg[64];
    u8 current_minute;
    u8 current_second;
    static u8 debug_counter = 0; // 移到函数开头
    u8 i;                        // 用于for循环的变量

    // HC05控制相关变量
    static u8 hc05_timer = 0; // HC05定时器
    char sendbuf[32];         // 蓝牙发送缓冲区
    u8 reclen = 0;            // 接收数据长度
    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    LED_Init();
    USART1_Init(115200);
    USART3_Init(9600);
    TFTLCD_Init();
    KEY_Init();
    RTC_Init();
    DHT11_Init();
    BEEP_Init();
    HC05_Init();
    system_init();
    Hwjs_Init(); // 初始化红外遥控模块
    LCD_Clear(BLUE);
    FRONT_COLOR = WHITE;
    LCD_ShowString(40, 100, 200, 40, 32, (u8 *)"Smart Medicine Box");
    LCD_ShowString(20, 150, 200, 30, 24, (u8 *)"System Starting...");
    LCD_ShowString(20, 180, 200, 30, 16, (u8 *)"Bluetooth Ready");
    delay_ms(2000);

    // 发送蓝牙连接成功消息
    Bluetooth_Send("SYSTEM_READY");
    printf("=== Smart Medicine Box Started ===\r\n");
    printf("Bluetooth Ready - Waiting for Commands...\r\n");

    while (1)
    {
        current_minute = calendar.min;
        current_second = calendar.sec;

        // 红外遥控处理逻辑
        if(hw_jsbz) // 红外接收到一帧数据
        {
            u32 code = hw_jsm; // 读取红外码
            u8 key_value;      // 声明提前到最前面
            hw_jsbz = 0;       // 清零标志，准备下一次接收

            printf("IR code: 0x%08X\r\n", code); // 串口打印，方便调试

            key_value = code & 0xFF; // 取最低8位作为按键码

            switch(key_value)
            {
                case 0xCF: // 示例：遥控器某个键
                    system_state.bt_led1_ctrl = 1;
                    LED1 = 0;
                    printf("IR: LED1 ON\r\n");
                    break;
                // 可继续添加其他按键功能
                default:
                    printf("IR: Unknown key\r\n");
                    break;
            }
        }

        // 处理蓝牙数据（主要方法）
        bluetooth_data_process();

        // 简单蓝牙测试（备用方法）
        simple_bluetooth_test();

        // 调试信息：每10秒显示一次USART3接收状态
        if (++debug_counter >= 100) // 100 * 100ms = 10秒
        {
            debug_counter = 0;
            printf("USART3 Status: RX_STA=0x%04X, DataLen=%d\r\n",
                   USART3_RX_STA, USART3_RX_STA & 0x7FFF);
            if ((USART3_RX_STA & 0x7FFF) > 0)
            {
                printf("RX Buffer Content: ");
                for (i = 0; i < (USART3_RX_STA & 0x7FFF) && i < 20; i++)
                {
                    printf("0x%02X ", USART3_RX_BUF[i]);
                }
                printf("\r\n");
            }
        }

        // HC05控制逻辑 - 定时发送和状态更新
        if (++hc05_timer >= 50) // 50 * 100ms = 5秒
        {
            if (bt_send_mask && current_screen == 0) // 只在主界面且开启发送时才发送
            {
                sprintf(sendbuf, "SmartBox %d", bt_send_cnt);
                LCD_ShowString(50, 160, 180, 16, 16, (u8 *)sendbuf); // 显示发送数据
                printf("Sending: %s\r\n", sendbuf);
                u3_printf("SmartBox %d\r\n", bt_send_cnt); // 发送到蓝牙模块
                bt_send_cnt++;
                if (bt_send_cnt > 99)
                    bt_send_cnt = 0;
            }

            // 更新HC05连接状态显示（仅在主界面）
            if (current_screen == 0)
            {
                HC05_Sta_Show();
            }

            hc05_timer = 0;
        }

        // 处理蓝牙接收数据（显示功能）
        if (USART3_RX_STA & 0x8000) // 接收到一次数据了
        {
            if (current_screen == 0) // 仅在主界面显示接收数据
            {
                LCD_Fill(10, 190, 240, 210, WHITE); // 清除接收显示区域
            }

            reclen = USART3_RX_STA & 0x7FFF; // 得到数据长度
            USART3_RX_BUF[reclen] = '\0';    // 加入结束符
            printf("Additional RX - Received length=%d\r\n", reclen);
            printf("Additional RX - Received data=%s\r\n", USART3_RX_BUF);

            // 显示接收到的数据（仅在主界面）
            if (current_screen == 0)
            {
                LCD_ShowString(60, 180, 180, 16, 16, USART3_RX_BUF);
            }

            USART3_RX_STA = 0;
        }

        if (current_minute != last_minute)
        {
            check_medication_time();
            check_environment();
            last_minute = current_minute;
            force_refresh = 1; // 标记需要刷新显示
        }

        // 设备控制逻辑 - 区分系统警报和蓝牙控制
        if (system_state.current_state == STATE_ALARM)
        {
             BEEP = 1;           // 蜂鸣器响
        LED1 = !LED1;       // LED闪烁(500ms周期)
        LED2 = 0;
        
        // 每5秒发送一次提醒
        if (current_second % 5 == 0 && last_second != current_second) {
            sprintf(msg, "ALARM:%s,%02d:%02d", 
                   medicines[system_state.next_med_index].name,
                   medicines[system_state.next_med_index].hour,
                   medicines[system_state.next_med_index].minute);
            Bluetooth_Send(msg);
        }
        }
        else if (system_state.current_state == STATE_ENV_ALERT)
        {
            BEEP = (current_second % 2); // 蜂鸣器间歇响(1秒周期)
        LED1 = 0;
        LED2 = 1;           // LED2常亮
        
        // 每10秒发送一次环境警报
        if (current_second % 10 == 0 && last_second != current_second) {
            if (system_state.temperature > 30.0) {
                sprintf(msg, "ENV_ALERT:HIGH_TEMP,%.1fC", system_state.temperature);
            } else if (system_state.temperature < 10.0) {
                sprintf(msg, "ENV_ALERT:LOW_TEMP,%.1fC", system_state.temperature);
            } else {
                sprintf(msg, "ENV_ALERT:HIGH_HUMI,%.1f%%", system_state.humidity);
            }
            Bluetooth_Send(msg);
        }
        }
        else
        {
            // 正常状态下，根据蓝牙控制状态设置设备
            if (system_state.bt_beep_ctrl)
            {
                BEEP = 1;
            }
            else
            {
                BEEP = 0;
            }

            if (system_state.bt_led1_ctrl)
            {
                LED1 = 0; // 低电平点亮
            }
            else
            {
                LED1 = 1; // 高电平熄灭
            }

            if (system_state.bt_led2_ctrl)
            {
                LED2 = 0; // 低电平点亮
            }
            else
            {
                LED2 = 1; // 高电平熄灭
            }
        }

        key = KEY_Scan(0);
        if (key != 0)
        {
            force_refresh = 1; // 按键操作后强制刷新
            if (key == KEY_UP_PRESS)
            {
                if (system_state.current_state == STATE_ALARM)
                {
                    medicines[system_state.next_med_index].taken = 1;
                    system_state.current_state = STATE_MED_TAKEN;
                    sprintf(msg, "MED_TAKEN:%s,%02d:%02d", medicines[system_state.next_med_index].name, calendar.hour, calendar.min);
                    Bluetooth_Send(msg);
                }
                else if (system_state.current_state == STATE_MED_TAKEN || system_state.current_state == STATE_ENV_ALERT)
                {
                    system_state.current_state = STATE_NORMAL;
                    current_screen = 0;
                }
                else if (current_screen == 0) // 在主界面时，切换HC05主从模式
                {
                    u8 role = HC05_Get_Role();
                    if (role != 0xFF)
                    {
                        role = !role; // 状态取反
                        if (role == 0)
                            HC05_Set_Cmd("AT+ROLE=0");
                        else
                            HC05_Set_Cmd("AT+ROLE=1");
                        HC05_Role_Show();
                        HC05_Set_Cmd("AT+RESET"); // 复位HC05模块
                        delay_ms(200);
                        printf("HC05 Role switched\r\n");
                    }
                }
                else
                {
                    current_screen = 0;
                }
            }
            else if (key == KEY0_PRESS)
            {
                current_screen = 1;
            }
            else if (key == KEY1_PRESS)
            {
                current_screen = 2;
            }
            else if (key == KEY2_PRESS) // 新增KEY2控制蓝牙发送
            {
                bt_send_mask = !bt_send_mask; // 发送/停止发送
                if (bt_send_mask == 0)
                {
                    LCD_Fill(50, 160, 240, 176, WHITE); // 清除发送显示
                    printf("BT Send: OFF\r\n");
                }
                else
                {
                    printf("BT Send: ON\r\n");
                }
            }
        }
        if (system_state.current_state == STATE_NORMAL)
        {
            switch (current_screen)
            {
            case 0:
                show_home_screen();
                break;
            case 1:
                show_medication_screen();
                break;
            case 2:
                show_environment_screen();
                break;
            }
        }
        else
        {
            show_alert_screen(system_state.current_state);
        }
        // Bluetooth_Process(); // 注释掉未定义的蓝牙处理函数
        // 重置强制刷新标志
        force_refresh = 0;
        delay_ms(100);
    }
}
// 只保留Bluetooth_Send的实现
void Bluetooth_Send(const char *msg)
{
    u3_printf("%s\r\n", msg);
}
