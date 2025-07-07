#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "tftlcd.h"
#include "time.h"
#include "key.h"
#include "rtc.h"
#include "stdlib.h"
#include "dht11.h"         // ????????
#include "beep.h"        // ?????
#include "hc05.h"
#include "usart3.h"	

// ??????
#define STATE_NORMAL       0
#define STATE_ALARM        1
#define STATE_MED_TAKEN    2
#define STATE_ENV_ALERT    3

// ??????
typedef struct {
    char name[16];         // ????
    u8 hour;               // ????(??)
    u8 minute;             // ????(??)
    u8 taken;              // ?????
} Medicine;

// ??????
struct SystemState {
    u8 current_state;      // ??????
    u8 env_alert;          // ??????
    u8 med_count;          // ????
    u8 next_med_index;     // ????????????
    float temperature;     // ????
    float humidity;        // ????
} system_state;

// ????(??)
Medicine medicines[] = {
    {"???", 8, 0, 0},
    {"???", 12, 30, 0},
    {"???", 19, 0, 0}
};

// ???????(????????)
u32 ir_sensor_count = 0;

// ????
void show_home_screen(void);
void show_medication_screen(void);
void show_environment_screen(void);
void show_alert_screen(u8 alert_type);

// 文件顶部声明外部函数
void USART1_Init(u32 bound);
void USART3_Init(u32 bound);
void BEEP_Init(void);
u8 HC05_Init(void);
void Bluetooth_Send(const char *msg);

// ?????
void system_init(void) {
    system_state.current_state = STATE_NORMAL;
    system_state.env_alert = 0;
    system_state.med_count = sizeof(medicines)/sizeof(Medicine);
    system_state.next_med_index = 0;
    system_state.temperature = 0.0;
    system_state.humidity = 0.0;
}

// ??????
void check_medication_time(void) {
    u8 current_hour = calendar.hour;
    u8 current_minute = calendar.min;
    u8 i;
    for (i = 0; i < system_state.med_count; i++) {
        if (!medicines[i].taken && 
            current_hour == medicines[i].hour && 
            current_minute == medicines[i].minute) {
            system_state.next_med_index = i;
            system_state.current_state = STATE_ALARM;
            break;
        }
    }
}

// ??????
void check_environment(void) {
    u8 temp = 0, humi = 0;
    if (DHT11_Read_Data(&temp, &humi) == 0) {
        system_state.temperature = temp;
        system_state.humidity = humi;
        if (system_state.temperature > 30.0 || system_state.temperature < 10.0 || 
            system_state.humidity > 70.0) {
            system_state.env_alert = 1;
            system_state.current_state = STATE_ENV_ALERT;
        } else {
            system_state.env_alert = 0;
        }
    }
}

// ?????????(??)
void EXTI0_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        ir_sensor_count++;
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

// ??????
void handle_medication(void) {
    // ????????????(????)
		char msg[64];
    if (ir_sensor_count >= 3) {
        medicines[system_state.next_med_index].taken = 1;
        system_state.current_state = STATE_MED_TAKEN;
        ir_sensor_count = 0;
        
        // ??????????
        
        sprintf(msg, "MED_TAKEN:%s,%02d:%02d", 
                medicines[system_state.next_med_index].name,
                calendar.hour, calendar.min);
        Bluetooth_Send(msg);
    }
}

// ?????
void show_home_screen(void) {
    char time_str[20];
    char med_info[40];
    char env_status[30];
    LCD_Clear(WHITE);
    FRONT_COLOR = BLUE;
    BACK_COLOR = WHITE;
    LCD_ShowString(60, 10, 200, 30, 24, (u8*)"Smart Medicine Box");
    sprintf(time_str, "%04d-%02d-%02d %02d:%02d", calendar.w_year, calendar.w_month, calendar.w_date, calendar.hour, calendar.min);
    FRONT_COLOR = BLACK;
    LCD_ShowString(50, 50, 200, 30, 16, (u8*)time_str);
    if (system_state.next_med_index < system_state.med_count) {
        sprintf(med_info, "Next: %s %02d:%02d", medicines[system_state.next_med_index].name, medicines[system_state.next_med_index].hour, medicines[system_state.next_med_index].minute);
        LCD_ShowString(30, 80, 200, 30, 16, (u8*)med_info);
    }
    if (system_state.env_alert) {
        sprintf(env_status, "Env Alert!");
        FRONT_COLOR = RED;
    } else {
        sprintf(env_status, "Env Normal");
        FRONT_COLOR = GREEN;
    }
    LCD_ShowString(80, 110, 200, 30, 16, (u8*)env_status);
    FRONT_COLOR = BLACK;
    LCD_ShowString(20, 200, 200, 30, 16, (u8*)"KEY0: Medicine");
    LCD_ShowString(20, 220, 200, 30, 16, (u8*)"KEY1: Env");
    LCD_ShowString(20, 240, 200, 30, 16, (u8*)"KEY_UP: Return");
}

// ????????
void show_medication_screen(void) {
    u8 y_pos = 50;
    u8 i;
    char med_info[50];
    u8 *status_str;
    LCD_Clear(LGRAY);
    FRONT_COLOR = BLUE;
    BACK_COLOR = LGRAY;
    LCD_ShowString(80, 10, 200, 30, 24, (u8*)"Medicine Info");
    for (i = 0; i < system_state.med_count; i++) {
        if (medicines[i].taken)
            status_str = (u8*)"Taken";
        else
            status_str = (u8*)"Not taken";
        sprintf(med_info, "%s %02d:%02d %s", medicines[i].name, medicines[i].hour, medicines[i].minute, status_str);
        FRONT_COLOR = medicines[i].taken ? GREEN : RED;
        LCD_ShowString(30, y_pos, 200, 30, 16, (u8*)med_info);
        y_pos += 30;
    }
    FRONT_COLOR = BLACK;
    LCD_ShowString(20, 260, 200, 30, 16, (u8*)"KEY_UP: Return");
}

// ????????
void show_environment_screen(void) {
    char temp_str[20];
    char humi_str[20];
    LCD_Clear(LGRAY);
    FRONT_COLOR = BLUE;
    BACK_COLOR = LGRAY;
    LCD_ShowString(80, 10, 200, 30, 24, (u8*)"Env Info");
    sprintf(temp_str, "Temp: %.1fC", system_state.temperature);
    sprintf(humi_str, "Humi: %.1f%%", system_state.humidity);
    FRONT_COLOR = BLACK;
    LCD_ShowString(80, 60, 200, 30, 24, (u8*)temp_str);
    LCD_ShowString(80, 90, 200, 30, 24, (u8*)humi_str);
    if (system_state.env_alert) {
        LCD_ShowString(60, 150, 200, 30, 24, (u8*)"Env Alert!");
        LCD_ShowString(30, 180, 200, 30, 16, (u8*)"Please check!");
        FRONT_COLOR = RED;
    } else {
        LCD_ShowString(80, 150, 200, 30, 24, (u8*)"Env Normal");
        FRONT_COLOR = GREEN;
    }
    FRONT_COLOR = BLACK;
    LCD_ShowString(20, 260, 200, 30, 16, (u8*)"KEY_UP: Return");
}

// ??????
void show_alert_screen(u8 alert_type) {
    char med_info[50];
    char alert_msg[50];
    if (alert_type == STATE_ALARM) {
        LCD_Clear(YELLOW);
        FRONT_COLOR = RED;
        LCD_ShowString(60, 60, 200, 40, 32, (u8*)"Take Medicine!");
        sprintf(med_info, "Time: %s", medicines[system_state.next_med_index].name);
        FRONT_COLOR = BLACK;
        LCD_ShowString(50, 120, 200, 30, 24, (u8*)med_info);
        LCD_ShowString(30, 180, 200, 30, 16, (u8*)"Please take now!");
        LCD_ShowString(60, 210, 200, 30, 16, (u8*)"Please take now!");
    } else if (alert_type == STATE_MED_TAKEN) {
        LCD_Clear(GREEN);
        FRONT_COLOR = BLUE;
        LCD_ShowString(60, 60, 200, 40, 32, (u8*)"Taken!");
        sprintf(med_info, "%s Taken", medicines[system_state.next_med_index].name);
        FRONT_COLOR = BLACK;
        LCD_ShowString(50, 120, 200, 30, 24, (u8*)med_info);
        LCD_ShowString(80, 180, 200, 30, 16, (u8*)"Press KEY_UP to return");
    } else if (alert_type == STATE_ENV_ALERT) {
        LCD_Clear(RED);
        FRONT_COLOR = WHITE;
        LCD_ShowString(60, 60, 200, 40, 32, (u8*)"Env Alert!");
        if (system_state.temperature > 30.0) {
            sprintf(alert_msg, "Temp High: %.1fC", system_state.temperature);
        } else if (system_state.temperature < 10.0) {
            sprintf(alert_msg, "Temp Low: %.1fC", system_state.temperature);
        } else {
            sprintf(alert_msg, "Humi Alert: %.1f%%", system_state.humidity);
        }
        LCD_ShowString(30, 120, 200, 30, 24, (u8*)alert_msg);
        LCD_ShowString(30, 160, 200, 30, 16, (u8*)"Please check!");
        LCD_ShowString(60, 190, 200, 30, 16, (u8*)"Press KEY_UP to return");
    }
}



// main函数变量声明提前
int main(void) {
    u8 last_minute = 0;
    u8 last_second = 0;
    u8 current_screen = 0;
    u8 key;
    char msg[64];
    u8 current_minute;
    u8 current_second;
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
    LCD_Clear(BLUE);
    FRONT_COLOR = WHITE;
    LCD_ShowString(40, 100, 200, 40, 32, (u8*)"Smart Medicine Box");
    LCD_ShowString(20, 150, 200, 30, 24, (u8*)"System Starting...");
    delay_ms(2000);
    while(1) {
        current_minute = calendar.min;
        current_second = calendar.sec;
        if (current_minute != last_minute) {
            check_medication_time();
            check_environment();
            last_minute = current_minute;
        }
        if (system_state.current_state == STATE_ALARM) {
            BEEP = 1;
            LED1 = !LED1;
            handle_medication();
        } else if (system_state.current_state == STATE_ENV_ALERT) {
            BEEP = 1;
            LED2 = 1;
        } else {
            BEEP = 0;
            LED1 = 0;
            LED2 = 0;
        }
        key = KEY_Scan(0);
        if (key != 0) {
            if (key == KEY_UP_PRESS) {
                if (system_state.current_state == STATE_ALARM) {
                    medicines[system_state.next_med_index].taken = 1;
                    system_state.current_state = STATE_MED_TAKEN;
                    sprintf(msg, "MED_TAKEN:%s,%02d:%02d", medicines[system_state.next_med_index].name, calendar.hour, calendar.min);
                    Bluetooth_Send(msg);
                } else if (system_state.current_state == STATE_MED_TAKEN || system_state.current_state == STATE_ENV_ALERT) {
                    system_state.current_state = STATE_NORMAL;
                    current_screen = 0;
                } else {
                    current_screen = 0;
                }
            } else if (key == KEY0_PRESS) {
                current_screen = 1;
            } else if (key == KEY1_PRESS) {
                current_screen = 2;
            }
        }
        if (system_state.current_state == STATE_NORMAL) {
            switch (current_screen) {
                case 0: show_home_screen(); break;
                case 1: show_medication_screen(); break;
                case 2: show_environment_screen(); break;
            }
        } else {
            show_alert_screen(system_state.current_state);
        }
        // Bluetooth_Process(); // 注释掉未定义的蓝牙处理函数
        delay_ms(100);
    }
}
// 只保留Bluetooth_Send的实现
void Bluetooth_Send(const char *msg) {
    u3_printf("%s\r\n", msg);
}