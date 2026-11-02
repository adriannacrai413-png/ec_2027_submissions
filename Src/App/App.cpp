/*

 */
#include "App.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "StmLog.hpp"
#include "Bsp_pwm.hpp"
#include "Bsp_uart.hpp"
#include <cstdint>
#include <cstring>

#if EXTENSION_ET08A
#include "main.h"
#endif

extern TIM_HandleTypeDef htim3;    
extern UART_HandleTypeDef huart1;  

uint32_t breathTable[200];         
Pwm ledPwm(&htim3, TIM_CHANNEL_1); // 



static Uart uartTx_{ &huart1 };

static uint8_t s_rxByte;
static char    s_line[16];
static uint8_t s_count = 0;
static bool    s_ready = false;


void makeBreathTable()
{
    uint16_t i;

    for (i = 0; i < 100; i++) {
        breathTable[i] = 999 * i / 100;
    }
    for (i = 100; i < 200; i++) {
        breathTable[i] = 999 * (199 - i) / 100;
    }
}

void App::init()
{
      makeBreathTable();
    ledPwm.startDMA(breathTable, 200); 
    HAL_UART_Receive_IT(&huart1, &s_rxByte, 1); // 启动第一次接收中断；回调内会重新挂起
}



void App::task()
{
    static uint32_t lastTime = 0;
    uint32_t now = xTaskGetTickCount();

   
    if (now - lastTime >= 5) {
        lastTime = now;
        uint32_t ccr = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);

        float value = (float)ccr / 999.0f;
        uint8_t num[8]; 
        std::memcpy(num, &value, sizeof(value));
        num[4] = 0x00;
        num[5] = 0x00;
        num[6] = 0x80;
        num[7] = 0x7f;
       //HAL_StatusTypeDef st =uartTx_.transmitDma(num, sizeof(num));
         (void)uartTx_.transmit(num, sizeof(num)); 
        
    }

 
    if (s_ready) {
       
        int num = 0;
        for (uint8_t i = 0; i < s_count; i++) {
            if (s_line[i] >= '0' && s_line[i] <= '9') {
                num = num * 10 + (s_line[i] - '0');
            }
        }

        if (num == 9999) {
            // 发 9999：恢复呼吸
            HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
            ledPwm.startDMA(breathTable, 200);
        } else if (num <= 999) {
           
            HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
            ledPwm.setDutyCycle(num);
            ledPwm.start();

        }
    }
s_ready = false;   
        s_count = 0;
#if EXTENSION_ET08A
    if (lxServo_ == nullptr || receiver_ == nullptr) {
        return;
    }

    const bool online = receiver_->isOnline();

    if (!online) {
        lxServo_->setCenter();
        const HAL_StatusTypeDef recoveryStatus = receiver_->recoverIfOffline();
        if (recoveryStatus != HAL_OK && recoveryStatus != HAL_BUSY) {
            LOG::error("App", "ET08A UART DMA recovery failed: %d", recoveryStatus);
        }
    }
#endif
}


extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    uint8_t ch = s_rxByte;
    if (ch == '\n' || ch == '\r') {
        if (s_count > 0) {
            s_ready = true;
        }
    } else {
        uint8_t cnt = s_count;
        if (cnt < 15) {
            s_line[cnt] = (char)ch;
            s_count = cnt + 1;
        }
    }

    HAL_UART_Receive_IT(huart, &s_rxByte, 1); // 重新挂起下一字节接收
}

#if EXTENSION_ET08A
bool App::readRcData(ET08A::RcData_s &_data) {}
#endif
