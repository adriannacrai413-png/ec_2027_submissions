/*
 * @Author: userName userEmail
 * @Date: 2026-10-28 15:35:27
 * @LastEditors: MJH 2264698762@qq.com
 * @LastEditTime: 2026-11-01 21:51:52
 * @FilePath: \leaner\ec_2027_submissions\Src\App\App.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "App.hpp"
#include "FreeRTOS.h"
#include "StmLog.hpp"
#include "Bsp_pwm.hpp"
#include <cstdint>


#if EXTENSION_ET08A
#include "main.h"
#endif

extern TIM_HandleTypeDef htim3;    // 控制灯的定时器
extern UART_HandleTypeDef huart1; 

uint32_t breathTable[200];         // 存 200 个亮度值
Pwm ledPwm(&htim3, TIM_CHANNEL_1); // 灯接在 htim3 的通道 1（PA6）

// 造一个亮度表：先从 0 慢慢加到 999，再慢慢减回 0
void makeBreathTable()
{
    uint16_t i;

    for (i = 0; i < 100; i++) {
        breathTable[i] = 999 * i / 100;           // 前 100 个：越来越亮
    }
    for (i = 100; i < 200; i++) {
        breathTable[i] = 999 * (199 - i) / 100;   // 后 100 个：越来越暗
    }
}


void App::init() {
    
    initBreath();
}
void App::initBreath()
{
    makeBreathTable();                  // 先造好亮度表
    ledPwm.startDMA(breathTable, 200);  // 让 DMA 按表自动改亮度，灯就呼吸起来
  
}

void App::task()
{

     static uint32_t lastTime = 0;
    uint32_t now = xTaskGetTickCount();

    if (now - lastTime >= 5) {
        lastTime = now;
        uint32_t ccr = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1); // 读当前亮度
        LOG::justFloat((float)ccr / 999.0f);                         // 发给电脑
    }

    // ===== 第二件事：收电脑发来的数字，一个字节一个字节读，不卡住程序 =====
    static char text[16];    // 存一行文字
    static uint8_t count = 0; // 已经存了几个字
    uint8_t ch;

    while (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK) {
        if (ch == '\n' || ch == '\r') {
            // 收到回车，说明一行数字发完了
            if (count > 0) {
                int num = 0;

                // 把文字 "500" 变成数字 500
                for (uint8_t i = 0; i < count; i++) {
                    if (text[i] >= '0' && text[i] <= '9') {
                        num = num * 10 + (text[i] - '0');
                    }
                }

                if (num == 9999) {
                    // 发 9999：恢复呼吸
                    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
                    ledPwm.startDMA(breathTable, 200);
                } else if (num <= 999) {
                    // 发 0~999：把灯固定在这个亮度（500 就是半亮）
                    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
                      }

                if (num == 9999) {
                    // 发 9999：恢复呼吸
                    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
                    ledPwm.startDMA(breathTable, 200);
                } else if (num <= 999) {
                    // 发 0~999：把灯固定在这个亮度（500 就是半亮）
                    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
                    ledPwm.setDutyCycle(num); // 设亮度，内部就是写 CCR1

                    ledPwm.start();
                }

                    ledPwm.start();
                }

                count = 0; // 清空，准备收下一行
            }
        } 



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

#if EXTENSION_ET08A
bool App::readRcData(ET08A::RcData_s &_data) {}

#endif
