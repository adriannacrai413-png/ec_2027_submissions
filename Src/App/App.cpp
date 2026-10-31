/*
 * @Author: userName userEmail
 * @Date: 2026-10-28 15:35:27
 * @LastEditors: userName userEmail
 * @LastEditTime: 2026-10-30 21:31:20
 * @FilePath: \leaner\ec_2027_submissions\Src\App\App.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "App.hpp"
#include "FreeRTOS.h"
#include "StmLog.hpp"
#include "Bsp_pwm.hpp" // Pwm 类：PWM + DMA 呼吸灯（TIM3_CH1 / PA6）
#include "Bsp_uart.hpp"
#include <cstdint>
#include <cstdlib> // strtol：解析 ASCII 数字（RawData 发送方式）
#include <cstring> // memcpy / memmove：解析 JustFloat 帧

// ============================================================
// PWM + DMA 呼吸灯：TIM3/DMA 由 CubeMX 生成代码初始化（MX_TIM3_Init）
// App 层只负责：占空比表 + 启动（DMA_CIRCULAR 自动循环）
// 数据流：内存占空比表 --DMA1 Ch6 自动搬运--> TIM3 CCR1 --> PWM --> PA6 --> LED
// ============================================================

// 引用 CubeMX 生成的全局 TIM3 句柄（定义在 hal/Core/Src/main.c，main() 里已初始化）
extern TIM_HandleTypeDef htim3;

namespace {

// 呼吸灯占空比表（三角波：0% -> 100% -> 0%）
constexpr uint16_t kTableLen = 200; // 200 点 = 100ms 上升 + 100ms 下降（PWM 1kHz）

// 占空比表：DMA 逐点自动写入 TIM3 的 CCR1。
// 必须是 uint32_t：HAL 的 PWM-DMA 接口要求 32 位指针
uint32_t sBreathTable[kTableLen];

// 框架 PWM 驱动对象：绑定 CubeMX 生成的 htim3 + 通道 1（对应 PA6）
Pwm ledPwm(&htim3, TIM_CHANNEL_1);

// Buffer incoming bursts while the 1 ms app task is asleep.
uint8_t rxBuffer[32], rxQueue[64];
volatile uint8_t rxHead = 0, rxTail = 0;
volatile bool rxLost = false;
Uart &commandUart()
{
    static Uart uart(&huart1);
    return uart;
}

// 生成三角波占空比表：值从 0 -> 999 -> 0
void buildTable()
{
    constexpr uint32_t kMaxDuty = 999; // 与 ARR 相同，即 100% 占空比

    for (uint16_t i = 0; i < kTableLen; i++) {
        if (i < kTableLen / 2) {
            sBreathTable[i] = kMaxDuty * 2 * i / kTableLen;               // 上升段：0 -> 999
        } else {
            sBreathTable[i] = kMaxDuty * 2 * (kTableLen - i) / kTableLen; // 下降段：999 -> 0
        }
    }
}

} // namespace

// TIM3 is DMA_CIRCULAR; no completion callback needs to restart it.

void App::init()
{
    commandUart().registerCallback([](UART_HandleTypeDef *, uint16_t size) {
        if (size == 0) { rxLost = true; } // RX error recovery notification.
        for (uint16_t i = 0; i < size; ++i) {
            const uint8_t next = (rxHead + 1U) % sizeof(rxQueue);
            if (next == rxTail) { rxLost = true; break; }
            rxQueue[rxHead] = rxBuffer[i];
            __DMB();
            rxHead = next;
        }
        commandUart().receiveIT(rxBuffer, sizeof(rxBuffer));
    });
    commandUart().receiveIT(rxBuffer, sizeof(rxBuffer));
    initBreath(); // 启动 PWM + DMA 呼吸灯（TIM3_CH1 / PA6）
}

// 呼吸灯启动：CubeMX 已初始化 TIM3/DMA，这里只需两步
void App::initBreath()
{
    buildTable();                              // 1. 生成 200 点三角波占空比表
    ledPwm.startDMA(sBreathTable, kTableLen);  // 2. 启动：DMA 开始逐点更新占空比 -> LED 呼吸
    LOG::info("LedBreath", "breath start, %u points", kTableLen);
}

void App::task()
{
    // ---- vofa+ 波形演示：UART（USART1/PA9）+ JustFloat 协议 ----
    // 每 5ms 读一次 TIM3 当前 CCR（DMA 正在写入的占空比），归一化后发出。
    // LOG::justFloat 是框架原生 vofa+ 协议：1 个 float32 + 帧尾 {00 00 80 7F}
    static uint32_t lastTick = 0;
    const uint32_t now = xTaskGetTickCount();
    if (now - lastTick >= 5) {
        lastTick = now;

        constexpr uint32_t kMaxDuty = 999; // 与 ARR 相同 = 100% 占空比
        const uint32_t ccr = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1); // 读当前占空比
        LOG::justFloat((float)ccr / (float)kMaxDuty); // 归一化 0.0~1.0，经 UART 发出
    }

    // ---- 接收：VOFA+ 发送 -> 改变占空比（双向通信演示）----
    // 支持两种发送方式：
    //   ① JustFloat 帧（4 字节 float + 帧尾 00 00 80 7F）：值 0.0~1.0 -> 固定占空比；>1000 -> 恢复呼吸
    //   ② RawData 纯文本数字 + 回车：0~999 -> 固定占空比；9999 -> 恢复呼吸
    static uint8_t jfBuf[8] = {0}; // JustFloat 帧缓冲（4B float + 4B 帧尾）
    static uint8_t jfIdx = 0;      // 帧缓冲写入位置
    static char    asBuf[16];      // ASCII 数字行缓冲
    static uint8_t asIdx = 0;      // 行缓冲长度
    uint8_t byte;
    if (rxLost) {
        jfIdx = asIdx = 0;
        rxTail = rxHead;
        rxLost = false;
    }
    while (rxTail != rxHead) {
        byte = rxQueue[rxTail];
        __DMB();
        rxTail = (rxTail + 1U) % sizeof(rxQueue);
        // --- ① JustFloat 帧解析 ---
        jfBuf[jfIdx++] = byte;
        if (jfIdx == 8) { // 攒够 8 字节才判断：4B float + 4B 帧尾
            const bool tail = (jfBuf[4] == 0x00 && jfBuf[5] == 0x00 && jfBuf[6] == 0x80 && jfBuf[7] == 0x7F);
            if (tail) { // 帧尾 {00 00 80 7F} 校验通过 = 一帧完整的 JustFloat
                uint32_t u = (uint32_t)jfBuf[0] | ((uint32_t)jfBuf[1] << 8) |
                             ((uint32_t)jfBuf[2] << 16) | ((uint32_t)jfBuf[3] << 24); // 小端拼成 uint32
                float v;
                memcpy(&v, &u, sizeof(v)); // 位拷贝成 float
                if (v >= 0.0f && v <= 1.0f) { // 0~1：停呼吸，固定占空比 = v * 999
                    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(v * 999.0f));
                    ledPwm.start(); // Stop_DMA also disables PWM output.
                } else if (v > 1000.0f) { // 发 9999 恢复呼吸
                    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
                    ledPwm.startDMA(sBreathTable, kTableLen);
                }
                jfIdx = 0;
            } else { // 帧尾不对：丢掉最老 1 字节，继续滑动找帧尾
                memmove(jfBuf, jfBuf + 1, 7);
                jfIdx = 7;
            }
        }
        // --- ② ASCII 数字行解析（RawData 方式）---
        if (byte == '\n' || byte == '\r') { // 回车/换行 = 一行结束
            if (asIdx > 0) {
                // 跳过前导非数字字节（可能被 JustFloat 二进制帧污染，避免误解析成 0）
                uint8_t start = 0;
                while (start < asIdx && !((asBuf[start] >= '0' && asBuf[start] <= '9') || asBuf[start] == '.')) {
                    start++;
                }
                if (start < asIdx) { // 找到数字才开始解析
                    // 手工解析数字："500" 整数或 "0.5" 小数（不用 strtof，省 FLASH）
                    float f = 0.0f;
                    bool inFrac = false;
                    float fracDiv = 1.0f;
                    uint8_t i = start;
                    while (i < asIdx) {
                        const char c = asBuf[i];
                        if (c >= '0' && c <= '9') {
                            if (inFrac) { // 小数部分：0.1 -> 0.01 -> ...
                                fracDiv *= 10.0f;
                                f += (float)(c - '0') / fracDiv;
                            } else { // 整数部分：1 -> 12 -> 123
                                f = f * 10.0f + (float)(c - '0');
                            }
                        } else if (c == '.') {
                            inFrac = true;
                        } else {
                            break; // 非数字字符结束解析
                        }
                        i++;
                    }
                    if (f >= 0.0f && f <= 999.0f) { // 0~999：停呼吸，固定占空比
                        HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
                        const uint32_t duty = (f <= 1.0f) ? (uint32_t)(f * 999.0f) : (uint32_t)f;
                        // 0.0~1.0 按比例算（0.5 -> 499）；>1.0 当绝对占空比（500 -> 500）
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
                        ledPwm.start(); // Re-enable fixed-duty PWM.
                    } else if (f > 1000.0f) { // 发 9999 恢复呼吸
                        HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
                        ledPwm.startDMA(sBreathTable, kTableLen);
                    }
                }
                asIdx = 0;
            }
        } else if (asIdx < sizeof(asBuf) - 1) { // 非换行字节：攒进行缓冲
            asBuf[asIdx++] = (char)byte;
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
