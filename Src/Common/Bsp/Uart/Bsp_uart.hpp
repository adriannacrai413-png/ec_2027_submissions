#pragma once

#include "sdkconfig.h"
#include "Soc.hpp"
#include HAL_INCLUDE
#if defined(TARGET_STM32F103C8TX) && TARGET_STM32F103C8TX
#include "HAL_Driver/Inc/stm32f1xx_hal_uart.h"
#endif
#include <functional>


class Uart {
public:
    Uart(UART_HandleTypeDef *_huart);

    /**
    * @brief uart registerCallback
    */
    using callback = std::function<void(UART_HandleTypeDef *, uint16_t)>;
    void registerCallback(callback _pCallback);

    using txCallback = std::function<void()>;
    void registerTxCallback(txCallback _pCallback);

    /**
    * @brief uart unregisterCallback
    */
    void unregisterCallback();
    void unregisterTxCallback();

    /**
    * @brief uart 接收被错误处理流程重启后的通知回调
    *
    * HAL 在 ORE/FE/PE 等错误发生后会中止本次 DMA 接收，需要用
    * HAL_UARTEx_ReceiveToIdle_DMA 从缓冲区起始地址重新开始。
    * 上层如果维护了软件环形指针，必须在这里同步归零，否则下一次回调会把
    * 缓冲区里的旧字节当成新数据再搬一遍（表现为“几十字节才拼出一帧”）。
    * 注意：该回调在中断上下文中执行，里面不要做耗时操作。
    */
    using rxRestartCallback = std::function<void(UART_HandleTypeDef *)>;
    void registerRxRestartCallback(rxRestartCallback _pCallback);
    void unregisterRxRestartCallback();

    /**
    * @brief uart multi_DMA_rx_buf init
    *
    * @param DataLength 请开辟两倍的缓冲区
    */
    HAL_StatusTypeDef recvDmaMultiBufInit(uint32_t *_dstAddress, uint32_t _dataLength);

    /**
    * @brief uart multi_DMA_rx_buf init
    */
    HAL_StatusTypeDef recvDmaInit(uint8_t *_dstAddress, uint32_t _dataLength);

    /**
    * @brief uart rx callbackFromISR
    */
    void callbackFromISR(uint16_t _size);

    /**
    * @brief uart tx
    */
    HAL_StatusTypeDef transmit(const uint8_t *_pData, uint16_t _size);

    /**
    * @brief uart tx dma
    */
    HAL_StatusTypeDef transmitDma(const uint8_t *_pData, uint16_t _size);

    /**
    * @brief uart rx
    */
    HAL_StatusTypeDef receive(uint8_t *_pData, uint16_t _size);

    /**
    * @brief uart rx
    */
    HAL_StatusTypeDef receiveDma(uint8_t *_pData, uint16_t _size);

    UART_HandleTypeDef *huart_;
};
