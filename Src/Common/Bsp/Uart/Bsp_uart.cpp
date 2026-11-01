#include "Bsp_uart.hpp"
#include HAL_INCLUDE
#include <vector>
#include <cstdint>

static std::vector<std::pair<UART_HandleTypeDef *, Uart::callback> > cbTable;
static std::vector<std::pair<UART_HandleTypeDef *, Uart::txCallback> > txCbTable;
static std::vector<std::pair<UART_HandleTypeDef *, Uart::rxRestartCallback> > rxRestartTable;

Uart::Uart(UART_HandleTypeDef *_huart) : huart_(_huart)
{
    /* reserve 4 firstly */
    cbTable.reserve(4);
    txCbTable.reserve(4);
    rxRestartTable.reserve(4);
}

void Uart::registerCallback(callback _pCallback)
{
    for (const auto &pair : cbTable) {
        if (pair.first == huart_) {
            return;
        }
    }
    cbTable.emplace_back(huart_, _pCallback);
}

void Uart::unregisterCallback()
{
    for (auto it = cbTable.begin(); it != cbTable.end(); ++it) {
        if (it->first == huart_) {
            cbTable.erase(it);
            return;
        }
    }
}

void Uart::registerTxCallback(txCallback _pCallback)
{
    for (const auto &pair : txCbTable) {
        if (pair.first == huart_) {
            return;
        }
    }
    txCbTable.emplace_back(huart_, _pCallback);
}

void Uart::unregisterTxCallback()
{
    for (auto it = txCbTable.begin(); it != txCbTable.end(); ++it) {
        if (it->first == huart_) {
            txCbTable.erase(it);
            return;
        }
    }
}

void Uart::registerRxRestartCallback(rxRestartCallback _pCallback)
{
    for (const auto &pair : rxRestartTable) {
        if (pair.first == huart_) {
            return;
        }
    }
    rxRestartTable.emplace_back(huart_, _pCallback);
}

void Uart::unregisterRxRestartCallback()
{
    for (auto it = rxRestartTable.begin(); it != rxRestartTable.end(); ++it) {
        if (it->first == huart_) {
            rxRestartTable.erase(it);
            return;
        }
    }
}

/* 判断这个串口有没有注册接收回调（没注册就不必重启接收） */
static bool hasRxCallback(UART_HandleTypeDef *_huart)
{
    for (const auto &pair : cbTable) {
        if (pair.first == _huart) {
            return true;
        }
    }
    return false;
}

/* 通知上层：接收已被 HAL 从头重启，软件环形指针需要同步归零 */
static void notifyRxRestart(UART_HandleTypeDef *_huart)
{
    for (const auto &pair : rxRestartTable) {
        if (pair.first == _huart) {
            pair.second(_huart);
            return;
        }
    }
}

HAL_StatusTypeDef Uart::recvDmaMultiBufInit(uint32_t *_dstAddress, uint32_t _dataLength)
{
#if defined(TARGET_STM32F407IGHX) && TARGET_STM32F407IGHX
    HAL_StatusTypeDef result = HAL_OK;
    huart_->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart_->RxEventType = HAL_UART_RXEVENT_IDLE;
    huart_->RxXferSize = _dataLength;
    SET_BIT(huart_->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
    uint32_t *secondMemAddress = _dstAddress + (_dataLength / 2);
    result = HAL_DMAEx_MultiBufferStart(huart_->hdmarx, (uint32_t)&SOC_UART_DMA_DR, (uint32_t)_dstAddress,
                                        (uint32_t)secondMemAddress, _dataLength);

    return result;
#else
    (void)_dstAddress;
    (void)_dataLength;
    return HAL_ERROR;
#endif
}

HAL_StatusTypeDef Uart::recvDmaInit(uint8_t *_dstAddress, uint32_t _dataLength)
{
    HAL_StatusTypeDef result = HAL_OK;
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
    result = HAL_UARTEx_ReceiveToIdle_DMA(huart_, _dstAddress, _dataLength);
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
    return result;
}

HAL_StatusTypeDef Uart::transmit(const uint8_t *_pData, uint16_t _size)
{
    return HAL_UART_Transmit(huart_, _pData, _size, 0);
}

HAL_StatusTypeDef Uart::transmitDma(const uint8_t *_pData, uint16_t _size)
{
    return HAL_UART_Transmit_DMA(huart_, _pData, _size);
}

HAL_StatusTypeDef Uart::receive(uint8_t *_pData, uint16_t _size) { return HAL_UART_Receive(huart_, _pData, _size, 0); }

HAL_StatusTypeDef Uart::receiveDma(uint8_t *_pData, uint16_t _size)
{
    HAL_StatusTypeDef result = HAL_UARTEx_ReceiveToIdle_DMA(huart_, _pData, _size);
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
    return result;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *_huart, uint16_t _size)
{
    for (const auto &pair : cbTable) {
        if (pair.first == _huart) {
            pair.second(_huart, _size);
            break;
        }
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *_huart)
{
    for (const auto &pair : txCbTable) {
        if (pair.first == _huart) {
            pair.second();
            break;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *_huart)
{
    /* 错误类型必须按位判断：原来用的是 "=="，一旦同时置了多个错误位
     * （比如 FE|NE），两个分支都匹配不上，接收就再也不会被重新挂起，串口从此哑掉。 */
    const uint32_t error = HAL_UART_GetError(_huart);

    if ((error & HAL_UART_ERROR_ORE) != 0U) {
        __HAL_UART_CLEAR_OREFLAG(_huart);
    }
    if ((error & HAL_UART_ERROR_FE) != 0U) {
        //https://blog.csdn.net/Jonshen/article/details/144032215 (上电初始化出现帧错误（FE）)
        __HAL_UART_CLEAR_FEFLAG(_huart);
    }
    if ((error & HAL_UART_ERROR_NE) != 0U) {
        __HAL_UART_CLEAR_NEFLAG(_huart);
    }
    if ((error & HAL_UART_ERROR_PE) != 0U) {
        __HAL_UART_CLEAR_PEFLAG(_huart);
    }

    /* 出错时 HAL 已经中止了本次 DMA 接收（先异步 abort DMA，再回调到这里），
     * 所以这里必须重新挂上，否则这个串口就再也收不到数据了。
     * 只有重启成功才通知上层同步软件环：重启失败时 DMA 写指针并没有归零，
     * 这时如果去清指针反而会把数据读错位。 */
    if (hasRxCallback(_huart)) {
        const HAL_StatusTypeDef restartStatus =
            HAL_UARTEx_ReceiveToIdle_DMA(_huart, _huart->pRxBuffPtr, _huart->RxXferSize);
        if (restartStatus == HAL_OK) {
            __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
            notifyRxRestart(_huart);
        }
    }
}
