#include "ET08A.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <cstring>

namespace {
constexpr uint32_t OFFLINE_RECOVERY_DELAY_MS = 500;


constexpr uint32_t FORMAT_PROBE_WINDOW_MS = 400;

constexpr uint32_t FORMAT_PROBE_MAX_RESTARTS = 4;
}

ET08A::ET08A(UART_HandleTypeDef *_huart) : uart_(_huart) {}

ET08A::~ET08A()
{
    if (started_) {
        HAL_UART_DMAStop(uart_.huart_);
        __HAL_UART_DISABLE_IT(uart_.huart_, UART_IT_IDLE);
    }
    uart_.unregisterCallback();
    uart_.unregisterRxRestartCallback();
}

HAL_StatusTypeDef ET08A::start()
{
    if (started_) {
        return HAL_BUSY;
    }

    rxQueue_ = xQueueCreateStatic(RX_QUEUE_LENGTH, sizeof(RxChunk_s), rxQueueStorage_.data(), &rxQueueControl_);
    if (rxQueue_ == nullptr) {
        return HAL_ERROR;
    }

    uart_.registerCallback([this](UART_HandleTypeDef *_huart, uint16_t _position) {
        onRxEvent(_huart, _position);
    });


    uart_.registerRxRestartCallback([this](UART_HandleTypeDef *) { onRxRestart(); });

    const HAL_StatusTypeDef status = uart_.receiveDma(rxBuffer_.data(), RX_BUFFER_LENGTH);
    if (status != HAL_OK) {
        uart_.unregisterCallback();
        uart_.unregisterRxRestartCallback();
        rxQueue_ = nullptr;
        return status;
    }

    started_ = true;
    lastRecoveryTick_ = static_cast<uint32_t>(xTaskGetTickCount());
    return HAL_OK;
}

HAL_StatusTypeDef ET08A::recoverIfOffline()
{
    if (!started_) {
        return HAL_ERROR;
    }

    if (probeActive_) {
        return HAL_BUSY;
    }

    const uint32_t now = static_cast<uint32_t>(xTaskGetTickCount());
    const uint32_t lastFrame = lastFrameTick_.load(std::memory_order_relaxed);
    const uint32_t timeout = pdMS_TO_TICKS(OFFLINE_RECOVERY_DELAY_MS);

    if (receivedFrameCount_.load(std::memory_order_relaxed) != 0 &&
        static_cast<uint32_t>(now - lastFrame) < timeout) {
        return HAL_BUSY;
    }
    if (static_cast<uint32_t>(now - lastRecoveryTick_) < timeout) {
        return HAL_BUSY;
    }

    lastRecoveryTick_ = now;

    const HAL_StatusTypeDef stopStatus = HAL_UART_DMAStop(uart_.huart_);
    __HAL_UART_DISABLE_IT(uart_.huart_, UART_IT_IDLE);
    if (stopStatus != HAL_OK) {
        return stopStatus;
    }

    xQueueReset(rxQueue_);
    lastRxPosition_ = 0;
    framePosition_ = 0;
    collectingFrame_ = false;

    return uart_.receiveDma(rxBuffer_.data(), RX_BUFFER_LENGTH);
}


void ET08A::applyFrameFormat(uint8_t _format)
{

    (void)HAL_UART_DMAStop(uart_.huart_);
    __HAL_UART_DISABLE_IT(uart_.huart_, UART_IT_IDLE);
    if (uart_.huart_->hdmarx != nullptr) {
        (void)HAL_DMA_Abort(uart_.huart_->hdmarx);
    }


    switch (_format) {
    case FORMAT_E81:
        uart_.huart_->Init.WordLength = UART_WORDLENGTH_9B;
        uart_.huart_->Init.Parity = UART_PARITY_EVEN;
        break;
    case FORMAT_O81:
        uart_.huart_->Init.WordLength = UART_WORDLENGTH_9B;
        uart_.huart_->Init.Parity = UART_PARITY_ODD;
        break;
    case FORMAT_N81:
    default:
        uart_.huart_->Init.WordLength = UART_WORDLENGTH_8B;
        uart_.huart_->Init.Parity = UART_PARITY_NONE;
        break;
    }
    uart_.huart_->Init.StopBits = UART_STOPBITS_1;

    (void)HAL_UART_Init(uart_.huart_);


    xQueueReset(rxQueue_);
    lastRxPosition_ = 0;
    collectingFrame_ = false;
    framePosition_ = 0;

    (void)uart_.receiveDma(rxBuffer_.data(), RX_BUFFER_LENGTH);
}


void ET08A::updateFormatDetection()
{
    if (probeStep_ == FORMAT_PROBE_DONE) {
        return;
    }


    constexpr uint8_t PROBE_ORDER[FORMAT_PROBE_COUNT] = { FORMAT_E81, FORMAT_O81, FORMAT_N81 };

    const uint32_t now = static_cast<uint32_t>(xTaskGetTickCount());

    if (!probeActive_) {

        if (rxEventCount_.load(std::memory_order_relaxed) == 0U) {
            return;
        }

        applyFrameFormat(PROBE_ORDER[probeStep_]);
        probeStartTick_ = now;
        probeStartFrames_ = receivedFrameCount_.load(std::memory_order_relaxed);
        probeStartRestarts_ = rxRestartCount_;
        probeActive_ = true;
        return;
    }

    if (static_cast<uint32_t>(now - probeStartTick_) < pdMS_TO_TICKS(FORMAT_PROBE_WINDOW_MS)) {
        return;
    }

    const uint32_t frames = receivedFrameCount_.load(std::memory_order_relaxed);
    const bool framesOk = frames > probeStartFrames_;
    const bool clean = (rxRestartCount_ - probeStartRestarts_) <= FORMAT_PROBE_MAX_RESTARTS;

    probeActive_ = false;

    if (framesOk && clean) {
        probeStep_ = FORMAT_PROBE_DONE;
        return;
    }


    ++probeStep_;
    if (probeStep_ >= FORMAT_PROBE_COUNT) {
        applyFrameFormat(FORMAT_N81);
        probeStep_ = FORMAT_PROBE_DONE;
    }
}

void ET08A::processRxQueue()
{
    if (rxQueue_ == nullptr) {
        return;
    }

    RxChunk_s chunk{};
    while (xQueueReceive(rxQueue_, &chunk, 0) == pdTRUE) {
        consumeBytes(chunk);
    }
}

ET08A::RcData_s ET08A::getData() const
{
    taskENTER_CRITICAL();
    const RcData_s snapshot = data_;
    taskEXIT_CRITICAL();
    return snapshot;
}

bool ET08A::isOnline()
{
    if (receivedFrameCount_.load(std::memory_order_relaxed) == 0) {
        return false;
    }

    const uint32_t now = static_cast<uint32_t>(xTaskGetTickCount());
    const uint32_t last = lastFrameTick_.load(std::memory_order_relaxed);
    return static_cast<uint32_t>(now - last) <= pdMS_TO_TICKS(100);
}

uint32_t ET08A::receivedFrames() const
{
    return receivedFrameCount_.load(std::memory_order_relaxed);
}

uint32_t ET08A::receivedBytes() const
{
    return receivedByteCount_.load(std::memory_order_relaxed);
}

uint32_t ET08A::rxEvents() const
{
    return rxEventCount_.load(std::memory_order_relaxed);
}

uint32_t ET08A::droppedRxBytes() const
{
    return droppedRxBytes_.load(std::memory_order_relaxed);
}


void ET08A::onRxRestart()
{
    lastRxPosition_ = 0;
    rxRestartCount_ = rxRestartCount_ + 1U;
}

void ET08A::onRxEvent(UART_HandleTypeDef *_huart, uint16_t _position)
{
    if (_huart != uart_.huart_ || _position > RX_BUFFER_LENGTH) {
        return;
    }

    rxEventCount_.fetch_add(1, std::memory_order_relaxed);
    receivedByteCount_.fetch_add(_position >= lastRxPosition_
                                     ? static_cast<uint32_t>(_position - lastRxPosition_)
                                     : static_cast<uint32_t>(RX_BUFFER_LENGTH - lastRxPosition_ + _position),
                                 std::memory_order_relaxed);

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    if (_position == RX_BUFFER_LENGTH) {
        enqueueBytes(lastRxPosition_, RX_BUFFER_LENGTH, higherPriorityTaskWoken);
        lastRxPosition_ = 0;
    } else if (_position < lastRxPosition_) {
        enqueueBytes(lastRxPosition_, RX_BUFFER_LENGTH, higherPriorityTaskWoken);
        enqueueBytes(0, _position, higherPriorityTaskWoken);
        lastRxPosition_ = _position;
    } else {
        enqueueBytes(lastRxPosition_, _position, higherPriorityTaskWoken);
        lastRxPosition_ = _position;
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void ET08A::enqueueBytes(uint16_t _begin, uint16_t _end, BaseType_t &_higherPriorityTaskWoken)
{
    if (_begin == _end) {
        return;
    }

    RxChunk_s chunk{};
    chunk.length = static_cast<uint16_t>(_end - _begin);
    for (uint16_t i = 0; i < chunk.length; ++i) {
        chunk.bytes[i] = rxBuffer_[_begin + i];
    }

    if (xQueueSendFromISR(rxQueue_, &chunk, &_higherPriorityTaskWoken) != pdTRUE) {
        droppedRxBytes_.fetch_add(chunk.length, std::memory_order_relaxed);
    }
}

void ET08A::consumeBytes(const RxChunk_s &_chunk)
{
    for (uint16_t i = 0; i < _chunk.length; ++i) {
        consumeByte(_chunk.bytes[i]);
    }
}

void ET08A::consumeByte(uint8_t _byte)
{
    if (!collectingFrame_) {
        if (_byte != 0x0F) {
            return;
        }
        collectingFrame_ = true;
        framePosition_ = 0;
    }

    frame_[framePosition_++] = _byte;
    if (framePosition_ != FRAME_LENGTH) {
        return;
    }

    collectingFrame_ = false;
    framePosition_ = 0;

    RcData_s decoded{};
    if (decodeFrame(decoded)) {
        taskENTER_CRITICAL();
        data_ = decoded;
        taskEXIT_CRITICAL();
        lastFrameTick_.store(static_cast<uint32_t>(xTaskGetTickCount()), std::memory_order_relaxed);
        receivedFrameCount_.fetch_add(1, std::memory_order_relaxed);
        return;
    }


    uint8_t nextFrameStart = 1;
    while (nextFrameStart < FRAME_LENGTH && frame_[nextFrameStart] != 0x0F) {
        ++nextFrameStart;
    }

    if (nextFrameStart == FRAME_LENGTH) {
        collectingFrame_ = false;
        framePosition_ = 0;
        return;
    }

    framePosition_ = static_cast<uint8_t>(FRAME_LENGTH - nextFrameStart);
    std::memmove(frame_.data(), frame_.data() + nextFrameStart, framePosition_);
}

bool ET08A::decodeFrame(RcData_s &_data) const
{
    if (frame_[0] != 0x0F) {
        return false;
    }

    const uint16_t channels[8] = {
        static_cast<uint16_t>(((frame_[2] << 8) | frame_[1]) & 0x07FF),
        static_cast<uint16_t>(((frame_[3] << 5) | (frame_[2] >> 3)) & 0x07FF),
        static_cast<uint16_t>(((frame_[5] << 10) | (frame_[4] << 2) | (frame_[3] >> 6)) & 0x07FF),
        static_cast<uint16_t>(((frame_[6] << 7) | (frame_[5] >> 1)) & 0x07FF),
        static_cast<uint16_t>(((frame_[7] << 4) | (frame_[6] >> 4)) & 0x07FF),
        static_cast<uint16_t>(((frame_[9] << 9) | (frame_[8] << 1) | (frame_[7] >> 7)) & 0x07FF),
        static_cast<uint16_t>(((frame_[10] << 6) | (frame_[9] >> 2)) & 0x07FF),
        static_cast<uint16_t>(((frame_[11] << 3) | (frame_[10] >> 5)) & 0x07FF),
    };


    _data.ch0 = static_cast<int16_t>(static_cast<int32_t>(channels[0]) - CH_VALUE_OFFSET);
    _data.ch1 = static_cast<int16_t>(static_cast<int32_t>(CH_VALUE_OFFSET) - channels[1]);
    _data.ch2 = static_cast<int16_t>(static_cast<int32_t>(channels[3]) - CH_VALUE_OFFSET);
    _data.ch3 = static_cast<int16_t>(static_cast<int32_t>(channels[2]) - CH_VALUE_OFFSET);
    _data.SB = decodeSwitch(channels[4]);
    _data.SD = decodeSwitch(channels[5]);
    _data.wheel = channels[6];
    _data.SA = decodeSwitch(channels[7]);
    return true;
    //这里是不是要加一些什么
}

ET08A::Sw_e ET08A::decodeSwitch(uint16_t _value)
{
    if (_value < 689) {
        return Sw_e::UP;
    }
    if (_value < 1359) {
        return Sw_e::MID;
    }
    return Sw_e::DOWN;
}
