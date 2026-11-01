#pragma once

#include "Bsp.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include <array>
#include <atomic>
#include <cstdint>

class ET08A final {
public:
    static constexpr uint16_t FRAME_LENGTH = 25;
    static constexpr uint16_t RX_BUFFER_LENGTH = 2 * FRAME_LENGTH;

    static constexpr uint16_t CH_VALUE_MIN = 353;
    static constexpr uint16_t CH_VALUE_OFFSET = 1024;
    static constexpr uint16_t CH_VALUE_MAX = 1694;

    enum class Sw_e : uint16_t { UP = 353, MID = 1024, DOWN = 1694 };

    struct RcData_s {
        int16_t ch0 = 0;
        int16_t ch1 = 0;
        int16_t ch2 = 0;
        int16_t ch3 = 0;
        Sw_e SB = Sw_e::MID;
        Sw_e SA = Sw_e::MID;
        Sw_e SD = Sw_e::MID;
        uint16_t wheel = CH_VALUE_OFFSET;
    };

    explicit ET08A(UART_HandleTypeDef *_huart);
    ~ET08A();

    ET08A(const ET08A &) = delete;
    ET08A &operator=(const ET08A &) = delete;

    HAL_StatusTypeDef start();
    HAL_StatusTypeDef recoverIfOffline();
    void processRxQueue();


    void updateFormatDetection();

    RcData_s getData() const;
    bool isOnline();
    uint32_t receivedFrames() const;
    uint32_t receivedBytes() const;
    uint32_t rxEvents() const;
    uint32_t droppedRxBytes() const;

private:
    struct RxChunk_s {
        uint16_t length = 0;
        std::array<uint8_t, RX_BUFFER_LENGTH> bytes{};
    };

    static constexpr uint8_t RX_QUEUE_LENGTH = 4;

    void onRxEvent(UART_HandleTypeDef *_huart, uint16_t _position);

    void onRxRestart();


    enum : uint8_t {
        FORMAT_N81 = 0,        
        FORMAT_E81 = 1,        
        FORMAT_O81 = 2,       
        FORMAT_PROBE_COUNT = 3,
        FORMAT_PROBE_DONE = 0xFF,
    };

    void applyFrameFormat(uint8_t _format);

    uint8_t probeStep_ = 0; 
    bool probeActive_ = false;
    uint32_t probeStartTick_ = 0;
    uint32_t probeStartFrames_ = 0;
    uint32_t probeStartRestarts_ = 0;

    void enqueueBytes(uint16_t _begin, uint16_t _end, BaseType_t &_higherPriorityTaskWoken);
    void consumeBytes(const RxChunk_s &_chunk);
    void consumeByte(uint8_t _byte);
    bool decodeFrame(RcData_s &_data) const;
    static Sw_e decodeSwitch(uint16_t _value);

    Uart uart_;
    StaticQueue_t rxQueueControl_{};
    alignas(8) std::array<uint8_t, RX_QUEUE_LENGTH * sizeof(RxChunk_s)> rxQueueStorage_{};
    QueueHandle_t rxQueue_ = nullptr;
    alignas(4) std::array<uint8_t, RX_BUFFER_LENGTH> rxBuffer_{};
    std::array<uint8_t, FRAME_LENGTH> frame_{};
    uint16_t lastRxPosition_ = 0;
    uint8_t framePosition_ = 0;
    bool collectingFrame_ = false;
    bool started_ = false;
    RcData_s data_{};
    std::atomic<uint32_t> receivedFrameCount_{0};
    std::atomic<uint32_t> receivedByteCount_{0};
    std::atomic<uint32_t> rxEventCount_{0};
    std::atomic<uint32_t> droppedRxBytes_{0};
    std::atomic<uint32_t> lastFrameTick_{0};
    volatile uint32_t rxRestartCount_ = 0; 
    uint32_t lastRecoveryTick_ = 0;
};
