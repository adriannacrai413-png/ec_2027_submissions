#include "ServoPwm.hpp"
#include <algorithm>

ServoPwm::ServoPwm(TIM_HandleTypeDef *_timer, uint32_t _channel) : timer_(_timer), channel_(_channel) {}

HAL_StatusTypeDef ServoPwm::init()
{
    return HAL_ERROR;
}

void ServoPwm::setFromRc(int16_t)
{
}

void ServoPwm::setCenter()
{
    
}
