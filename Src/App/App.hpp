/*
 * @Author: MJH 2264698762@qq.com
 * @Date: 2026-11-01 20:23:47
 * @LastEditors: MJH 2264698762@qq.com
 * @LastEditTime: 2026-11-01 22:16:18
 * @FilePath: \ec_2027_submissions\Src\App\App.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include "Singleton.hpp"
#include "sdkconfig.h"

#if EXTENSION_ET08A
#include "ET08A.hpp"
#include "ServoPwm.hpp"
#include <memory>
#endif

class App : public Singleton<App> {
    // pwm呼吸灯、转舵机，串口，电机pid测试，遥控器上层实现，考核框架
public:
    void init();

    void task();
   
#if EXTENSION_ET08A
    bool readRcData(ET08A::RcData_s &_data);
    uint32_t receivedRcFrames() const;

private:
    std::unique_ptr<ET08A> receiver_;
    std::unique_ptr<ServoPwm> lxServo_;
#endif

};