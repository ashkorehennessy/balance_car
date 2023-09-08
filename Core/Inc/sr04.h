//
// Created by ashkore on 23-9-2.
//


#ifndef SR04_SR04_H
#define SR04_SR04_H

#include "stm32f1xx_hal.h"

class SR04 {
private:
    GPIO_TypeDef *trig_port;  // Trigger pin port
    uint16_t trig_pin;  // Trigger pin number
    TIM_HandleTypeDef *echo_htim;  // Echo pin timer
    uint16_t echo_channel;  // Echo pin timer channel
    uint8_t capture_flag;  // Echo pin capture flag
    uint32_t distance;  // Distance in mm
    uint16_t sound_speed;  // Sound speed in m/s
public:
    SR04(GPIO_TypeDef *trig_port, uint16_t trig_pin, TIM_HandleTypeDef *echo_htim, uint16_t echo_channel, uint16_t sound_speed=340);

    void init();  // Initialize SR04 and timer

    void trigger();  // Send trigger pulse

    void measure_distance();  // Measure distance and update distance variable

    [[nodiscard]] uint32_t get_distance() const { return this->distance;}  // Get distance in mm

    uint16_t tim_update_count;  // Timer update count for calibration distance
};

#endif //SR04_SR04_H