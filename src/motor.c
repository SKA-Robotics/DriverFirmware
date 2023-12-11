#include "motor.h"
#include "mag_alpha_driver.h"
#include <math.h>

void SetMotorDuty(motor_t* motor, float duty) {
    uint32_t pwm = fabs(duty) * 1023;
    if (pwm > MAX_PWM) {
        pwm = MAX_PWM;
    }
    if (duty >= 0) {
        *(motor->pwmChannelForward) = pwm;
        *(motor->pwmChannelReverse) = 0;
        motor->state.direction = FORWARD;
    } else {
        *(motor->pwmChannelForward) = 0;
        *(motor->pwmChannelReverse) = pwm;
        motor->state.direction = REVERSE;
    }
}

void UpdateMotorState(motor_t* motor) {
    uint16_t positionMeasurement =
        readMagAlphaAngle(motor->encoderCsPort, motor->encoderCsPin);
    long deltaPositionRaw =
        positionMeasurement - motor->state.prevPositionMeasurement;
    motor->state.prevPositionMeasurement = positionMeasurement;

    if (deltaPositionRaw > ENCODER_REVOLUTION_THRESHOLD) {
        // gone below 0
        deltaPositionRaw -= ENCODER_RESOLUTION;
    } else if (deltaPositionRaw < -ENCODER_REVOLUTION_THRESHOLD) {
        // gone above 65536
        deltaPositionRaw += ENCODER_RESOLUTION;
    }

    motor->state.positionRaw += deltaPositionRaw;

    motor->state.position =
        motor->state.positionRaw * 2 * PI / ENCODER_RESOLUTION;

    motor->state.velocity =
        0.05f * (deltaPositionRaw * 2 * PI / ENCODER_RESOLUTION / DELTA_TIME) +
        0.95f * motor->state.prevVelocity;
    motor->state.prevVelocity = motor->state.velocity;

    uint16_t adc_value = readAdc(motor->adcChannel);
    float scaled_adc_value = (float)adc_value * 0.0015f;
    motor->state.current = (motor->state.direction == FORWARD)
                               ? scaled_adc_value
                               : -scaled_adc_value;
}