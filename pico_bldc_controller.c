#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pico_uart_transports.h"
#include "pico/stdio.h"
#include "pico/time.h"

#define MOTOR_PIN 15
#define MOTOR_PIN2 14
#define DIR_PIN1 16
#define DIR_PIN2 17
#define PWM_RANGE 255 // Neuer PWM-Bereich

rcl_subscription_t subscription1, subscription2;
std_msgs__msg__Int32 MOTOR_pwm_msg1, MOTOR_pwm_msg2;

void MOTOR_pwm_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    int32_t pwm_value = msg->data;
    uint16_t pwm_abs = (uint16_t)((pwm_value > 0) ? pwm_value : -pwm_value); // absolute value of pwm
    pwm_abs = (pwm_abs > PWM_RANGE) ? PWM_RANGE : pwm_abs; // Ensure pwm_abs is within range

    // Scale the pwm value to the new range
    uint16_t pwm_scaled = (uint16_t)((pwm_abs / 255.0f) * PWM_RANGE);

    // Set direction based on pwm sign
    gpio_put(DIR_PIN1, (pwm_value >= 0) ? 0 : 1);
    pwm_set_gpio_level(MOTOR_PIN, pwm_scaled);
}

void MOTOR2_pwm_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    int32_t pwm_value = msg->data;
    uint16_t pwm_abs = (uint16_t)((pwm_value > 0) ? pwm_value : -pwm_value); // absolute value of pwm
    pwm_abs = (pwm_abs > PWM_RANGE) ? PWM_RANGE : pwm_abs; // Ensure pwm_abs is within range

    // Scale the pwm value to the new range
    uint16_t pwm_scaled = (uint16_t)((pwm_abs / 255.0f) * PWM_RANGE);

    // Set direction based on pwm sign
    gpio_put(DIR_PIN2, (pwm_value >= 0) ? 0 : 1);
    pwm_set_gpio_level(MOTOR_PIN2, pwm_scaled);
}

void MOTOR_pwm_init() {
    pwm_set_wrap(pwm_gpio_to_slice_num(MOTOR_PIN), PWM_RANGE);
    gpio_set_function(MOTOR_PIN, GPIO_FUNC_PWM);
    pwm_set_enabled(pwm_gpio_to_slice_num(MOTOR_PIN), true);
}

void MOTOR2_pwm_init() {
    pwm_set_wrap(pwm_gpio_to_slice_num(MOTOR_PIN2), PWM_RANGE);
    gpio_set_function(MOTOR_PIN2, GPIO_FUNC_PWM);
    pwm_set_enabled(pwm_gpio_to_slice_num(MOTOR_PIN2), true);
}

void dir_pins_init() {
    gpio_init(DIR_PIN1);
    gpio_init(DIR_PIN2);
    gpio_set_dir(DIR_PIN1, GPIO_OUT);
    gpio_set_dir(DIR_PIN2, GPIO_OUT);
}

int main() {
    stdio_init_all();
    MOTOR_pwm_init();
    MOTOR2_pwm_init();
    dir_pins_init();

    // Initialize micro-ROS Custom Transport
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_subscription_init_default(&subscription1, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "MOTOR_pwm");

    rclc_executor_init(&executor, &support.context, 2, &allocator);

    rclc_executor_add_subscription(&executor, &subscription1, &MOTOR_pwm_msg1, &MOTOR_pwm_callback, ON_NEW_DATA);

    rclc_subscription_init_default(&subscription2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "MOTOR2_pwm");

    rclc_executor_add_subscription(&executor, &subscription2, &MOTOR_pwm_msg2, &MOTOR2_pwm_callback, ON_NEW_DATA);

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    return 0;
}
