#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

// LED引脚定义（可以在包含此头文件前自定义）
#ifndef BLINK_LED_PIN
#define BLINK_LED_PIN 21
#endif

/**
 * @brief 初始化LED
 */
void led_init(void);

/**
 * @brief 打开LED
 */
void led_on(void);

/**
 * @brief 关闭LED
 */
void led_off(void);

/**
 * @brief 切换LED状态
 */
void led_toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_CONTROL_H */