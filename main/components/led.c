#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include <stdint.h>

/* LED configuration */
#define LED_GREEN_GPIO 3
#define LED_RED_GPIO 4

static TaskHandle_t s_xHandle_blinking = NULL;
static const char *TAG = "LED";

#define LED_BLINK_LOW_SPEED 1000
#define LED_BLINK_FAST_SPEED 200

typedef struct {
  uint32_t GPIO_num;
  uint32_t delay_ms;
} led_blink_config_t;

/**
 * @brief Initializes onboard status LEDs.
 * * Sets Red LED as Power/Status indicator and Green LED for
 * Connectivity/Activity.
 */
void led_init(void) {
  gpio_reset_pin(LED_GREEN_GPIO);
  gpio_set_direction(LED_GREEN_GPIO, GPIO_MODE_OUTPUT);

  gpio_reset_pin(LED_RED_GPIO);
  gpio_set_direction(LED_RED_GPIO, GPIO_MODE_OUTPUT);

  ESP_LOGI(TAG, "LEDs initialized: Green=GPIO%d (OFF), Red=GPIO%d (ON)",
           LED_GREEN_GPIO, LED_RED_GPIO);
}

static void led_blink_reset() {
  if (s_xHandle_blinking) {
    vTaskDelete(s_xHandle_blinking);
    s_xHandle_blinking = NULL;
  }
}

void led_set_red() {
  led_blink_reset();
  gpio_set_level(LED_GREEN_GPIO, 0);
  gpio_set_level(LED_RED_GPIO, 1);
}

void led_set_green() {
  led_blink_reset();
  gpio_set_level(LED_RED_GPIO, 0);
  gpio_set_level(LED_GREEN_GPIO, 1);
}

static void led_blink_task(void *args) {
  led_blink_config_t *config = (led_blink_config_t *)args;

  gpio_set_level(LED_RED_GPIO, 0);
  gpio_set_level(LED_GREEN_GPIO, 0);

  for (;;) {
    gpio_set_level(((uint32_t)config->GPIO_num), 1);
    vTaskDelay(pdMS_TO_TICKS(config->delay_ms));
    gpio_set_level(((uint32_t)config->GPIO_num), 0);
    vTaskDelay(pdMS_TO_TICKS(config->delay_ms));
  }
}

static void led_blink_all_task(void *args) {
  led_blink_config_t *config = (led_blink_config_t *)args;

  gpio_set_level(LED_RED_GPIO, 0);
  gpio_set_level(LED_GREEN_GPIO, 0);

  for (;;) {
    gpio_set_level(LED_RED_GPIO, 0);
    gpio_set_level(LED_GREEN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(config->delay_ms));
    gpio_set_level(LED_GREEN_GPIO, 0);
    gpio_set_level(LED_RED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(config->delay_ms));
  }
}

void led_blink_all() {
  static const led_blink_config_t config = {0, LED_BLINK_LOW_SPEED};

  led_blink_reset();
  xTaskCreate(led_blink_all_task, "led_blinking", 4096, (void *)&config, 2,
              &s_xHandle_blinking);
}

void led_blink_all_fast() {
  static const led_blink_config_t config = {0, LED_BLINK_FAST_SPEED};

  led_blink_reset();
  xTaskCreate(led_blink_all_task, "led_blinking", 4096, (void *)&config, 2,
              &s_xHandle_blinking);
}

void led_red_blink_fast() {
  static const led_blink_config_t config = {LED_RED_GPIO, LED_BLINK_FAST_SPEED};

  led_blink_reset();
  xTaskCreate(led_blink_task, "led_blinking", 4096, (void *)&config, 2,
              &s_xHandle_blinking);
}

void led_green_blink_fast() {
  static const led_blink_config_t config = {LED_GREEN_GPIO, LED_BLINK_FAST_SPEED};

  led_blink_reset();
  xTaskCreate(led_blink_task, "led_blinking", 4096, (void *)&config, 2,
              &s_xHandle_blinking);
}

void led_red_blink() {
  static const led_blink_config_t config = {LED_RED_GPIO, LED_BLINK_LOW_SPEED};

  led_blink_reset();
  xTaskCreate(led_blink_task, "led_blinking", 4096, (void *)&config, 2,
              &s_xHandle_blinking);
}

void led_green_blink() {
  static const led_blink_config_t config = {LED_GREEN_GPIO, LED_BLINK_LOW_SPEED};

  led_blink_reset();
  xTaskCreate(led_blink_task, "led_blinking", 4096, (void *)&config, 2,
              &s_xHandle_blinking);
}
