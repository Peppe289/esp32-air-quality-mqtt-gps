#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include <stdint.h>

/* LED configuration */
#define LED_GREEN_GPIO 3
#define LED_RED_GPIO 4

static TaskHandle_t s_xHandle_blinking = NULL;
static const char *TAG = "LED";

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
  gpio_set_level(LED_RED_GPIO, 0);
  gpio_set_level(LED_GREEN_GPIO, 0);

  for (;;) {
    gpio_set_level(((uint32_t)args), 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(((uint32_t)args), 0);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

static void led_blink_all_task() {
  gpio_set_level(LED_RED_GPIO, 0);
  gpio_set_level(LED_GREEN_GPIO, 0);

  for (;;) {
    gpio_set_level(LED_RED_GPIO, 0);
    gpio_set_level(LED_GREEN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(LED_GREEN_GPIO, 0);
    gpio_set_level(LED_RED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void led_blink_all() {
  led_blink_reset();

  xTaskCreate(led_blink_all_task, "led_blinking", 4096, (void *)LED_RED_GPIO, 2,
              &s_xHandle_blinking);
}

void led_red_blink() {
  led_blink_reset();

  xTaskCreate(led_blink_task, "led_blinking", 4096, (void *)LED_RED_GPIO, 2,
              &s_xHandle_blinking);
}

void led_green_blink() {
  led_blink_reset();

  xTaskCreate(led_blink_task, "led_blinking", 4096, (void *)LED_GREEN_GPIO, 2,
              &s_xHandle_blinking);
}
