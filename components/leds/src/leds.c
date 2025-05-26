#include "leds.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"


// Set to 1 to use DMA for driving the LED strip, 0 otherwise
// Please note the RMT DMA feature is only available on chips e.g. ESP32-S3/P4
#define LED_STRIP_USE_DMA  0

#define FRESH_RATE 30 // 30Hz refresh rate

// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 12
#define LED_STRIP_MEMORY_BLOCK_WORDS 0 // let the driver choose a proper memory block size automatically

// GPIO assignment
#define LED_STRIP_GPIO_PIN  LED_PIN

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

static const char *TAG = "LED";
led_strip_handle_t led_strip = NULL;
led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN, // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = LED_STRIP_MEMORY_BLOCK_WORDS, // the memory block size used by the RMT channel
        .flags = {
            .with_dma = LED_STRIP_USE_DMA,     // Using DMA can improve performance when driving more LEDs
        }
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

void led_refresh_task(void *arg)
{
    led_strip_handle_t led_strip = (led_strip_handle_t)arg;
    while (1) {
        // Refresh the strip to send data
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000/FRESH_RATE)); // Delay for 1 second
    }
}

void led_init(void)
{
    led_strip = configure_led();
    ESP_LOGI(TAG, "LED strip initialized");

    // Create a task to refresh the LED strip
    // xTaskCreate(led_refresh_task, "led_refresh_task", 2048, (void *)led_strip, 5, NULL);
}

State_t previous_state = STARTING; // Initialize previous state
uint32_t prev_red = 0, prev_green = 0, prev_blue = 0; // Initialize previous color values

void fade_led_strip(led_strip_handle_t led_strip, uint32_t target_red, uint32_t target_green, uint32_t target_blue, int fade_in) {
    // Number of fade steps
    const int fade_steps = 50;  // Reduced steps for faster fade
    
    // Update previous color for next fade
    if (fade_in) {
        // When fading in, store target colors for next operation
        prev_red = target_red;
        prev_green = target_green;
        prev_blue = target_blue;
    }
    
    for (int step = 0; step <= fade_steps; step++) {
        uint32_t r, g, b;
        
        if (fade_in) {
            // Fade in: from black (0,0,0) to target color
            r = target_red * step / fade_steps;
            g = target_green * step / fade_steps;
            b = target_blue * step / fade_steps;
        } else {
            // Fade out: from previous color to black
            // Using the stored prev_ values for current color
            r = prev_red * (fade_steps - step) / fade_steps;
            g = prev_green * (fade_steps - step) / fade_steps;
            b = prev_blue * (fade_steps - step) / fade_steps;
        }
        
        // Set all LEDs to the same color
        for (int led = 0; led < LED_STRIP_LED_COUNT; led++) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, led, r, g, b));
        }
        
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        vTaskDelay(pdMS_TO_TICKS(10));  // Slightly faster refresh for smoother fade
    }
}


void update_led_state(led_strip_handle_t led_strip, State_t state) {
    if(state == previous_state && state != STARTING) {
        return; // No state change, do nothing
    }
    
    // Only fade out if we have some non-zero previous colors
    if (prev_red > 0 || prev_green > 0 || prev_blue > 0) {
        fade_led_strip(led_strip, 0, 0, 0, 0); // Fade out current color using prev_* values
    }
    
    previous_state = state; // Update previous state
    switch (state) {
        case STARTING:
            fade_led_strip(led_strip, 100, 255, 100, 1); // Fade in green
            break;
        case MACHINE_IDLE:
            fade_led_strip(led_strip, 30, 30, 255, 1); // Fade in blue
            break;
        case STANDBY:
            fade_led_strip(led_strip, 50, 255, 50, 1); // Fade in green
            break;
        case SEARCHING_BALL:
            fade_led_strip(led_strip, 255, 255, 30, 1); // Fade in yellow
            break;
        case SEARCHING_HUMAN:
            fade_led_strip(led_strip, 255, 165, 30, 1); // Fade in orange
            break;
        case RETRIEVING_BALL:
            fade_led_strip(led_strip, 128, 30, 128, 1); // Fade in purple
            break;
        case STOPPED:
            fade_led_strip(led_strip, 255, 192, 203, 1); // Fade in pink
            break;
        case ERROR:
            fade_led_strip(led_strip, 255, 30, 30, 1); // Fade in red
            break;
    }
}

void update_led_state_noHandle(State_t state) {
    update_led_state(led_strip, state); // Update LED state
}

void led_example() {
    led_strip_handle_t led_strip = configure_led(); // Initialize LED strip
    // led_init(); // Initialize LED strip task

    // Example usage of update_led_state function
    while (1) {
        update_led_state(led_strip, STARTING);
        vTaskDelay(pdMS_TO_TICKS(2000));
        update_led_state(led_strip, MACHINE_IDLE);
        vTaskDelay(pdMS_TO_TICKS(2000));
        update_led_state(led_strip, STANDBY);
        vTaskDelay(pdMS_TO_TICKS(2000));
        update_led_state(led_strip, SEARCHING_BALL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        update_led_state(led_strip, SEARCHING_HUMAN);
        vTaskDelay(pdMS_TO_TICKS(2000));
        update_led_state(led_strip, RETRIEVING_BALL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        update_led_state(led_strip, STOPPED);
        vTaskDelay(pdMS_TO_TICKS(2000));
        update_led_state(led_strip, ERROR);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void led_on_off_example(void)
{
    led_strip_handle_t led_strip = configure_led();
    bool led_on_off = false;

    ESP_LOGI(TAG, "Start blinking LED strip");
    while (1) {
        if (led_on_off) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 5, 5, 5));
            }
            /* Refresh the strip to send data */
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            ESP_LOGI(TAG, "LED ON!");
        } else {
            /* Set all LED off to clear all pixels */
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
            ESP_LOGI(TAG, "LED OFF!");
        }

        led_on_off = !led_on_off;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}