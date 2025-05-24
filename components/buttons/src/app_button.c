#include "app_button.h"
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "app_twai.h"
#include "esp_log.h"

// Include the servo control interface
extern void toggle_servo(void); // External function from app_motors.h

#define TAG "BUTTON"

static QueueHandle_t gpio_evt_queue = NULL;

// GPIO interrupt service routine
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// GPIO task function
static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            
            // If it's GPIO0, toggle the servo position
            if (io_num == GPIO_NUM_0) {
                // Wait a moment to debounce
                vTaskDelay(pdMS_TO_TICKS(100));
                
                // Toggle servo position using the C-compatible function
                toggle_servo();
            }
        }
    }
}

void button_init()
{
    gpio_config_t io0_conf = {
        .intr_type = GPIO_INTR_NEGEDGE, // Falling edge interrupt
        .mode = GPIO_MODE_INPUT,        // Input mode
        .pin_bit_mask = 1<<GPIO_NUM_0,  // Select GPIO0
        .pull_down_en = 0,              // Disable internal pull-down
        .pull_up_en = 1                 // Enable internal pull-up
    };
    
    // Configure GPIO based on the settings above
    gpio_config(&io0_conf);
    
    // Create a queue to handle GPIO events
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 5120, NULL, 10, NULL);
    
    // Create GPIO interrupt service
    gpio_install_isr_service(0);
    
    // Add interrupt handler for GPIO0
    gpio_isr_handler_add(GPIO_NUM_0, gpio_isr_handler, (void*) GPIO_NUM_0);
    
    ESP_LOGI(TAG, "Button handler initialized for servo control on GPIO0");
}