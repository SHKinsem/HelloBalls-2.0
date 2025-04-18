#include "app_button.h"
#include "app_twai.h"
#include "ui.h"
#include "bldc_motors.h"
#include "debugger.h"
#include "ui_events.h"
#include "app_motors.h"
#include "esp_timer.h"

#define TAG "DEBUG"
int16_t output = 200;

base_motor_t* motor_ptr[4] = {nullptr, nullptr, nullptr, nullptr}; // Array of motor pointers


void motor_displayer(void* arg){
    // Create a task for motor control
    setGlobalSpeedPtr(motor_ptr[0]->getTargetSpeedPtr());
    setGlobalSpeedPtr2(motor_ptr[1]->getTargetSpeedPtr());

    motor_ptr[0]->setPIDParameters(0.06, 0.0005, 0.00001, 1.0, 0.0, 5000.0, -5000.0); // Set PID parameters
    motor_ptr[1]->setPIDParameters(0.03, 0.001, 0.00001, 1.0, 0.0, 5000.0, -5000.0); // Set PID parameters

    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 100 ms
    int counter = 0; // Counter for the run time
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(200));
        if (counter < 10) {
            lv_textarea_set_text(ui_MainMenuTextareaTextArea2, motor_ptr[0]->getMotorInfo()); // Update the UI with motor info
        }
        if (counter > 10 && counter < 20) {
            lv_textarea_set_text(ui_MainMenuTextareaTextArea2, motor_ptr[1]->getMotorInfo()); // Update the UI with motor info
        }
        if (counter > 20) {
            counter = 0; // Reset the counter
        }
        counter++;
    }
}

#define RUN_MOTOR_PERIOD 20000 // Run motor for 10000 ms
#define MESSAGE_FREQUENCY 15 // Frequency of sending messages in ms

void debugLoggingTask(void *arg) {
    while (1) {
        // Log the current speed of the motor
        int counter = RUN_MOTOR_PERIOD / MESSAGE_FREQUENCY; // Counter for the run time
        twai_transmit_speed(0, 0); // Transmit the speed to the motor
        if(*get_button_state_ptr()) {

            motor_ptr[0]->enable(); // Use the pointer to enable the motor
            motor_ptr[1]->enable(); // Use the pointer to enable the motor
            ESP_LOGI(TAG, "Motor started, output: %d", output);
            lv_obj_set_style_bg_color(ui_MainMenuButtonTestButton, lv_color_hex(0xFF0085), LV_PART_MAIN | LV_STATE_DEFAULT);

            while(get_twai_running() && counter > 0) {
                // output = motor_ptr[0]->getControlOutput(); // Get the control output from the motor
                // output = motor_ptr[1]->getControlOutput();

                twai_transmit_speed(motor_ptr[0]->getControlOutput(), motor_ptr[1]->getControlOutput()); // Transmit the speed to the motor
                // ESP_LOGI(TAG, "O: %d, R: %d, T: %d", output, motor_ptr[0]->getRawSpeed(), motor_ptr[0]->getTargetSpeed());
                vTaskDelay(pdMS_TO_TICKS(MESSAGE_FREQUENCY)); // Adjusted delay for consistent timing
                counter--;
            }
            motor_ptr[0]->disable();
            motor_ptr[1]->disable();
            ESP_LOGI(TAG, "Motor stopped, counter: %d", counter);
            *get_button_state_ptr() = false; // Reset button state
            lv_obj_set_style_bg_color(ui_MainMenuButtonTestButton, lv_color_hex(0x838383), LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        // ESP_LOGI(TAG, "R: %d, T: %d", motor_ptr[0]->getRawSpeed(), motor_ptr[0]->getTargetSpeed());

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void measure_important_function(void) {
    const unsigned MEASUREMENTS = 5000;
    uint64_t start = esp_timer_get_time();

    for (int retries = 0; retries < MEASUREMENTS; retries++) {
        twai_transmit_speed(motor_ptr[0]->getControlOutput(), motor_ptr[1]->getControlOutput()); // Transmit the speed to the motor
    }

    uint64_t end = esp_timer_get_time();

    printf("%u iterations took %llu milliseconds (%llu microseconds per invocation)\n",
           MEASUREMENTS, (end - start)/1000, (end - start)/MEASUREMENTS);
}

void motor_task_init(){
    twai_init();
    motor_ptr[0] = get_motor_ptr(1); // Get the pointer to motor 1
    motor_ptr[1] = get_motor_ptr(2);

    // Create a task for motor control
    xTaskCreatePinnedToCore(motor_displayer, "Motor displayer Task", 4096, NULL, 3, NULL, tskNO_AFFINITY);
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 ms
    xTaskCreatePinnedToCore(debugLoggingTask, "Debug Logging Task", 4096, NULL, 6, NULL, tskNO_AFFINITY);
}