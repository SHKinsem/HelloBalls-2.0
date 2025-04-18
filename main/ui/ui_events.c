// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"
#include "ui_events.h"
#include "app_twai.h"
#define TAG "UI_EVT"

int16_t global_speed = 0; // Global speed variable
float global_angle = 0; // Global angle variable
bool button_state = false; // Button state variable

int16_t* global_speed_ptr = &global_speed; // Pointer to global speed variable
int16_t* global_speed_ptr2 = &global_speed; // Pointer to global speed variable

void lvgl_can_task(lv_event_t * e)
{
	// can_task();
	button_state = true; // Set button state to true
}

void lvgl_start_twai_receive(lv_event_t * e)
{
	// Your code here
	start_twai_receive();
}

void lvgl_stop_twai_receive(lv_event_t * e)
{
	// Your code here
	stop_twai_receive();
}

int16_t* getGlobalSpeed() { return &global_speed; }
void setGlobalSpeedPtr(int16_t* ptr) { global_speed_ptr = ptr;}
void setGlobalSpeedPtr2(int16_t* ptr) { global_speed_ptr2 = ptr;}
// float getGlobalAngle() { return global_angle; }

void lvgl_set_global_speed(lv_event_t * e)
{
	lv_obj_t * target = lv_event_get_target(e);
	// global_speed = (int16_t)lv_slider_get_value(target);
	*global_speed_ptr = lv_slider_get_value(target);
	*global_speed_ptr2 = -lv_slider_get_value(target);
}

bool* get_button_state_ptr(){
	return &button_state;
}