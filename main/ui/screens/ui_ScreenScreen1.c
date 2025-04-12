// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "../ui.h"

void ui_ScreenScreen1_screen_init(void)
{
    ui_ScreenScreen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ScreenScreen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Screen1ButtonTestButton = lv_btn_create(ui_ScreenScreen1);
    lv_obj_set_width(ui_Screen1ButtonTestButton, 122);
    lv_obj_set_height(ui_Screen1ButtonTestButton, 50);
    lv_obj_set_align(ui_Screen1ButtonTestButton, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Screen1ButtonTestButton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Screen1ButtonTestButton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen1ButtonTestButton, lv_color_hex(0x4E4E4E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen1ButtonTestButton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Screen1LabelTestButtonLabel = lv_label_create(ui_Screen1ButtonTestButton);
    lv_obj_set_width(ui_Screen1LabelTestButtonLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Screen1LabelTestButtonLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Screen1LabelTestButtonLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Screen1LabelTestButtonLabel, "RUN MOTOR");
    lv_obj_set_style_text_color(ui_Screen1LabelTestButtonLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Screen1LabelTestButtonLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Screen1ButtonTestButton, ui_event_Screen1ButtonTestButton, LV_EVENT_ALL, NULL);

}
