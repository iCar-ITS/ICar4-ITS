// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.2.1
// LVGL VERSION: 8.2.0
// PROJECT: icar_menu_record

#include "ui_screen_menu_record.h"
#include "ui_helpers.h"
#include "ui_screen_main.h"

///////////////////// VARIABLES ////////////////////
lv_obj_t *ui_Screen_Menu_Record;
void ui_event_Button_Record_Start(lv_event_t *e);
lv_obj_t *ui_Button_Record_Start;
lv_obj_t *ui_Label_Record_Start;
void ui_event_Button_Record_Stop(lv_event_t *e);
lv_obj_t *ui_Button_Record_Stop;
lv_obj_t *ui_Label_Record_Stop;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 32
#error "LV_COLOR_DEPTH should be 32bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP != 0
#error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_Button_Record_Start(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        cllbck_record_start(e);
    }
}
void ui_event_Button_Record_Stop(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        cllbck_record_stop(e);
    }
}

///////////////////// SCREENS ////////////////////
void ui_Screen_Menu_Record_screen_init(void)
{
    ui_Screen_Menu_Record = ui_Panel_Menu_Record;
    lv_obj_clear_flag(ui_Screen_Menu_Record, LV_OBJ_FLAG_SCROLLABLE); /// Flags

    ui_Button_Record_Start = lv_btn_create(ui_Screen_Menu_Record);
    lv_obj_set_width(ui_Button_Record_Start, 200);
    lv_obj_set_height(ui_Button_Record_Start, 200);
    lv_obj_add_flag(ui_Button_Record_Start, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_Button_Record_Start, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_border_color(ui_Button_Record_Start, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button_Record_Start, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button_Record_Start, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button_Record_Start, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button_Record_Start, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Button_Record_Start, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Button_Record_Start, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Record_Start = lv_label_create(ui_Button_Record_Start);
    lv_obj_set_width(ui_Label_Record_Start, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Record_Start, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Record_Start, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Record_Start, "START");
    lv_obj_set_style_text_color(ui_Label_Record_Start, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Record_Start, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Record_Start, &ui_font_BebasNeueRegular64, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button_Record_Stop = lv_btn_create(ui_Screen_Menu_Record);
    lv_obj_set_width(ui_Button_Record_Stop, 200);
    lv_obj_set_height(ui_Button_Record_Stop, 200);
    lv_obj_set_align(ui_Button_Record_Stop, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_add_flag(ui_Button_Record_Stop, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_Button_Record_Stop, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_border_color(ui_Button_Record_Stop, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button_Record_Stop, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button_Record_Stop, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button_Record_Stop, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button_Record_Stop, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Button_Record_Stop, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Button_Record_Stop, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Record_Stop = lv_label_create(ui_Button_Record_Stop);
    lv_obj_set_width(ui_Label_Record_Stop, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Record_Stop, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Record_Stop, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Record_Stop, "STOP");
    lv_obj_set_style_text_color(ui_Label_Record_Stop, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Record_Stop, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Record_Stop, &ui_font_BebasNeueRegular64, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Button_Record_Start, ui_event_Button_Record_Start, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button_Record_Stop, ui_event_Button_Record_Stop, LV_EVENT_ALL, NULL);
}

void ui_screen_menu_record_init(void)
{
    ui_Screen_Menu_Record_screen_init();
}
