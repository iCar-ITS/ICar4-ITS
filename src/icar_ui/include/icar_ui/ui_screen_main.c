// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.2.1
// LVGL VERSION: 8.2.0
// PROJECT: icar_main

#include "ui_screen_main.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////
lv_obj_t *ui_Screen_Main;
lv_obj_t *ui_Panel_Header;
lv_obj_t *ui_Panel_Instrumentation;
lv_obj_t *ui_Panel_Throttle_Brake;
lv_obj_t *ui_Bar_Throttle;
lv_obj_t *ui_Bar_Brake;
lv_obj_t *ui_Label_Throttle;
lv_obj_t *ui_Label_Brake;
lv_obj_t *ui_Panel_Steering;
lv_obj_t *ui_Arc_Steering_Output;
lv_obj_t *ui_Arc_Steering_Input;
lv_obj_t *ui_Label_Steering_Unit;
lv_obj_t *ui_Label_Steering_Shadow;
lv_obj_t *ui_Label_Steering;
lv_obj_t *ui_Panel_Velocity;
lv_obj_t *ui_Arc_Velocity_Output;
lv_obj_t *ui_Arc_Velocity_Input;
lv_obj_t *ui_Label_Velocity_Unit;
lv_obj_t *ui_Label_Velocity_Shadow;
lv_obj_t *ui_Label_Velocity;
lv_obj_t *ui_Panel_Menu_Info;
lv_obj_t *ui_Panel_Menu_Record;
lv_obj_t *ui_Panel_Menu_Load;
void ui_event_Button_Info(lv_event_t *e);
lv_obj_t *ui_Button_Info;
lv_obj_t *ui_Label_Info;
void ui_event_Button_Record(lv_event_t *e);
lv_obj_t *ui_Button_Record;
lv_obj_t *ui_Label_Record;
void ui_event_Button_Load(lv_event_t *e);
lv_obj_t *ui_Button_Load;
lv_obj_t *ui_Label_Load;
lv_obj_t *ui_Panel_Information;
lv_obj_t *ui_Panel_Route;
lv_obj_t *ui_Label_Route;
lv_obj_t *ui_Bar_Route;
lv_obj_t *ui_Panel_Trip;
lv_obj_t *ui_Label_Trip_Unit;
lv_obj_t *ui_Label_Trip;
lv_obj_t *ui_Image_Lambang_ITS;
lv_obj_t *ui_Panel_Start_Stop;
void ui_event_Button_Start(lv_event_t *e);
lv_obj_t *ui_Button_Start;
lv_obj_t *ui_Label_Start;
void ui_event_Button_Stop(lv_event_t *e);
lv_obj_t *ui_Button_Stop;
lv_obj_t *ui_Label_Stop;
void ui_event_Button_Confirm(lv_event_t *e);
lv_obj_t *ui_Button_Confirm;
lv_obj_t *ui_Label_Confirm;
lv_obj_t *ui_Panel_Footer;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 32
#error "LV_COLOR_DEPTH should be 32bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP != 0
#error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_Button_Info(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        cllbck_button_info(e);
    }
}
void ui_event_Button_Record(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        cllbck_button_record(e);
    }
}
void ui_event_Button_Load(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        cllbck_button_load(e);
    }
}
void ui_event_Button_Start(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        cllbck_button_start(e);
    }
}
void ui_event_Button_Stop(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        cllbck_button_stop(e);
    }
}
void ui_event_Button_Confirm(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        cllbck_button_confirm(e);
    }
}

///////////////////// SCREENS ////////////////////
void ui_Screen_Main_screen_init(void)
{
    ui_Screen_Main = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen_Main, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_flex_flow(ui_Screen_Main, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(ui_Screen_Main, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_left(ui_Screen_Main, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Screen_Main, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Screen_Main, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Screen_Main, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel_Header = lv_obj_create(ui_Screen_Main);
    lv_obj_set_height(ui_Panel_Header, 50);
    lv_obj_set_width(ui_Panel_Header, lv_pct(100));
    lv_obj_set_align(ui_Panel_Header, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel_Header, LV_OBJ_FLAG_SCROLLABLE); /// Flags

    ui_Panel_Instrumentation = lv_obj_create(ui_Screen_Main);
    lv_obj_set_width(ui_Panel_Instrumentation, LV_SIZE_CONTENT);  /// 0
    lv_obj_set_height(ui_Panel_Instrumentation, LV_SIZE_CONTENT); /// 940
    lv_obj_set_x(ui_Panel_Instrumentation, 17);
    lv_obj_set_y(ui_Panel_Instrumentation, -24);
    lv_obj_set_align(ui_Panel_Instrumentation, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel_Instrumentation, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_pad_left(ui_Panel_Instrumentation, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Instrumentation, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Instrumentation, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Instrumentation, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel_Throttle_Brake = lv_obj_create(ui_Panel_Instrumentation);
    lv_obj_set_width(ui_Panel_Throttle_Brake, 140);
    lv_obj_set_height(ui_Panel_Throttle_Brake, 400);
    lv_obj_clear_flag(ui_Panel_Throttle_Brake, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_shadow_color(ui_Panel_Throttle_Brake, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Panel_Throttle_Brake, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Panel_Throttle_Brake, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Panel_Throttle_Brake, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Bar_Throttle = lv_bar_create(ui_Panel_Throttle_Brake);
    lv_bar_set_value(ui_Bar_Throttle, 50, LV_ANIM_OFF);
    lv_obj_set_width(ui_Bar_Throttle, 40);
    lv_obj_set_height(ui_Bar_Throttle, 300);
    lv_obj_set_x(ui_Bar_Throttle, -30);
    lv_obj_set_y(ui_Bar_Throttle, 0);
    lv_obj_set_align(ui_Bar_Throttle, LV_ALIGN_TOP_MID);
    lv_obj_set_style_bg_color(ui_Bar_Throttle, lv_color_hex(0x2F3237), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Bar_Throttle, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_Bar_Throttle, lv_color_hex(0x20F320), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Bar_Throttle, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_Bar_Brake = lv_bar_create(ui_Panel_Throttle_Brake);
    lv_bar_set_value(ui_Bar_Brake, 50, LV_ANIM_OFF);
    lv_obj_set_width(ui_Bar_Brake, 40);
    lv_obj_set_height(ui_Bar_Brake, 300);
    lv_obj_set_x(ui_Bar_Brake, 30);
    lv_obj_set_y(ui_Bar_Brake, 0);
    lv_obj_set_align(ui_Bar_Brake, LV_ALIGN_TOP_MID);
    lv_obj_set_style_bg_color(ui_Bar_Brake, lv_color_hex(0x2F3237), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Bar_Brake, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_Bar_Brake, lv_color_hex(0xF32020), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Bar_Brake, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_Label_Throttle = lv_label_create(ui_Panel_Throttle_Brake);
    lv_obj_set_width(ui_Label_Throttle, 40);
    lv_obj_set_height(ui_Label_Throttle, 40);
    lv_obj_set_x(ui_Label_Throttle, -30);
    lv_obj_set_y(ui_Label_Throttle, 0);
    lv_obj_set_align(ui_Label_Throttle, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(ui_Label_Throttle, "50%\nTHR");
    lv_obj_set_style_text_align(ui_Label_Throttle, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Throttle, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Brake = lv_label_create(ui_Panel_Throttle_Brake);
    lv_obj_set_width(ui_Label_Brake, 40);
    lv_obj_set_height(ui_Label_Brake, 40);
    lv_obj_set_x(ui_Label_Brake, 30);
    lv_obj_set_y(ui_Label_Brake, 0);
    lv_obj_set_align(ui_Label_Brake, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(ui_Label_Brake, "50%\nBRK");
    lv_obj_set_style_text_align(ui_Label_Brake, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Brake, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel_Steering = lv_obj_create(ui_Panel_Instrumentation);
    lv_obj_set_width(ui_Panel_Steering, 400);
    lv_obj_set_height(ui_Panel_Steering, 400);
    lv_obj_set_x(ui_Panel_Steering, 150);
    lv_obj_set_y(ui_Panel_Steering, 0);
    lv_obj_clear_flag(ui_Panel_Steering, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_shadow_color(ui_Panel_Steering, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Panel_Steering, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Panel_Steering, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Panel_Steering, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Arc_Steering_Output = lv_arc_create(ui_Panel_Steering);
    lv_obj_set_width(ui_Arc_Steering_Output, 360);
    lv_obj_set_height(ui_Arc_Steering_Output, 360);
    lv_obj_set_align(ui_Arc_Steering_Output, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Arc_Steering_Output, LV_OBJ_FLAG_CLICKABLE); /// Flags
    lv_arc_set_range(ui_Arc_Steering_Output, -350, 350);
    lv_arc_set_value(ui_Arc_Steering_Output, 200);
    lv_arc_set_mode(ui_Arc_Steering_Output, LV_ARC_MODE_SYMMETRICAL);
    lv_obj_set_style_arc_width(ui_Arc_Steering_Output, 40, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_width(ui_Arc_Steering_Output, 40, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_Arc_Steering_Input = lv_arc_create(ui_Panel_Steering);
    lv_obj_set_width(ui_Arc_Steering_Input, 240);
    lv_obj_set_height(ui_Arc_Steering_Input, 240);
    lv_obj_set_align(ui_Arc_Steering_Input, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Arc_Steering_Input, LV_OBJ_FLAG_CLICKABLE); /// Flags
    lv_arc_set_range(ui_Arc_Steering_Input, -350, 350);
    lv_arc_set_value(ui_Arc_Steering_Input, 200);
    lv_arc_set_mode(ui_Arc_Steering_Input, LV_ARC_MODE_SYMMETRICAL);
    lv_obj_set_style_arc_width(ui_Arc_Steering_Input, 20, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_width(ui_Arc_Steering_Input, 20, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_Label_Steering_Unit = lv_label_create(ui_Panel_Steering);
    lv_obj_set_width(ui_Label_Steering_Unit, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Steering_Unit, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Steering_Unit, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(ui_Label_Steering_Unit, "deg\nSTEERING");
    lv_obj_set_style_text_align(ui_Label_Steering_Unit, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Steering_Unit, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Steering_Shadow = lv_label_create(ui_Panel_Steering);
    lv_obj_set_width(ui_Label_Steering_Shadow, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Steering_Shadow, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Steering_Shadow, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Steering_Shadow, "88.88\n88.88");
    lv_obj_set_style_text_color(ui_Label_Steering_Shadow, lv_color_hex(0x2F3237), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Steering_Shadow, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui_Label_Steering_Shadow, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_Label_Steering_Shadow, 16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Steering_Shadow, &ui_font_DSEG7ModernBoldItalic48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Steering = lv_label_create(ui_Panel_Steering);
    lv_obj_set_width(ui_Label_Steering, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Steering, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Steering, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Steering, "!20.0\n!20.0");
    lv_obj_set_style_text_color(ui_Label_Steering, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Steering, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui_Label_Steering, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_Label_Steering, 16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Steering, &ui_font_DSEG7ModernBoldItalic48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel_Velocity = lv_obj_create(ui_Panel_Instrumentation);
    lv_obj_set_width(ui_Panel_Velocity, 400);
    lv_obj_set_height(ui_Panel_Velocity, 400);
    lv_obj_set_x(ui_Panel_Velocity, 560);
    lv_obj_set_y(ui_Panel_Velocity, 0);
    lv_obj_clear_flag(ui_Panel_Velocity, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_shadow_color(ui_Panel_Velocity, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Panel_Velocity, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Panel_Velocity, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Panel_Velocity, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Arc_Velocity_Output = lv_arc_create(ui_Panel_Velocity);
    lv_obj_set_width(ui_Arc_Velocity_Output, 360);
    lv_obj_set_height(ui_Arc_Velocity_Output, 360);
    lv_obj_set_align(ui_Arc_Velocity_Output, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Arc_Velocity_Output, LV_OBJ_FLAG_CLICKABLE); /// Flags
    lv_arc_set_range(ui_Arc_Velocity_Output, 0, 250);
    lv_arc_set_value(ui_Arc_Velocity_Output, 100);
    lv_obj_set_style_arc_width(ui_Arc_Velocity_Output, 40, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_width(ui_Arc_Velocity_Output, 40, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_Arc_Velocity_Input = lv_arc_create(ui_Panel_Velocity);
    lv_obj_set_width(ui_Arc_Velocity_Input, 240);
    lv_obj_set_height(ui_Arc_Velocity_Input, 240);
    lv_obj_set_align(ui_Arc_Velocity_Input, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Arc_Velocity_Input, LV_OBJ_FLAG_CLICKABLE); /// Flags
    lv_arc_set_range(ui_Arc_Velocity_Input, 0, 250);
    lv_arc_set_value(ui_Arc_Velocity_Input, 100);
    lv_obj_set_style_arc_width(ui_Arc_Velocity_Input, 20, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_width(ui_Arc_Velocity_Input, 20, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_Label_Velocity_Unit = lv_label_create(ui_Panel_Velocity);
    lv_obj_set_width(ui_Label_Velocity_Unit, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Velocity_Unit, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Velocity_Unit, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(ui_Label_Velocity_Unit, "km/h\nVELOCITY");
    lv_obj_set_style_text_align(ui_Label_Velocity_Unit, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Velocity_Unit, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Velocity_Shadow = lv_label_create(ui_Panel_Velocity);
    lv_obj_set_width(ui_Label_Velocity_Shadow, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Velocity_Shadow, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Velocity_Shadow, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Velocity_Shadow, "88.88\n88.88");
    lv_obj_set_style_text_color(ui_Label_Velocity_Shadow, lv_color_hex(0x2F3237), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Velocity_Shadow, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui_Label_Velocity_Shadow, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_Label_Velocity_Shadow, 16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Velocity_Shadow, &ui_font_DSEG7ModernBoldItalic48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Velocity = lv_label_create(ui_Panel_Velocity);
    lv_obj_set_width(ui_Label_Velocity, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Velocity, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Velocity, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Velocity, "!10.0\n!10.0");
    lv_obj_set_style_text_color(ui_Label_Velocity, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Velocity, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui_Label_Velocity, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_Label_Velocity, 16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Velocity, &ui_font_DSEG7ModernBoldItalic48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel_Menu_Info = lv_obj_create(ui_Panel_Instrumentation);
    lv_obj_set_width(ui_Panel_Menu_Info, 960);
    lv_obj_set_height(ui_Panel_Menu_Info, 435);
    lv_obj_set_x(ui_Panel_Menu_Info, 0);
    lv_obj_set_y(ui_Panel_Menu_Info, 410);
    lv_obj_clear_flag(ui_Panel_Menu_Info, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_shadow_color(ui_Panel_Menu_Info, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Panel_Menu_Info, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Panel_Menu_Info, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Panel_Menu_Info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Menu_Info, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Menu_Info, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Menu_Info, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Menu_Info, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel_Menu_Record = lv_obj_create(ui_Panel_Instrumentation);
    lv_obj_set_width(ui_Panel_Menu_Record, 960);
    lv_obj_set_height(ui_Panel_Menu_Record, 435);
    lv_obj_set_x(ui_Panel_Menu_Record, 0);
    lv_obj_set_y(ui_Panel_Menu_Record, 410);
    lv_obj_clear_flag(ui_Panel_Menu_Record, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_shadow_color(ui_Panel_Menu_Record, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Panel_Menu_Record, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Panel_Menu_Record, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Panel_Menu_Record, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Menu_Record, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Menu_Record, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Menu_Record, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Menu_Record, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel_Menu_Load = lv_obj_create(ui_Panel_Instrumentation);
    lv_obj_set_width(ui_Panel_Menu_Load, 960);
    lv_obj_set_height(ui_Panel_Menu_Load, 435);
    lv_obj_set_x(ui_Panel_Menu_Load, 0);
    lv_obj_set_y(ui_Panel_Menu_Load, 410);
    lv_obj_clear_flag(ui_Panel_Menu_Load, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_shadow_color(ui_Panel_Menu_Load, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Panel_Menu_Load, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Panel_Menu_Load, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Panel_Menu_Load, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Menu_Load, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Menu_Load, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Menu_Load, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Menu_Load, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button_Info = lv_btn_create(ui_Panel_Instrumentation);
    lv_obj_set_width(ui_Button_Info, 150);
    lv_obj_set_height(ui_Button_Info, 60);
    lv_obj_set_x(ui_Button_Info, 0);
    lv_obj_set_y(ui_Button_Info, 855);
    lv_obj_add_flag(ui_Button_Info, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_Button_Info, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_border_color(ui_Button_Info, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button_Info, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button_Info, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button_Info, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button_Info, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Button_Info, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Button_Info, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Info = lv_label_create(ui_Button_Info);
    lv_obj_set_width(ui_Label_Info, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Info, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Info, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Info, "INFO");
    lv_obj_set_style_text_color(ui_Label_Info, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Info, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Info, &ui_font_BebasNeueRegular48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button_Record = lv_btn_create(ui_Panel_Instrumentation);
    lv_obj_set_width(ui_Button_Record, 150);
    lv_obj_set_height(ui_Button_Record, 60);
    lv_obj_set_x(ui_Button_Record, 160);
    lv_obj_set_y(ui_Button_Record, 0);
    lv_obj_set_align(ui_Button_Record, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_add_flag(ui_Button_Record, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_Button_Record, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_border_color(ui_Button_Record, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button_Record, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button_Record, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button_Record, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button_Record, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Button_Record, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Button_Record, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Record = lv_label_create(ui_Button_Record);
    lv_obj_set_width(ui_Label_Record, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Record, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Record, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Record, "RECORD");
    lv_obj_set_style_text_color(ui_Label_Record, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Record, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Record, &ui_font_BebasNeueRegular48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button_Load = lv_btn_create(ui_Panel_Instrumentation);
    lv_obj_set_width(ui_Button_Load, 150);
    lv_obj_set_height(ui_Button_Load, 60);
    lv_obj_set_x(ui_Button_Load, 320);
    lv_obj_set_y(ui_Button_Load, 0);
    lv_obj_set_align(ui_Button_Load, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_add_flag(ui_Button_Load, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_Button_Load, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_border_color(ui_Button_Load, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button_Load, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button_Load, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button_Load, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button_Load, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Button_Load, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Button_Load, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Load = lv_label_create(ui_Button_Load);
    lv_obj_set_width(ui_Label_Load, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Load, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Load, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Load, "LOAD");
    lv_obj_set_style_text_color(ui_Label_Load, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Load, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Load, &ui_font_BebasNeueRegular48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel_Information = lv_obj_create(ui_Screen_Main);
    lv_obj_set_width(ui_Panel_Information, LV_SIZE_CONTENT);  /// 0
    lv_obj_set_height(ui_Panel_Information, LV_SIZE_CONTENT); /// 940
    lv_obj_set_x(ui_Panel_Information, 1);
    lv_obj_set_y(ui_Panel_Information, -4);
    lv_obj_set_align(ui_Panel_Information, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel_Information, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_pad_left(ui_Panel_Information, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Information, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Information, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Information, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel_Route = lv_obj_create(ui_Panel_Information);
    lv_obj_set_width(ui_Panel_Route, 435);
    lv_obj_set_height(ui_Panel_Route, 915);
    lv_obj_clear_flag(ui_Panel_Route, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_shadow_color(ui_Panel_Route, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Panel_Route, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Panel_Route, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Panel_Route, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Route, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Route, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Route, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Route, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Route = lv_label_create(ui_Panel_Route);
    lv_obj_set_width(ui_Label_Route, 300);
    lv_obj_set_height(ui_Label_Route, LV_SIZE_CONTENT); /// 1
    lv_obj_set_x(ui_Label_Route, -55);
    lv_obj_set_y(ui_Label_Route, 0);
    lv_obj_set_align(ui_Label_Route, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Route,
                      "ASRAMA\nMAHASISWA\n\nKANTIN\nPUSAT\n\nREKTORAT\nITS\n\n#ffffff TAMAN\n#ffffff ALUMNI\n\nBUNDARAN\nITS\n\nMASJID\nMANARUL ILMI\n\nASRAMA\nMAHASISWA");
    lv_label_set_recolor(ui_Label_Route, "true");
    lv_obj_set_style_text_color(ui_Label_Route, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Route, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui_Label_Route, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_Label_Route, 8, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Label_Route, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Route, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Bar_Route = lv_bar_create(ui_Panel_Route);
    lv_bar_set_range(ui_Bar_Route, 0, 600);
    lv_bar_set_value(ui_Bar_Route, 300, LV_ANIM_OFF);
    lv_obj_set_width(ui_Bar_Route, 100);
    lv_obj_set_height(ui_Bar_Route, 890);
    lv_obj_set_x(ui_Bar_Route, 155);
    lv_obj_set_y(ui_Bar_Route, 0);
    lv_obj_set_align(ui_Bar_Route, LV_ALIGN_CENTER);

    ui_Panel_Trip = lv_obj_create(ui_Panel_Information);
    lv_obj_set_width(ui_Panel_Trip, 435);
    lv_obj_set_height(ui_Panel_Trip, 775);
    lv_obj_set_x(ui_Panel_Trip, 445);
    lv_obj_set_y(ui_Panel_Trip, 0);
    lv_obj_clear_flag(ui_Panel_Trip, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_shadow_color(ui_Panel_Trip, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Panel_Trip, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Panel_Trip, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Panel_Trip, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Trip, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Trip, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Trip, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Trip, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Trip_Unit = lv_label_create(ui_Panel_Trip);
    lv_obj_set_width(ui_Label_Trip_Unit, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Trip_Unit, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Trip_Unit, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_Label_Trip_Unit, "DISTANCE (est.)\nTIME (est.)");
    lv_obj_set_style_text_letter_space(ui_Label_Trip_Unit, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_Label_Trip_Unit, 168, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Label_Trip_Unit, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Trip_Unit, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Trip = lv_label_create(ui_Panel_Trip);
    lv_obj_set_width(ui_Label_Trip, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Trip, LV_SIZE_CONTENT); /// 1
    lv_obj_set_x(ui_Label_Trip, 0);
    lv_obj_set_y(ui_Label_Trip, 52);
    lv_obj_set_align(ui_Label_Trip, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_Label_Trip, "0000\n00:00");
    lv_obj_set_style_text_letter_space(ui_Label_Trip, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_Label_Trip, 112, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Label_Trip, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Trip, &ui_font_DSEG7ModernBoldItalic96, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image_Lambang_ITS = lv_img_create(ui_Panel_Trip);
    lv_img_set_src(ui_Image_Lambang_ITS, &ui_img_1103760277);
    lv_obj_set_width(ui_Image_Lambang_ITS, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Image_Lambang_ITS, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Image_Lambang_ITS, LV_ALIGN_BOTTOM_MID);
    lv_obj_add_flag(ui_Image_Lambang_ITS, LV_OBJ_FLAG_ADV_HITTEST);  /// Flags
    lv_obj_clear_flag(ui_Image_Lambang_ITS, LV_OBJ_FLAG_SCROLLABLE); /// Flags

    ui_Panel_Start_Stop = lv_obj_create(ui_Panel_Information);
    lv_obj_set_width(ui_Panel_Start_Stop, 435);
    lv_obj_set_height(ui_Panel_Start_Stop, 125);
    lv_obj_set_x(ui_Panel_Start_Stop, 445);
    lv_obj_set_y(ui_Panel_Start_Stop, 785);
    lv_obj_clear_flag(ui_Panel_Start_Stop, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_shadow_color(ui_Panel_Start_Stop, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Panel_Start_Stop, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Panel_Start_Stop, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Panel_Start_Stop, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Panel_Start_Stop, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel_Start_Stop, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel_Start_Stop, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel_Start_Stop, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button_Start = lv_btn_create(ui_Panel_Start_Stop);
    lv_obj_set_width(ui_Button_Start, 200);
    lv_obj_set_height(ui_Button_Start, 100);
    lv_obj_set_x(ui_Button_Start, -105);
    lv_obj_set_y(ui_Button_Start, 0);
    lv_obj_set_align(ui_Button_Start, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button_Start, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_Button_Start, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_bg_color(ui_Button_Start, lv_color_hex(0x20F320), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button_Start, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button_Start, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button_Start, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button_Start, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button_Start, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button_Start, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Button_Start, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Button_Start, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Start = lv_label_create(ui_Button_Start);
    lv_obj_set_width(ui_Label_Start, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Start, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Start, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Start, "START");
    lv_obj_set_style_text_color(ui_Label_Start, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Start, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Start, &ui_font_BebasNeueRegular64, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button_Stop = lv_btn_create(ui_Panel_Start_Stop);
    lv_obj_set_width(ui_Button_Stop, 200);
    lv_obj_set_height(ui_Button_Stop, 100);
    lv_obj_set_x(ui_Button_Stop, 105);
    lv_obj_set_y(ui_Button_Stop, 0);
    lv_obj_set_align(ui_Button_Stop, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button_Stop, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_Button_Stop, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_bg_color(ui_Button_Stop, lv_color_hex(0xF32020), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button_Stop, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button_Stop, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button_Stop, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button_Stop, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button_Stop, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button_Stop, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Button_Stop, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Button_Stop, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Stop = lv_label_create(ui_Button_Stop);
    lv_obj_set_width(ui_Label_Stop, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Stop, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Stop, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Stop, "STOP");
    lv_obj_set_style_text_color(ui_Label_Stop, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Stop, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Stop, &ui_font_BebasNeueRegular64, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button_Confirm = lv_btn_create(ui_Panel_Start_Stop);
    lv_obj_set_width(ui_Button_Confirm, 410);
    lv_obj_set_height(ui_Button_Confirm, 100);
    lv_obj_set_align(ui_Button_Confirm, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button_Confirm, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_Button_Confirm, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_bg_color(ui_Button_Confirm, lv_color_hex(0xF3F31F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button_Confirm, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Button_Confirm, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button_Confirm, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button_Confirm, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button_Confirm, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button_Confirm, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Button_Confirm, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Button_Confirm, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Confirm = lv_label_create(ui_Button_Confirm);
    lv_obj_set_width(ui_Label_Confirm, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Confirm, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Confirm, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Confirm, "!!! CONFIRM !!!");
    lv_obj_set_style_text_color(ui_Label_Confirm, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Confirm, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Confirm, &ui_font_BebasNeueRegular64, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel_Footer = lv_obj_create(ui_Screen_Main);
    lv_obj_set_height(ui_Panel_Footer, 50);
    lv_obj_set_width(ui_Panel_Footer, lv_pct(100));
    lv_obj_set_align(ui_Panel_Footer, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel_Footer, LV_OBJ_FLAG_SCROLLABLE); /// Flags

    lv_obj_add_event_cb(ui_Button_Info, ui_event_Button_Info, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button_Record, ui_event_Button_Record, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button_Load, ui_event_Button_Load, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button_Start, ui_event_Button_Start, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button_Stop, ui_event_Button_Stop, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button_Confirm, ui_event_Button_Confirm, LV_EVENT_ALL, NULL);
}

void ui_screen_main_init(void)
{
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                              true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Screen_Main_screen_init();
    lv_disp_load_scr(ui_Screen_Main);
}
