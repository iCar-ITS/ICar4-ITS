// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.2.1
// LVGL VERSION: 8.2.0
// PROJECT: icar_menu_load

#include "ui_screen_menu_load.h"
#include "ui_helpers.h"
#include "ui_screen_main.h"

///////////////////// VARIABLES ////////////////////
lv_obj_t *ui_Screen_Menu_Load;
void ui_event_Button_Load_Refresh(lv_event_t *e);
lv_obj_t *ui_Button_Load_Refresh;
lv_obj_t *ui_Label_Load_Refresh;
void ui_event_Button_Load_Open(lv_event_t *e);
lv_obj_t *ui_Button_Load_Open;
lv_obj_t *ui_Label_Load_Open;
lv_obj_t *ui_Roller_Route;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 32
#error "LV_COLOR_DEPTH should be 32bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP != 0
#error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_Button_Load_Refresh(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        cllbck_load_refresh(e);
    }
}
void ui_event_Button_Load_Open(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);
    if (event_code == LV_EVENT_CLICKED)
    {
        cllbck_load_open(e);
    }
}

///////////////////// SCREENS ////////////////////
void ui_Screen_Menu_Load_screen_init(void)
{
    ui_Screen_Menu_Load = ui_Panel_Menu_Load;
    lv_obj_clear_flag(ui_Screen_Menu_Load, LV_OBJ_FLAG_SCROLLABLE); /// Flags

    ui_Button_Load_Refresh = lv_btn_create(ui_Screen_Menu_Load);
    lv_obj_set_width(ui_Button_Load_Refresh, 200);
    lv_obj_set_height(ui_Button_Load_Refresh, 200);
    lv_obj_add_flag(ui_Button_Load_Refresh, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_Button_Load_Refresh, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_border_color(ui_Button_Load_Refresh, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button_Load_Refresh, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button_Load_Refresh, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button_Load_Refresh, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button_Load_Refresh, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Button_Load_Refresh, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Button_Load_Refresh, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Load_Refresh = lv_label_create(ui_Button_Load_Refresh);
    lv_obj_set_width(ui_Label_Load_Refresh, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Load_Refresh, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Load_Refresh, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Load_Refresh, "REFRESH");
    lv_obj_set_style_text_color(ui_Label_Load_Refresh, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Load_Refresh, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Load_Refresh, &ui_font_BebasNeueRegular64, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button_Load_Open = lv_btn_create(ui_Screen_Menu_Load);
    lv_obj_set_width(ui_Button_Load_Open, 200);
    lv_obj_set_height(ui_Button_Load_Open, 200);
    lv_obj_set_align(ui_Button_Load_Open, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_add_flag(ui_Button_Load_Open, LV_OBJ_FLAG_SCROLL_ON_FOCUS); /// Flags
    lv_obj_clear_flag(ui_Button_Load_Open, LV_OBJ_FLAG_SCROLLABLE);    /// Flags
    lv_obj_set_style_border_color(ui_Button_Load_Open, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Button_Load_Open, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Button_Load_Open, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Button_Load_Open, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Button_Load_Open, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Button_Load_Open, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Button_Load_Open, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Load_Open = lv_label_create(ui_Button_Load_Open);
    lv_obj_set_width(ui_Label_Load_Open, LV_SIZE_CONTENT);  /// 1
    lv_obj_set_height(ui_Label_Load_Open, LV_SIZE_CONTENT); /// 1
    lv_obj_set_align(ui_Label_Load_Open, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label_Load_Open, "OPEN");
    lv_obj_set_style_text_color(ui_Label_Load_Open, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Load_Open, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Load_Open, &ui_font_BebasNeueRegular64, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Roller_Route = lv_roller_create(ui_Screen_Menu_Load);
    lv_roller_set_options(ui_Roller_Route, "Option 1\nOption 2\nOption 3", LV_ROLLER_MODE_NORMAL);
    lv_obj_set_width(ui_Roller_Route, 410);
    lv_obj_set_height(ui_Roller_Route, 410);
    lv_obj_set_x(ui_Roller_Route, 210);
    lv_obj_set_y(ui_Roller_Route, 0);
    lv_obj_set_align(ui_Roller_Route, LV_ALIGN_LEFT_MID);
    lv_obj_set_style_text_font(ui_Roller_Route, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Roller_Route, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Roller_Route, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Roller_Route, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_Roller_Route, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_Roller_Route, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui_Roller_Route, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_spread(ui_Roller_Route, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_font(ui_Roller_Route, &lv_font_montserrat_24, LV_PART_SELECTED | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Button_Load_Refresh, ui_event_Button_Load_Refresh, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button_Load_Open, ui_event_Button_Load_Open, LV_EVENT_ALL, NULL);
}

void ui_screen_menu_load_init(void)
{
    ui_Screen_Menu_Load_screen_init();
}
