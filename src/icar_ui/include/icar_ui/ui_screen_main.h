// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.2.1
// LVGL VERSION: 8.2.0
// PROJECT: icar_main

#ifndef _ICAR_MAIN_UI_H
#define _ICAR_MAIN_UI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "lvgl/lvgl.h"

    extern lv_obj_t *ui_Screen_Main;
    extern lv_obj_t *ui_Panel_Header;
    extern lv_obj_t *ui_Panel_Instrumentation;
    extern lv_obj_t *ui_Panel_Throttle_Brake;
    extern lv_obj_t *ui_Bar_Throttle;
    extern lv_obj_t *ui_Bar_Brake;
    extern lv_obj_t *ui_Label_Throttle;
    extern lv_obj_t *ui_Label_Brake;
    extern lv_obj_t *ui_Panel_Steering;
    extern lv_obj_t *ui_Arc_Steering_Output;
    extern lv_obj_t *ui_Arc_Steering_Input;
    extern lv_obj_t *ui_Label_Steering_Unit;
    extern lv_obj_t *ui_Label_Steering_Shadow;
    extern lv_obj_t *ui_Label_Steering;
    extern lv_obj_t *ui_Panel_Velocity;
    extern lv_obj_t *ui_Arc_Velocity_Output;
    extern lv_obj_t *ui_Arc_Velocity_Input;
    extern lv_obj_t *ui_Label_Velocity_Unit;
    extern lv_obj_t *ui_Label_Velocity_Shadow;
    extern lv_obj_t *ui_Label_Velocity;
    extern lv_obj_t *ui_Panel_Menu_Info;
    extern lv_obj_t *ui_Panel_Menu_Record;
    extern lv_obj_t *ui_Panel_Menu_Load;
    void ui_event_Button_Info(lv_event_t *e);
    extern lv_obj_t *ui_Button_Info;
    extern lv_obj_t *ui_Label_Info;
    void ui_event_Button_Record(lv_event_t *e);
    extern lv_obj_t *ui_Button_Record;
    extern lv_obj_t *ui_Label_Record;
    void ui_event_Button_Load(lv_event_t *e);
    extern lv_obj_t *ui_Button_Load;
    extern lv_obj_t *ui_Label_Load;
    extern lv_obj_t *ui_Panel_Information;
    extern lv_obj_t *ui_Panel_Route;
    extern lv_obj_t *ui_Label_Route;
    extern lv_obj_t *ui_Bar_Route;
    extern lv_obj_t *ui_Panel_Trip;
    extern lv_obj_t *ui_Label_Trip_Unit;
    extern lv_obj_t *ui_Label_Trip;
    extern lv_obj_t *ui_Image_Lambang_ITS;
    extern lv_obj_t *ui_Panel_Start_Stop;
    void ui_event_Button_Start(lv_event_t *e);
    extern lv_obj_t *ui_Button_Start;
    extern lv_obj_t *ui_Label_Start;
    void ui_event_Button_Stop(lv_event_t *e);
    extern lv_obj_t *ui_Button_Stop;
    extern lv_obj_t *ui_Label_Stop;
    void ui_event_Button_Confirm(lv_event_t *e);
    extern lv_obj_t *ui_Button_Confirm;
    extern lv_obj_t *ui_Label_Confirm;
    extern lv_obj_t *ui_Panel_Footer;

    void cllbck_button_info(lv_event_t *e);
    void cllbck_button_record(lv_event_t *e);
    void cllbck_button_load(lv_event_t *e);
    void cllbck_button_start(lv_event_t *e);
    void cllbck_button_stop(lv_event_t *e);
    void cllbck_button_confirm(lv_event_t *e);

    LV_IMG_DECLARE(ui_img_1103760277); // assets\Lambang-ITS-2-300x300.png

    LV_FONT_DECLARE(ui_font_BebasNeueRegular16);
    LV_FONT_DECLARE(ui_font_BebasNeueRegular32);
    LV_FONT_DECLARE(ui_font_BebasNeueRegular48);
    LV_FONT_DECLARE(ui_font_BebasNeueRegular64);
    LV_FONT_DECLARE(ui_font_DSEG7ModernBoldItalic16);
    LV_FONT_DECLARE(ui_font_DSEG7ModernBoldItalic32);
    LV_FONT_DECLARE(ui_font_DSEG7ModernBoldItalic48);
    LV_FONT_DECLARE(ui_font_DSEG7ModernBoldItalic64);
    LV_FONT_DECLARE(ui_font_DSEG7ModernBoldItalic80);
    LV_FONT_DECLARE(ui_font_DSEG7ModernBoldItalic96);

    void ui_screen_main_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
