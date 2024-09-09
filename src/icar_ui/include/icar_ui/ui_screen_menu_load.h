// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.2.1
// LVGL VERSION: 8.2.0
// PROJECT: icar_menu_load

#ifndef _ICAR_MENU_LOAD_UI_H
#define _ICAR_MENU_LOAD_UI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "lvgl/lvgl.h"

    extern lv_obj_t *ui_Screen_Menu_Load;
    void ui_event_Button_Load_Refresh(lv_event_t *e);
    extern lv_obj_t *ui_Button_Load_Refresh;
    extern lv_obj_t *ui_Label_Load_Refresh;
    void ui_event_Button_Load_Open(lv_event_t *e);
    extern lv_obj_t *ui_Button_Load_Open;
    extern lv_obj_t *ui_Label_Load_Open;
    extern lv_obj_t *ui_Roller_Route;

    void cllbck_load_refresh(lv_event_t *e);
    void cllbck_load_open(lv_event_t *e);

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

    void ui_screen_menu_load_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
