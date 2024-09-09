#include "boost/thread/mutex.hpp"
#include "icar_ui/ui_input_data.h"
#include "icar_ui/ui_input_keyboard.h"
#include "icar_ui/ui_input_pointer.h"
#include "icar_ui/ui_output_data.h"
#include "icar_ui/ui_output_image.h"
#include "icar_ui/ui_output_sound.h"
#include "icar_ui/ui_screen_main.h"
#include "icar_ui/ui_screen_menu_load.h"
#include "icar_ui/ui_screen_menu_record.h"
#include "lvgl/lvgl.h"
#include "ps_ros_lib/help_log.h"
#include "ros/ros.h"

#define DISPLAY_WIDTH 1920
#define DISPLAY_HEIGHT 1080

#define HIDE_OBJ(obj) lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN)
#define SHOW_OBJ(obj) lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN)
#define DARKEN_BTN(btn) lv_obj_set_style_bg_color(btn, lv_color_hex(0x264057), LV_PART_MAIN | LV_STATE_DEFAULT)
#define LIGHTEN_BTN(btn) lv_obj_set_style_bg_color(btn, lv_color_hex(0x2196f3), LV_PART_MAIN | LV_STATE_DEFAULT)
#define DISABLE_BTN(btn)                                                                         \
    {                                                                                            \
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);                                           \
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x2f3237), LV_PART_MAIN | LV_STATE_DEFAULT); \
    }
#define ENABLE_BTN(btn)                                                                          \
    {                                                                                            \
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);                                             \
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x2196f3), LV_PART_MAIN | LV_STATE_DEFAULT); \
    }

//=====Prototype
void cllbck_tim_50hz(const ros::TimerEvent &event);
void cllbck_tim_100hz(const ros::TimerEvent &event);

void cllbck_sub_ui_input_data(const icar_ui::ui_input_dataConstPtr &msg);
void cllbck_sub_ui_input_pointer(const icar_ui::ui_input_pointerConstPtr &msg);
void cllbck_sub_ui_input_keyboard(const icar_ui::ui_input_keyboardConstPtr &msg);

int ui_process_init();
int ui_process_routine();

void flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color);
void monitor_cb(lv_disp_drv_t *disp_drv, uint32_t time, uint32_t px);
void read_pointer_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
void read_keyboard_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);

void play_sound(std::string filename);

//=====Timer
ros::Timer tim_50hz;
ros::Timer tim_100hz;
//=====Subscriber
ros::Subscriber sub_ui_input_data;
ros::Subscriber sub_ui_input_pointer;
ros::Subscriber sub_ui_input_keyboard;
//=====Publisher
ros::Publisher pub_ui_output_data;
ros::Publisher pub_ui_output_image;
ros::Publisher pub_ui_output_sound;
//=====Mutex
boost::mutex mutex_ui;
boost::mutex mutex_ui_input_data;
boost::mutex mutex_ui_output_data;
//=====Help
help_log _log;

//-----UI data
//============
icar_ui::ui_input_data ui_input_data;
icar_ui::ui_output_data ui_output_data;

//-----UI output image
//====================
bool ui_output_image_refresh = false;

//-----Input device
//=================
icar_ui::ui_input_pointer indev_pointer;
icar_ui::ui_input_keyboard indev_keyboard;

//-----Start Stop
//===============
ros::Time time_confirm;
bool start_button_pressed = false;
bool stop_button_pressed = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ui_process");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    //=====Timer
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim_50hz);
    tim_100hz = NH.createTimer(ros::Duration(0.01), cllbck_tim_100hz);
    //=====Subscriber
    sub_ui_input_data = NH.subscribe("/ui_input/data", 1, cllbck_sub_ui_input_data);
    sub_ui_input_pointer = NH.subscribe("/ui_input/pointer", 1, cllbck_sub_ui_input_pointer);
    sub_ui_input_keyboard = NH.subscribe("/ui_input/keyboard", 1, cllbck_sub_ui_input_keyboard);
    //=====Publisher
    pub_ui_output_data = NH.advertise<icar_ui::ui_output_data>("ui_output/data", 0);
    pub_ui_output_image = NH.advertise<icar_ui::ui_output_image>("ui_output/image", 0);
    pub_ui_output_sound = NH.advertise<icar_ui::ui_output_sound>("ui_output/sound", 0);
    //=====Help
    _log.init(NH);

    if (ui_process_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();

    tim_50hz.stop();
    tim_100hz.stop();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_tim_50hz(const ros::TimerEvent &event)
{
    mutex_ui.lock();
    mutex_ui_input_data.lock();

    //----------------------------------
    char buf[32];
    //----------------------------------

    static std::string last_route_option;
    if (ui_input_data.route_option != last_route_option)
    {
        last_route_option = ui_input_data.route_option;
        lv_roller_set_options(ui_Roller_Route, ui_input_data.route_option.c_str(), LV_ROLLER_MODE_NORMAL);
    }

    if (ui_input_data.route_option != "N/A")
        ENABLE_BTN(ui_Button_Load_Open)
    else
        DISABLE_BTN(ui_Button_Load_Open)

    lv_bar_set_value(ui_Bar_Throttle, ui_input_data.throttle, LV_ANIM_ON);
    lv_bar_set_value(ui_Bar_Brake, ui_input_data.brake, LV_ANIM_ON);
    lv_label_set_text_fmt(ui_Label_Throttle, "%d%%\nTHR", ui_input_data.throttle);
    lv_label_set_text_fmt(ui_Label_Brake, "%d%%\nBRK", ui_input_data.brake);

    lv_arc_set_value(ui_Arc_Velocity_Output, ui_input_data.velocity_output * 10);
    lv_arc_set_value(ui_Arc_Velocity_Input, ui_input_data.velocity_input * 10);
    sprintf(buf, "%5.2f\n%5.2f", ui_input_data.velocity_output, ui_input_data.velocity_input);
    for (int i = 0; i < 11; i++)
        if (buf[i] == ' ')
            buf[i] = '!';
    lv_label_set_text(ui_Label_Velocity, buf);

    lv_arc_set_value(ui_Arc_Steering_Output, ui_input_data.steering_output * 10);
    lv_arc_set_value(ui_Arc_Steering_Input, ui_input_data.steering_input * 10);
    sprintf(buf, "%5.1f\n%5.1f", ui_input_data.steering_output, ui_input_data.steering_input);
    for (int i = 0; i < 11; i++)
        if (buf[i] == ' ')
            buf[i] = '!';
    lv_label_set_text(ui_Label_Steering, buf);

    mutex_ui_input_data.unlock();
    mutex_ui.unlock();
}

void cllbck_tim_100hz(const ros::TimerEvent &event)
{
    if (ui_process_routine() == -1)
        ros::shutdown();
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_sub_ui_input_data(const icar_ui::ui_input_dataConstPtr &msg)
{
    mutex_ui_input_data.lock();
    ui_input_data = *msg;
    mutex_ui_input_data.unlock();
}

void cllbck_sub_ui_input_pointer(const icar_ui::ui_input_pointerConstPtr &msg)
{
    indev_pointer = *msg;
}

void cllbck_sub_ui_input_keyboard(const icar_ui::ui_input_keyboardConstPtr &msg)
{
    indev_keyboard = *msg;
}

//------------------------------------------------------------------------------
//==============================================================================

int ui_process_init()
{
    ros::Duration(2).sleep();

    // =================================

    lv_init();

    static lv_disp_draw_buf_t disp_draw_buf;
    static lv_color_t buf_1[DISPLAY_WIDTH * DISPLAY_HEIGHT / 10];
    static lv_color_t buf_2[DISPLAY_WIDTH * DISPLAY_HEIGHT / 10];
    lv_disp_draw_buf_init(&disp_draw_buf, buf_1, buf_2, DISPLAY_WIDTH * DISPLAY_HEIGHT / 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.draw_buf = &disp_draw_buf;
    disp_drv.flush_cb = flush_cb;
    disp_drv.monitor_cb = monitor_cb;
    disp_drv.hor_res = DISPLAY_WIDTH;
    disp_drv.ver_res = DISPLAY_HEIGHT;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_pointer_drv;
    lv_indev_drv_init(&indev_pointer_drv);
    indev_pointer_drv.type = LV_INDEV_TYPE_POINTER;
    indev_pointer_drv.read_cb = read_pointer_cb;
    lv_indev_drv_register(&indev_pointer_drv);

    static lv_indev_drv_t indev_keyboard_drv;
    lv_indev_drv_init(&indev_keyboard_drv);
    indev_keyboard_drv.type = LV_INDEV_TYPE_KEYPAD;
    indev_keyboard_drv.read_cb = read_keyboard_cb;
    lv_indev_drv_register(&indev_keyboard_drv);

    // =================================

    ui_screen_main_init();
    ui_screen_menu_load_init();
    ui_screen_menu_record_init();

    // =================================

    SHOW_OBJ(ui_Panel_Menu_Info);
    HIDE_OBJ(ui_Panel_Menu_Record);
    HIDE_OBJ(ui_Panel_Menu_Load);
    LIGHTEN_BTN(ui_Button_Info);
    DARKEN_BTN(ui_Button_Record);
    DARKEN_BTN(ui_Button_Load);

    SHOW_OBJ(ui_Button_Start);
    SHOW_OBJ(ui_Button_Stop);
    HIDE_OBJ(ui_Button_Confirm);

    ENABLE_BTN(ui_Button_Record_Start);
    DISABLE_BTN(ui_Button_Record_Stop);

    ENABLE_BTN(ui_Button_Load_Refresh);
    DISABLE_BTN(ui_Button_Load_Open);
    lv_roller_set_options(ui_Roller_Route, "N/A", LV_ROLLER_MODE_NORMAL);

    return 0;
}

int ui_process_routine()
{
    mutex_ui.lock();
    lv_tick_inc(10);
    lv_task_handler();
    mutex_ui.unlock();

    if (ros::Time::now() - time_confirm > ros::Duration(5.0))
    {
        start_button_pressed = false;
        stop_button_pressed = false;
        HIDE_OBJ(ui_Button_Confirm);
    }

    return 0;
}

//------------------------------------------------------------------------------
//==============================================================================

void flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color)
{
    icar_ui::ui_output_image msg_ui_output_image;
    msg_ui_output_image.x = area->x1;
    msg_ui_output_image.y = area->y1;
    msg_ui_output_image.w = area->x2 - area->x1 + 1;
    msg_ui_output_image.h = area->y2 - area->y1 + 1;
    for (int i = 0; i < msg_ui_output_image.w * msg_ui_output_image.h; i++)
    {
        msg_ui_output_image.data.push_back(color->ch.red);
        msg_ui_output_image.data.push_back(color->ch.green);
        msg_ui_output_image.data.push_back(color->ch.blue);
        color++;
    }
    msg_ui_output_image.refresh = ui_output_image_refresh;
    pub_ui_output_image.publish(msg_ui_output_image);

    lv_disp_flush_ready(disp_drv);

    /* Reset the refresh flag */
    ui_output_image_refresh = false;
}

void monitor_cb(lv_disp_drv_t *disp_drv, uint32_t time, uint32_t px)
{
    /* Set the refresh flag */
    ui_output_image_refresh = true;
}

void read_pointer_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    data->point.x = indev_pointer.x;
    data->point.y = indev_pointer.y;
    data->state = indev_pointer.pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

void read_keyboard_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    data->key = indev_keyboard.key;
    data->state = indev_keyboard.pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

//------------------------------------------------------------------------------
//==============================================================================

void play_sound(uint8_t id, uint8_t action, std::string filename)
{
    icar_ui::ui_output_sound msg_ui_output_sound;
    msg_ui_output_sound.id = id;
    msg_ui_output_sound.action = action;
    msg_ui_output_sound.filename = filename;
    pub_ui_output_sound.publish(msg_ui_output_sound);
}

//------------------------------------------------------------------------------
//==============================================================================

void cllbck_button_start(lv_event_t *e)
{
    _log.warn("\"Start\" button pressed");
    play_sound(0, icar_ui::ui_output_sound::ACTION_PLAY_ONCE, "beep.wav");

    time_confirm = ros::Time::now();

    start_button_pressed = true;
    SHOW_OBJ(ui_Button_Confirm);
}

void cllbck_button_stop(lv_event_t *e)
{
    _log.warn("\"Stop\" button pressed");
    play_sound(0, icar_ui::ui_output_sound::ACTION_PLAY_ONCE, "beep.wav");

    time_confirm = ros::Time::now();

    stop_button_pressed = true;
    SHOW_OBJ(ui_Button_Confirm);
}

void cllbck_button_confirm(lv_event_t *e)
{
    _log.warn("\"Confirm\" button pressed");
    play_sound(0, icar_ui::ui_output_sound::ACTION_PLAY_ONCE, "beep.wav");

    if (start_button_pressed)
    {
        ui_output_data.button_code = icar_ui::ui_output_data::BUTTON_START;
        pub_ui_output_data.publish(ui_output_data);
    }
    else if (stop_button_pressed)
    {
        ui_output_data.button_code = icar_ui::ui_output_data::BUTTON_STOP;
        pub_ui_output_data.publish(ui_output_data);
    }

    start_button_pressed = false;
    stop_button_pressed = false;
    HIDE_OBJ(ui_Button_Confirm);
}

void cllbck_button_info(lv_event_t *e)
{
    _log.warn("\"Info\" button pressed");
    play_sound(0, icar_ui::ui_output_sound::ACTION_PLAY_ONCE, "beep.wav");

    SHOW_OBJ(ui_Panel_Menu_Info);
    HIDE_OBJ(ui_Panel_Menu_Record);
    HIDE_OBJ(ui_Panel_Menu_Load);
    LIGHTEN_BTN(ui_Button_Info);
    DARKEN_BTN(ui_Button_Record);
    DARKEN_BTN(ui_Button_Load);
}

void cllbck_button_record(lv_event_t *e)
{
    _log.warn("\"Record\" button pressed");
    play_sound(0, icar_ui::ui_output_sound::ACTION_PLAY_ONCE, "beep.wav");

    HIDE_OBJ(ui_Panel_Menu_Info);
    SHOW_OBJ(ui_Panel_Menu_Record);
    HIDE_OBJ(ui_Panel_Menu_Load);
    DARKEN_BTN(ui_Button_Info);
    LIGHTEN_BTN(ui_Button_Record);
    DARKEN_BTN(ui_Button_Load);
}

void cllbck_button_load(lv_event_t *e)
{
    _log.warn("\"Load\" button pressed");
    play_sound(0, icar_ui::ui_output_sound::ACTION_PLAY_ONCE, "beep.wav");

    HIDE_OBJ(ui_Panel_Menu_Info);
    HIDE_OBJ(ui_Panel_Menu_Record);
    SHOW_OBJ(ui_Panel_Menu_Load);
    DARKEN_BTN(ui_Button_Info);
    DARKEN_BTN(ui_Button_Record);
    LIGHTEN_BTN(ui_Button_Load);
}

void cllbck_load_refresh(lv_event_t *e)
{
    _log.warn("\"Menu Refresh\" button pressed");
    play_sound(0, icar_ui::ui_output_sound::ACTION_PLAY_ONCE, "beep.wav");

    ui_output_data.button_code = icar_ui::ui_output_data::BUTTON_LOAD_REFRESH;
    pub_ui_output_data.publish(ui_output_data);
}

void cllbck_load_open(lv_event_t *e)
{
    _log.warn("\"Menu Open\" button pressed");
    play_sound(0, icar_ui::ui_output_sound::ACTION_PLAY_ONCE, "beep.wav");

    /* The below code is getting the selected string from
    the roller and storing it in the route_option variable. */
    char route_option[64];
    lv_roller_get_selected_str(ui_Roller_Route, route_option, sizeof(route_option));
    ui_output_data.route_option = std::string(route_option);

    ui_output_data.button_code = icar_ui::ui_output_data::BUTTON_LOAD_OPEN;
    pub_ui_output_data.publish(ui_output_data);
}

void cllbck_record_start(lv_event_t *e)
{
    _log.warn("\"Record Start\" button pressed");
    play_sound(0, icar_ui::ui_output_sound::ACTION_PLAY_ONCE, "beep.wav");

    DISABLE_BTN(ui_Button_Record_Start);
    ENABLE_BTN(ui_Button_Record_Stop);

    ui_output_data.button_code = icar_ui::ui_output_data::BUTTON_RECORD_START;
    pub_ui_output_data.publish(ui_output_data);
}

void cllbck_record_stop(lv_event_t *e)
{
    _log.warn("\"Record Stop\" button pressed");
    play_sound(0, icar_ui::ui_output_sound::ACTION_PLAY_ONCE, "beep.wav");

    ENABLE_BTN(ui_Button_Record_Start);
    DISABLE_BTN(ui_Button_Record_Stop);

    ui_output_data.button_code = icar_ui::ui_output_data::BUTTON_RECORD_STOP;
    pub_ui_output_data.publish(ui_output_data);
}