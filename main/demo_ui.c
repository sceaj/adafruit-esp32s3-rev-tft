#include "lvgl.h"
#include <stdio.h>

static const char HEADER_TEXT[] = "Adafruit Feather";
static const char TITLE_TEXT[] = "ESP32-S3 TFT Demo";
static const char BATTERY_TEXT[] = "Battery: 3.7V / 50%";
static const char BUTTON_TEXT[] = "Buttons:";
static lv_obj_t *header_label;
static lv_obj_t *title_label;
static lv_obj_t *battery_label;
static lv_obj_t *button_label;
static lv_obj_t *line_red;
static lv_obj_t *line_green;
static lv_obj_t *line_blue;
# static lv_display_rotation_t rotation = LV_DISPLAY_ROTATION_0;


void update_battery_ui(unsigned voltage, unsigned voltage_deci, unsigned capacity, unsigned capacity_deci) {
    char buffer[24];
    sprintf(buffer, "Battery: %d.%dV / %d.%d%%", voltage, voltage_deci, capacity, capacity_deci);
    lv_label_set_text(battery_label, buffer);
}


void update_button_ui(bool b0, bool b1, bool b2) {
    char buffer[24];
    sprintf(buffer, "Buttons: %s  %s  %s", b0 ? "B0" : "  ", b1 ? "B1" : "  ", b2 ? "B2" : "  ");
    lv_label_set_text(button_label, buffer);
}

void demo_ui(lv_display_t *disp)
{

    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    header_label = lv_label_create(scr);
    lv_obj_set_pos(header_label, 10, 10);
    lv_obj_set_width(header_label, 220);
    lv_obj_set_height(header_label, 24);
    // lv_obj_set_style_text_align(header_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_text_color(header_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(header_label, &lv_font_montserrat_22, 0);
    lv_label_set_text_static(header_label, HEADER_TEXT);

    title_label = lv_label_create(scr);
    lv_obj_set_pos(title_label, 10, 38);
    lv_obj_set_width(title_label, 220);
    lv_obj_set_height(title_label, 22);
    // lv_obj_set_style_text_align(header_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_text_color(title_label, lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_20, 0);
    lv_label_set_text_static(title_label, TITLE_TEXT);

    battery_label = lv_label_create(scr);
    lv_obj_set_pos(battery_label, 10, 62);
    lv_obj_set_width(battery_label, 220);
    lv_obj_set_height(battery_label, 22);
    // lv_obj_set_style_text_align(header_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_text_color(battery_label, lv_color_hex(0xC0C000), 0);
    lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_20, 0);
    lv_label_set_text_static(battery_label, BATTERY_TEXT);

    button_label = lv_label_create(scr);
    lv_obj_set_pos(button_label, 10, 86);
    lv_obj_set_width(button_label, 220);
    lv_obj_set_height(button_label, 22);
    lv_obj_set_style_text_color(button_label, lv_color_hex(0x808080), 0);
    lv_obj_set_style_text_font(button_label, &lv_font_montserrat_20, 0);
    lv_label_set_text_static(button_label, BUTTON_TEXT);

    /*Create an array for the points of the line*/
	static lv_point_precise_t line_red_points[5];
	line_red_points[0].x = 10;
	line_red_points[0].y = 111;
    line_red_points[1].x = 230;
    line_red_points[1].y = 111;
    line_red_points[2].x = 230;
    line_red_points[2].y = 119;
    line_red_points[3].x = 10;
    line_red_points[3].y = 119;
    line_red_points[4].x = 10;
    line_red_points[4].y = 111;

	line_red = lv_line_create(scr);
    lv_obj_set_style_line_color(line_red, lv_color_make(0xFF, 0x00, 0x00), 0);
    lv_obj_set_style_line_width(line_red, 2, 0);
	lv_line_set_points(line_red, line_red_points, 2);     /*Set the points*/

	static lv_point_precise_t line_green_points[5];
	line_green_points[0].x = 10;
	line_green_points[0].y = 119;
    line_green_points[1].x = 230;
    line_green_points[1].y = 119;
    line_green_points[2].x = 230;
    line_green_points[2].y = 127;
    line_green_points[3].x = 10;
    line_green_points[3].y = 127;
    line_green_points[4].x = 10;
    line_green_points[4].y = 119;

	line_green = lv_line_create(scr);
    lv_obj_set_style_line_color(line_green, lv_color_make(0x00, 0xA0, 0x00), 0);
    lv_obj_set_style_line_width(line_green, 2, 0);
	lv_line_set_points(line_green, line_green_points, 2);     /*Set the points*/

	static lv_point_precise_t line_blue_points[5];
	line_blue_points[0].x = 10;
	line_blue_points[0].y = 127;
    line_blue_points[1].x = 230;
    line_blue_points[1].y = 127;
    line_blue_points[2].x = 230;
    line_blue_points[2].y = 135;
    line_blue_points[3].x = 10;
    line_blue_points[3].y = 135;
    line_blue_points[4].x = 10;
    line_blue_points[4].y = 127;

	line_blue = lv_line_create(scr);
    lv_obj_set_style_line_color(line_blue, lv_color_make(0x00, 0x00, 0xFF), 0);
    lv_obj_set_style_line_width(line_blue, 2, 0);
	lv_line_set_points(line_blue, line_blue_points, 2);     /*Set the points*/
}
