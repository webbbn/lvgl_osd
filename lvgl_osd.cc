
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdio.h>

#include <cmath>

#include "telemetry.hh"
#include "lvgl/lvgl.h"
#if defined(WIN32)
#include "lv_drivers/display/monitor.h"
#include "lv_drivers/indev/mouse.h"
#include "lv_drivers/indev/keyboard.h"
#else
#include "lv_drivers/display/fbdev.h"
#endif

#define DISP_BUF_SIZE (LV_VER_RES_MAX * LV_HOR_RES_MAX)
#define CANVAS_WIDTH LV_HOR_RES_MAX
#define CANVAS_HEIGHT LV_VER_RES_MAX

uint32_t custom_tick_get(void);
void lv_ex_canvas_2(void);
static int tick_thread(void *data);

static lv_indev_t * kb_indev;
static lv_style_t style;
static const char *g_arducopter_mode_strings[] = {
  "Stabilize", // manual airframe angle with manual throttle
  "Acro",      // manual body-frame angular rate with manual throttle
  "Alt Hold",  // manual airframe angle with automatic throttle
  "Auto",      // fully automatic waypoint control using mission commands
  "Guided",    // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
  "Loiter",    // automatic horizontal acceleration with automatic throttle
  "RTL",       // automatic return to launching point
  "Circle",    // automatic circular flight with automatic throttle
  "Land",      // automatic landing with horizontal position control
  "Drift",     // semi-automous position, yaw and throttle control
  "Sport",     // manual earth-frame angular rate control with manual throttle
  "Flip",      // automatically flip the vehicle on the roll axis
  "Autotune",  // automatically tune the vehicle's roll and pitch gains
  "Poshold",   // automatic position hold with manual override, with automatic throttle
  "Brake",     // full-brake using inertial/GPS system, no pilot input
  "Throw",     // throw to launch mode using inertial/GPS system, no pilot input
  "Avoid",     // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
  "Guided",    // guided mode but only accepts attitude and altitude
};
static const char *g_arduplane_mode_strings[] = {
 "Manual",
 "Circle",
 "Stabilize",
 "Training",
 "Acro",
 "Fly By Wire A",
 "Fly By Wire B",
 "Cruise",
 "Autotune",
 "Auto",
 "RTL",
 "Loiter",
 "Takeoff",
 "Avoid ADSB",
 "Guided",
 "Initialixing",
 "QStabalize",
 "QHover",
 "QLoiter",
 "QLand",
 "QRTL",
 "Qautune",
 "QAcro"
};

class Label {
public:

  Label(const std::string &text, uint16_t x, uint16_t y, lv_obj_t *par = 0)
    : m_x(x), m_y(y) {
    lv_obj_t *parrent = par ? par : lv_scr_act();

    m_label = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_align(m_label, LV_LABEL_ALIGN_LEFT);
    lv_label_set_text(m_label, text.c_str());
    lv_obj_align(m_label, NULL, LV_ALIGN_CENTER, x, y);

    lv_style_copy(&m_style, &style);
    //lv_style_set_text_font(&m_style, LV_STATE_DEFAULT, &lv_font_montserrat_48);
    lv_style_set_text_font(&m_style, LV_STATE_DEFAULT, &lv_font_montserrat_24);
    lv_obj_add_style(m_label, LV_LABEL_PART_MAIN, &m_style);
  }

private:
  uint16_t m_x;
  uint16_t m_y;
  lv_style_t m_style;
  lv_obj_t *m_label;
};

// Images
LV_IMG_DECLARE(satellite);
LV_IMG_DECLARE(bat_full);
LV_IMG_DECLARE(bat_0);
LV_IMG_DECLARE(bat_1);
LV_IMG_DECLARE(bat_2);
LV_IMG_DECLARE(bat_3);
LV_IMG_DECLARE(bat_4);
LV_IMG_DECLARE(sig_0);
LV_IMG_DECLARE(sig_1);
LV_IMG_DECLARE(sig_2);
LV_IMG_DECLARE(sig_3);
LV_IMG_DECLARE(sig_4);
LV_IMG_DECLARE(sig_5);
LV_IMG_DECLARE(sig_6);
LV_IMG_DECLARE(north_needle);
LV_IMG_DECLARE(compass);
LV_IMG_DECLARE(home_arrow);

/**
 * Initialize the Hardware Abstraction Layer (HAL) for the Littlev graphics library
 */
static void hal_init(void) {

#if defined(WIN32)
  /* Add a display
   * Use the 'monitor' driver which creates window on PC's monitor to simulate a display*/
  monitor_init();

  static lv_disp_buf_t disp_buf1;
  static lv_color_t buf1_1[LV_HOR_RES_MAX * 120];
  lv_disp_buf_init(&disp_buf1, buf1_1, NULL, LV_HOR_RES_MAX * 120);
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
  disp_drv.buffer = &disp_buf1;
  disp_drv.flush_cb = monitor_flush;
  lv_disp_drv_register(&disp_drv);

  /* Add the mouse (or touchpad) as input device
   * Use the 'mouse' driver which reads the PC's mouse*/
  mouse_init();
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);          /*Basic initialization*/
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = mouse_read;         /*This function will be called periodically (by the library) to get the mouseposition and state*/
  lv_indev_drv_register(&indev_drv);

  /* If the PC keyboard driver is enabled in`lv_drv_conf.h`
   * add this as an input device. It might be used in some examples. */
#if USE_KEYBOARD
  lv_indev_drv_t kb_drv;
  lv_indev_drv_init(&kb_drv);
  kb_drv.type = LV_INDEV_TYPE_KEYPAD;
  kb_drv.read_cb = keyboard_read;
  kb_indev = lv_indev_drv_register(&kb_drv);
#endif

  /* Tick init.
   * You have to call 'lv_tick_inc()' in every milliseconds
   * Create an SDL thread to do this*/
  //SDL_CreateThread(tick_thread, "tick", NULL);

#else

  /* Linux frame buffer device init */
  fbdev_init();

  /* A small buffer for LittlevGL to draw the screen's content */
  static lv_color_t buf[DISP_BUF_SIZE];

  /* Initialize a descriptor for the buffer */
  static lv_disp_buf_t disp_buf;
  lv_disp_buf_init(&disp_buf, buf, NULL, DISP_BUF_SIZE);

  /* Initialize and register a display driver */
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.buffer   = &disp_buf;
  disp_drv.flush_cb = fbdev_flush;
  lv_disp_drv_register(&disp_drv);
#endif
}

int main(int argv, char**argc) {
  fflush(stdout);

  // Create the Telemetry class that controls the telemetry receive threads
  Telemetry telem;
  if (!telem.start("127.0.0.1", 14950, "127.0.0.1", 5800)) {
    fprintf(stderr, "Error starting the telemetry receive threads.\n");
    fflush(stderr);
  }

  /* LittlevGL init */
  lv_init();

  /*Initialize the HAL for LittlevGL*/
  hal_init();

  lv_obj_set_style_local_bg_opa(lv_scr_act(), LV_OBJMASK_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_disp_set_bg_opa(NULL, LV_OPA_TRANSP);

  // Default style properties
  static lv_style_t style;
  lv_style_set_bg_opa(&style, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_style_set_bg_color(&style, LV_STATE_DEFAULT, LV_COLOR_TRANSP);
  lv_style_set_bg_grad_color(&style, LV_STATE_DEFAULT, LV_COLOR_TRANSP);
  lv_style_set_bg_grad_dir(&style, LV_STATE_DEFAULT, LV_GRAD_DIR_NONE);
  lv_style_set_value_opa(&style, LV_STATE_DEFAULT, LV_OPA_COVER);
  lv_style_set_value_color(&style, LV_STATE_DEFAULT, LV_COLOR_WHITE);
  lv_style_set_text_font(&style, LV_STATE_DEFAULT, &lv_font_montserrat_16);

  // Gauge properties
  lv_style_set_bg_opa(&style, LV_GAUGE_PART_MAIN, LV_OPA_COVER);
  lv_style_set_bg_color(&style, LV_GAUGE_PART_MAIN, LV_COLOR_BLACK);
  lv_style_set_bg_grad_color(&style, LV_GAUGE_PART_MAIN, LV_COLOR_BLACK);
  lv_style_set_line_width(&style, LV_GAUGE_PART_MAIN, 2);
  lv_style_set_line_width(&style, LV_GAUGE_PART_MAJOR, 4);
  lv_style_set_line_color(&style, LV_GAUGE_PART_MAIN, LV_COLOR_WHITE);
  lv_style_set_scale_grad_color(&style, LV_GAUGE_PART_MAIN, LV_COLOR_WHITE);
  lv_style_set_scale_end_color(&style, LV_GAUGE_PART_MAIN, LV_COLOR_RED);
  lv_style_set_scale_end_line_width(&style, LV_GAUGE_PART_MAIN, 2);
  lv_style_set_scale_end_line_width(&style, LV_GAUGE_PART_MAJOR, 4);
  lv_style_set_scale_border_width(&style, LV_GAUGE_PART_MAIN, 0);
  lv_style_set_scale_border_width(&style, LV_GAUGE_PART_MAJOR, 2);
  lv_style_set_scale_end_border_width(&style, LV_GAUGE_PART_MAIN, 0);
  lv_style_set_scale_border_width(&style, LV_GAUGE_PART_MAIN, 0);
  lv_style_set_border_width(&style, LV_GAUGE_PART_MAIN, 2);
  lv_style_set_pad_inner(&style, LV_GAUGE_PART_MAIN, 20);
  lv_style_set_pad_top(&style, LV_GAUGE_PART_MAIN, 5);
  lv_style_set_pad_left(&style, LV_GAUGE_PART_MAIN, 5);
  lv_style_set_pad_right(&style, LV_GAUGE_PART_MAIN, 5);
  lv_style_set_text_font(&style, LV_GAUGE_PART_MAJOR, &lv_font_montserrat_14);
  lv_style_set_text_font(&style, LV_GAUGE_PART_MAIN, &lv_font_montserrat_14);

  // Label properties
  lv_style_set_bg_opa(&style, LV_LABEL_PART_MAIN, LV_OPA_TRANSP);
  lv_style_set_bg_color(&style, LV_LABEL_PART_MAIN, LV_COLOR_BLACK);
  lv_style_set_bg_grad_color(&style, LV_LABEL_PART_MAIN, LV_COLOR_WHITE);
  lv_style_set_text_color(&style, LV_LABEL_PART_MAIN, LV_COLOR_WHITE);
  lv_style_set_pad_top(&style, LV_LABEL_PART_MAIN, 3);
  lv_style_set_pad_bottom(&style, LV_LABEL_PART_MAIN, 3);
  lv_style_set_pad_left(&style, LV_LABEL_PART_MAIN, 3);
  lv_style_set_pad_right(&style, LV_LABEL_PART_MAIN, 3);
  lv_style_set_border_width(&style, LV_LABEL_PART_MAIN, 0);

  // Container properties
  lv_style_set_pad_inner(&style, LV_CONT_PART_MAIN, 0);
  lv_style_set_pad_top(&style, LV_CONT_PART_MAIN, 0);
  lv_style_set_pad_bottom(&style, LV_CONT_PART_MAIN, 0);
  lv_style_set_pad_left(&style, LV_CONT_PART_MAIN, 0);
  lv_style_set_pad_right(&style, LV_CONT_PART_MAIN, 0);
  lv_style_set_border_width(&style, LV_CONT_PART_MAIN, 0);

  // Image properties
  lv_style_set_pad_inner(&style, LV_IMG_PART_MAIN, 0);
  lv_style_set_pad_top(&style, LV_IMG_PART_MAIN, 0);
  lv_style_set_pad_bottom(&style, LV_IMG_PART_MAIN, 0);
  lv_style_set_pad_left(&style, LV_IMG_PART_MAIN, 0);
  lv_style_set_pad_right(&style, LV_IMG_PART_MAIN, 0);

  lv_obj_add_style(lv_scr_act(), LV_OBJ_PART_MAIN, &style);

  // Copy the style for labels, but increase the font size
  static lv_style_t label_style;
  lv_style_copy(&label_style, &style);
  lv_style_set_text_font(&label_style, LV_STATE_DEFAULT, &lv_font_montserrat_30);

  // Create a small font for units, etc
  static lv_style_t units_style;
  lv_style_copy(&units_style, &label_style);
  lv_style_set_text_font(&units_style, LV_STATE_DEFAULT, &lv_font_montserrat_14);

  /*******************************
   * Create the video stats gague
   *******************************/

  // Create the video stats group
  lv_obj_t *video_group = lv_cont_create(lv_scr_act(), NULL);
  lv_obj_set_auto_realign(video_group, true);
  lv_obj_align_origo(video_group, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_cont_set_fit(video_group, LV_FIT_TIGHT);
  lv_cont_set_layout(video_group, LV_LAYOUT_COLUMN_MID);
  lv_obj_align(video_group, NULL, LV_ALIGN_IN_TOP_MID, -200, 10);
  //lv_obj_add_style(video_group, LV_GAUGE_PART_MAJOR, &style);
  lv_obj_add_style(video_group, LV_GAUGE_PART_MAIN, &style);

  // Create a custom style for the video gague
  lv_style_t video_style;
  lv_style_copy(&video_style, &style);
  lv_style_set_pad_inner(&video_style, LV_GAUGE_PART_MAIN, 10);

  // Describe the color for the needles
  lv_color_t video_needle_colors[2];
  video_needle_colors[0] = LV_COLOR_YELLOW;
  video_needle_colors[1] = LV_COLOR_RED;

  // Create the video guage
  lv_obj_t *video_gauge = lv_gauge_create(video_group, NULL);
  lv_gauge_set_needle_count(video_gauge, 2, video_needle_colors);
  lv_obj_set_size(video_gauge, 150, 150);
  lv_obj_align(video_gauge, video_group, LV_ALIGN_CENTER, 0, 0);
  lv_gauge_set_range(video_gauge, 0, 20);
  lv_gauge_set_critical_value(video_gauge, 15);
  lv_obj_add_style(video_gauge, LV_GAUGE_PART_MAJOR, &video_style);
  lv_obj_add_style(video_gauge, LV_GAUGE_PART_MAIN, &video_style);

  // Add the bitrate down group
  lv_obj_t *bitrate_down_group = lv_cont_create(video_group, NULL);
  lv_obj_set_auto_realign(bitrate_down_group, true);
  lv_cont_set_fit(bitrate_down_group, LV_FIT_TIGHT);
  lv_cont_set_layout(bitrate_down_group, LV_LAYOUT_ROW_BOTTOM);
  lv_obj_align(bitrate_down_group, video_group, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
  lv_obj_add_style(bitrate_down_group, LV_LABEL_PART_MAIN, &label_style);

  // Add the bitrate label
  lv_obj_t *rx_bitrate_label = lv_label_create(bitrate_down_group, NULL);
  lv_label_set_align(rx_bitrate_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(rx_bitrate_label, bitrate_down_group, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
  lv_obj_set_auto_realign(rx_bitrate_label, true);

  // Add the Mbps label
  lv_obj_t *rx_mbps_label = lv_label_create(bitrate_down_group, NULL);
  lv_label_set_align(rx_mbps_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(rx_mbps_label, rx_bitrate_label, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
  lv_obj_set_auto_realign(rx_mbps_label, true);
  lv_obj_add_style(rx_mbps_label, LV_LABEL_PART_MAIN, &units_style);
  lv_label_set_text(rx_mbps_label, "Mbps " LV_SYMBOL_DOWN);

  /******************************
   * Create the compass gauge
   ******************************/

  // Create the top-level compass group
  lv_obj_t *compass_group = lv_cont_create(lv_scr_act(), NULL);
  lv_obj_set_auto_realign(compass_group, true);
  lv_obj_align_origo(compass_group, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_cont_set_fit(compass_group, LV_FIT_TIGHT);
  lv_cont_set_layout(compass_group, LV_LAYOUT_COLUMN_LEFT);
  lv_obj_align(compass_group, NULL, LV_ALIGN_IN_TOP_MID, 0, -90);
  lv_obj_add_style(compass_group, LV_GAUGE_PART_MAIN, &style);

  // Create the compass image
  lv_obj_t *compass_img = lv_img_create(compass_group, NULL);
  lv_img_set_src(compass_img, &compass);
  lv_img_set_zoom(compass_img, 128);
  lv_obj_align(compass_img, compass_group, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_auto_realign(compass_img, true);
  lv_obj_add_style(compass_img, LV_IMG_PART_MAIN, &style);

  // Add the rssi label
  lv_obj_t *orientation_label = lv_label_create(compass_img, NULL);
  lv_label_set_align(orientation_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(orientation_label, compass_img, LV_ALIGN_CENTER, 0, 40);
  lv_obj_set_auto_realign(orientation_label, true);
  lv_obj_add_style(orientation_label, LV_LABEL_PART_MAIN, &label_style);

  // Create the home arrow image
  lv_obj_t *home_img = lv_img_create(compass_img, NULL);
  lv_img_set_src(home_img, &home_arrow);
  lv_img_set_zoom(home_img, 128);
  lv_obj_align(home_img, compass_img, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_auto_realign(home_img, true);
  lv_obj_add_style(home_img, LV_IMG_PART_MAIN, &style);

  /*******************************
   * Create the rssi gague
   *******************************/

  // Create a custom style for the rssi gauge
  lv_style_t rssi_style;
  lv_style_copy(&rssi_style, &style);
  lv_style_set_pad_inner(&rssi_style, LV_GAUGE_PART_MAIN, 10);
  lv_style_set_line_color(&rssi_style, LV_GAUGE_PART_MAIN, LV_COLOR_RED);
  lv_style_set_line_color(&rssi_style, LV_GAUGE_PART_MAJOR, LV_COLOR_RED);
  lv_style_set_scale_grad_color(&rssi_style, LV_GAUGE_PART_MAIN, LV_COLOR_RED);
  lv_style_set_scale_grad_color(&rssi_style, LV_GAUGE_PART_MAJOR, LV_COLOR_RED);
  lv_style_set_scale_end_color(&rssi_style, LV_GAUGE_PART_MAIN, LV_COLOR_WHITE);
  lv_style_set_scale_end_color(&rssi_style, LV_GAUGE_PART_MAJOR, LV_COLOR_WHITE);
  lv_style_set_line_width(&rssi_style, LV_GAUGE_PART_MAIN, 2);
  lv_style_set_line_width(&rssi_style, LV_GAUGE_PART_MAJOR, 4);

  // Create the rssi stats group
  lv_obj_t *rssi_group = lv_cont_create(lv_scr_act(), NULL);
  lv_obj_set_auto_realign(rssi_group, true);
  lv_obj_align_origo(rssi_group, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_cont_set_fit(rssi_group, LV_FIT_TIGHT);
  lv_cont_set_layout(rssi_group, LV_LAYOUT_COLUMN_LEFT);
  lv_obj_align(rssi_group, NULL, LV_ALIGN_IN_TOP_MID, 200, 10);
  //lv_obj_add_style(rssi_group, LV_GAUGE_PART_MAJOR, &rssi_style);
  lv_obj_add_style(rssi_group, LV_GAUGE_PART_MAIN, &rssi_style);

  // Describe the color for the needles
  lv_color_t rssi_needle_colors[2];
  rssi_needle_colors[0] = LV_COLOR_YELLOW;
  rssi_needle_colors[1] = LV_COLOR_RED;

  // Create the rssi guage
  lv_obj_t *rssi_gauge = lv_gauge_create(rssi_group, NULL);
  lv_gauge_set_needle_count(rssi_gauge, 2, rssi_needle_colors);
  lv_obj_set_size(rssi_gauge, 150, 150);
  lv_obj_align(rssi_gauge, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_gauge_set_range(rssi_gauge, -100, 0);
  lv_gauge_set_critical_value(rssi_gauge, -80);
  lv_obj_add_style(rssi_gauge, LV_GAUGE_PART_MAJOR, &rssi_style);
  lv_obj_add_style(rssi_gauge, LV_GAUGE_PART_MAIN, &rssi_style);

  // Add the rssi down group
  lv_obj_t *rssi_down_group = lv_cont_create(rssi_group, NULL);
  lv_obj_set_auto_realign(rssi_down_group, true);
  lv_cont_set_fit(rssi_down_group, LV_FIT_TIGHT);
  lv_cont_set_layout(rssi_down_group, LV_LAYOUT_ROW_BOTTOM);
  lv_obj_align(rssi_down_group, rssi_group, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
  lv_obj_add_style(rssi_down_group, LV_LABEL_PART_MAIN, &label_style);

  // Add the rssi label
  lv_obj_t *rssi_down_label = lv_label_create(rssi_down_group, NULL);
  lv_label_set_align(rssi_down_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(rssi_down_label, rssi_down_group, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
  lv_obj_set_auto_realign(rssi_down_label, true);

  // Add the dbm label
  lv_obj_t *rx_dbm_label = lv_label_create(rssi_down_group, NULL);
  lv_label_set_align(rx_dbm_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(rx_dbm_label, rssi_down_label, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
  lv_obj_set_auto_realign(rx_dbm_label, true);
  lv_obj_add_style(rx_dbm_label, LV_LABEL_PART_MAIN, &units_style);
  lv_label_set_text(rx_dbm_label, "dBm" LV_SYMBOL_DOWN);

  /***********************
   * Create the GPS object
   ***********************/

  // Create the GPS group
  lv_obj_t *gps_group = lv_cont_create(lv_scr_act(), NULL);
  lv_obj_set_auto_realign(gps_group, true);
  lv_obj_align_origo(gps_group, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_cont_set_fit(gps_group, LV_FIT_TIGHT);
  lv_cont_set_layout(gps_group, LV_LAYOUT_COLUMN_LEFT);
  lv_obj_align(gps_group, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -50, -30);
  lv_obj_add_style(gps_group, LV_LABEL_PART_MAIN, &label_style);

  // Add the satellite status group
  lv_obj_t *gps_stats_group = lv_cont_create(gps_group, NULL);
  lv_obj_set_auto_realign(gps_stats_group, true);
  lv_obj_align_origo(gps_stats_group, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_cont_set_fit(gps_stats_group, LV_FIT_TIGHT);
  lv_cont_set_layout(gps_stats_group, LV_LAYOUT_ROW_BOTTOM);
  lv_obj_align(gps_stats_group, gps_group, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
  lv_obj_add_style(gps_stats_group, LV_LABEL_PART_MAIN, &label_style);

  // Add the satellite icon
  lv_obj_t *satellite_img = lv_img_create(gps_stats_group, NULL);
  lv_img_set_src(satellite_img, &satellite);
  lv_img_set_zoom(satellite_img, 100);
  lv_obj_align(satellite_img, gps_stats_group, LV_ALIGN_IN_TOP_LEFT, 0, 0);
  lv_obj_set_auto_realign(satellite_img, true);
  lv_obj_add_style(satellite_img, LV_IMG_PART_MAIN, &style);

  // Add the satellite status labels
  lv_obj_t *sats_label = lv_label_create(gps_stats_group, NULL);
  lv_label_set_align(sats_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(sats_label, satellite_img, LV_ALIGN_OUT_TOP_RIGHT, 0, 0);
  lv_obj_set_auto_realign(sats_label, true);
  lv_obj_t *nsats_label = lv_label_create(gps_stats_group, NULL);
  lv_label_set_align(nsats_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(nsats_label, sats_label, LV_ALIGN_OUT_TOP_RIGHT, 0, 0);
  lv_obj_set_auto_realign(nsats_label, true);
  lv_obj_add_style(nsats_label, LV_LABEL_PART_MAIN, &units_style);
  lv_label_set_text(nsats_label, "SATS");
  lv_obj_t *hdop_label = lv_label_create(gps_stats_group, NULL);
  lv_label_set_align(hdop_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(hdop_label, nsats_label, LV_ALIGN_OUT_TOP_RIGHT, 0, 0);
  lv_obj_set_auto_realign(hdop_label, true);
  lv_obj_t *hdopl_label = lv_label_create(gps_stats_group, NULL);
  lv_label_set_align(hdopl_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(hdopl_label, hdop_label, LV_ALIGN_OUT_TOP_RIGHT, 0, 0);
  lv_obj_set_auto_realign(hdopl_label, true);
  lv_obj_add_style(hdopl_label, LV_LABEL_PART_MAIN, &units_style);
  lv_label_set_text(hdopl_label, "HDOP");

  // Add the latitude label
  lv_obj_t *lat_label = lv_label_create(gps_group, NULL);
  lv_label_set_align(lat_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(lat_label, sats_label, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_auto_realign(lat_label, true);

  // Add the longitude label
  lv_obj_t *lon_label = lv_label_create(gps_group, NULL);
  lv_label_set_align(lon_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(lon_label, lat_label, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_auto_realign(lon_label, true);

  /**************************
   * Create the battery group
   **************************/

  // Create the top-level battery group
  lv_obj_t *bat_group = lv_cont_create(lv_scr_act(), NULL);
  lv_obj_set_auto_realign(bat_group, true);
  lv_obj_align_origo(bat_group, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_cont_set_fit(bat_group, LV_FIT_TIGHT);
  lv_cont_set_layout(bat_group, LV_LAYOUT_ROW_MID);
  lv_obj_align(bat_group, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 30, -30);
  lv_obj_add_style(bat_group, LV_LABEL_PART_MAIN, &label_style);

  // Create the vertical battery group
  lv_obj_t *batv_group = lv_cont_create(bat_group, NULL);
  lv_obj_set_auto_realign(batv_group, true);
  lv_obj_align_origo(batv_group, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_cont_set_fit(batv_group, LV_FIT_TIGHT);
  lv_cont_set_layout(batv_group, LV_LAYOUT_COLUMN_RIGHT);
  lv_obj_align(batv_group, gps_group, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
  lv_obj_add_style(batv_group, LV_LABEL_PART_MAIN, &label_style);

  // Create the battery labels
  lv_obj_t *volt_label = lv_label_create(batv_group, NULL);
  lv_label_set_align(volt_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(volt_label, batv_group, LV_ALIGN_IN_TOP_RIGHT, 0, 0);
  lv_obj_set_auto_realign(volt_label, true);
  lv_obj_t *cur_label = lv_label_create(batv_group, NULL);
  lv_label_set_align(cur_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(cur_label, batv_group, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_auto_realign(cur_label, true);

  // Add the battery icon
  lv_obj_t *bat_img = lv_img_create(bat_group, NULL);
  lv_img_set_src(bat_img, &bat_0);
  lv_obj_align(bat_img, volt_label, LV_ALIGN_IN_RIGHT_MID, 100, 100);
  lv_obj_set_auto_realign(bat_img, true);

  /******************************
   * Create the flight mode label
   ******************************/

  // Add the flight mode label
  lv_obj_t *mode_label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_align(mode_label, LV_LABEL_ALIGN_CENTER);
  lv_obj_align(mode_label, NULL, LV_ALIGN_IN_BOTTOM_MID, -30, -30);
  lv_obj_set_auto_realign(mode_label, true);

  // Increase the font size of the mode string
  static lv_style_t mode_style;
  lv_style_copy(&mode_style, &label_style);
  lv_style_set_text_font(&mode_style, LV_STATE_DEFAULT, &lv_font_montserrat_40);
  lv_obj_add_style(mode_label, LV_LABEL_PART_MAIN, &mode_style);

  /* Handle LitlevGL tasks (tickless mode) */
  uint64_t loop_counter = 0;
  uint8_t prev_bat_level = 0;
  while(1) {
#if defined(WIN32)
    lv_tick_inc(5);
    SDL_Delay(5);
#else
    usleep(5000);
#endif
    lv_task_handler();
    ++loop_counter;
    // Update the telemetry every 100 ms
    if ((loop_counter % 20) == 0) {
      bool blink_on = ((loop_counter % 200) > 100);

      // Get the geo coordinates from the telemetry
      float latitude = 0;
      float longitude = 0;
      telem.get_value("latitude", latitude);
      telem.get_value("longitude", longitude);
      float deg = fabs(std::max(std::min(latitude, 90.0F), -90.0F));
      int32_t deg_int = static_cast<int32_t>(fabs(deg));
      float min = (deg - static_cast<float>(deg_int)) * 60.0;
      int32_t min_int = static_cast<int32_t>(fabs(min));
      float sec = fabs((min - static_cast<float>(min_int)) * 60.0);
      int8_t geo_x = -10;
      char NS = (latitude < 0) ? 'S' : 'N';
      lv_label_set_text_fmt(lat_label, "%3d %2d %5.1f %c", deg_int, min_int, sec, NS);
      deg = fabs(std::max(std::min(longitude, 180.0F), -180.0F));
      deg_int = static_cast<int32_t>(deg);
      min = (deg - static_cast<float>(deg_int)) * 60.0;
      min_int = static_cast<int32_t>(fabs(min));
      sec = fabs((min - static_cast<float>(min_int)) * 60.0);
      char EW = ((longitude < 0) ? 'W' : 'E');
      lv_label_set_text_fmt(lon_label, "%3d %2d %5.1f %c", deg_int, min_int, sec, EW);

      // Set the battery status text
      float remain = 0;
      uint8_t bat_level = 0;
      if (telem.get_value("battery_remaining", remain)) {
        bat_level = floor(remain / 20);
      }
      if (bat_level != prev_bat_level) {
        switch (bat_level) {
          case 0:
            lv_img_set_src(bat_img, &bat_0);
            break;
          case 1:
            lv_img_set_src(bat_img, &bat_1);
            break;
          case 2:
            lv_img_set_src(bat_img, &bat_2);
            lv_obj_set_hidden(bat_img, false);
            break;
          case 3:
            lv_img_set_src(bat_img, &bat_3);
            lv_obj_set_hidden(bat_img, false);
            break;
          default:
            lv_img_set_src(bat_img, &bat_4);
            lv_obj_set_hidden(bat_img, false);
            break;
        }
        prev_bat_level = bat_level;
      }
      if (bat_level < 2) {
        lv_obj_set_hidden(bat_img, blink_on);
      }
      float voltage = 11.9;
      telem.get_value("voltage_battery", voltage);
      lv_label_set_text_fmt(volt_label, "%4.1f V ", voltage);
      float current = 10.2;
      telem.get_value("current_battery", current);
      lv_label_set_text_fmt(cur_label, "%4.1f A ", current);

      // Set the mode text.
      float mode_val = 0;
      if (!telem.get_value("mode", mode_val)) {
        mode_val = 0;
      }
      uint32_t mode = static_cast<uint32_t>(mode_val);
      lv_label_set_text(mode_label, g_arducopter_mode_strings[mode]);

      // Set the GPS stats
      float sats_vis = 0;
      float hdop = 0;
      telem.get_value("gps_num_sats", sats_vis);
      telem.get_value("gps_HDOP", hdop);
      // sats_vis < 8  sats_vis < 6   hdop > 15  hdop > 9
      lv_label_set_text_fmt(sats_label, "%2d", int(sats_vis + 0.5));
      lv_label_set_text_fmt(hdop_label, "%5.1f", hdop);
      uint8_t gps_error_level = 0;
      if (sats_vis < 5) {
        lv_obj_set_style_local_text_color(sats_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
                                          LV_COLOR_RED);
        lv_obj_set_style_local_text_color(nsats_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
                                          LV_COLOR_RED);
        gps_error_level = 2;
      } else if (sats_vis < 8) {
        lv_obj_set_style_local_text_color(sats_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
                                          LV_COLOR_YELLOW);
        lv_obj_set_style_local_text_color(nsats_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
                                          LV_COLOR_YELLOW);
        gps_error_level = 1;
      } else {
        lv_obj_set_style_local_text_color(sats_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
                                          LV_COLOR_WHITE);
        lv_obj_set_style_local_text_color(nsats_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
                                          LV_COLOR_WHITE);
      }
      if (hdop > 15) {
        lv_obj_set_style_local_text_color(hdop_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
                                          LV_COLOR_RED);
        lv_obj_set_style_local_text_color(hdopl_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
                                          LV_COLOR_RED);
        gps_error_level = 2;
      } else if (hdop > 9) {
        lv_obj_set_style_local_text_color(hdop_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
                                          LV_COLOR_YELLOW);
        lv_obj_set_style_local_text_color(hdopl_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
                                          LV_COLOR_YELLOW);
        gps_error_level = (gps_error_level == 2) ? 2 : 1;
      } else {
        lv_obj_set_style_local_text_color(hdop_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
                                          LV_COLOR_WHITE);
        lv_obj_set_style_local_text_color(hdopl_label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT,
                                          LV_COLOR_WHITE);
      }
      switch (gps_error_level) {
      case 2:
        if (blink_on) {
          lv_obj_set_style_local_image_recolor_opa(satellite_img, LV_IMG_PART_MAIN,
                                                   LV_STATE_DEFAULT, LV_OPA_COVER);
          lv_obj_set_style_local_image_recolor(satellite_img, LV_IMG_PART_MAIN,
                                               LV_STATE_DEFAULT, LV_COLOR_RED);
        } else {
          lv_obj_set_style_local_image_recolor_opa(satellite_img, LV_IMG_PART_MAIN,
                                                   LV_STATE_DEFAULT, LV_OPA_COVER);
          lv_obj_set_style_local_image_recolor(satellite_img, LV_IMG_PART_MAIN,
                                               LV_STATE_DEFAULT, LV_COLOR_WHITE);
        }
        break;
      case 1:
        lv_obj_set_style_local_image_recolor_opa(satellite_img, LV_IMG_PART_MAIN,
                                                 LV_STATE_DEFAULT, LV_OPA_COVER);
        lv_obj_set_style_local_image_recolor(satellite_img, LV_IMG_PART_MAIN,
                                             LV_STATE_DEFAULT, LV_COLOR_YELLOW);
        break;
      default:
        lv_obj_set_style_local_image_recolor_opa(satellite_img, LV_IMG_PART_MAIN,
                                                 LV_STATE_DEFAULT, LV_OPA_COVER);
        lv_obj_set_style_local_image_recolor(satellite_img, LV_IMG_PART_MAIN,
                                             LV_STATE_DEFAULT, LV_COLOR_GREEN);
        break;
      }

      // Set the downlink stats
      float rx_rssi = 0;
      float rx_video_packet_count = 0;
      float rx_video_rssi = 8347837.5;
      float rx_video_dropped_packets = 10;
      float rx_video_bad_blocks = 99;
      float rx_video_bitrate = 10.2;
      telem.get_value("rx_video_rssi", rx_rssi);
      telem.get_value("rx_video_packet_count", rx_video_packet_count);
      telem.get_value("rx_video_bitrate", rx_video_bitrate);
      telem.get_value("rx_video_dropped_packets", rx_video_dropped_packets);
      telem.get_value("rx_video_bad_blocks", rx_video_bad_blocks);
      lv_label_set_text_fmt(rssi_down_label, "%6.1f", rx_rssi);
      lv_label_set_text_fmt(rx_bitrate_label, "%4.1f",
                            rx_video_bitrate * 1e-6);
      lv_gauge_set_value(rssi_gauge, 0, rx_rssi);
      lv_gauge_set_value(video_gauge, 0, int(rint(rx_video_dropped_packets / rx_video_packet_count)));
      lv_gauge_set_value(video_gauge, 1, int(rint(rx_video_bad_blocks / rx_video_packet_count)));

      float heading = 0;
      float home_direction = 90.0;
      telem.get_value("heading", heading);
      telem.get_value("home_direction", home_direction);
      lv_img_set_angle(compass_img, (360.0 - heading) * 10);
      lv_label_set_text_fmt(orientation_label, "%5.1f", heading);
      lv_img_set_angle(home_img, home_direction * 10);
    }
  }

  return 0;
}

/* Set in lv_conf.h as `LV_TICK_CUSTOM_SYS_TIME_EXPR` */
uint32_t custom_tick_get(void) {
  static uint64_t start_ms = 0;
  if(start_ms == 0) {
    struct timeval tv_start;
    gettimeofday(&tv_start, NULL);
    start_ms = (tv_start.tv_sec * 1000000 + tv_start.tv_usec) / 1000;
  }

  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  uint64_t now_ms;
  now_ms = (tv_now.tv_sec * 1000000 + tv_now.tv_usec) / 1000;

  uint32_t time_ms = now_ms - start_ms;
  return time_ms;
}
