
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
/*
LV_IMG_DECLARE(attitude_background);
static lv_obj_t *att_back_img;
LV_IMG_DECLARE(attitude_foreground);
static lv_obj_t *att_fore_img;
LV_IMG_DECLARE(attitude_ground);
static lv_obj_t *att_ground_img;
LV_IMG_DECLARE(attitude_ring);
static lv_obj_t *att_ring_img;
LV_IMG_DECLARE(home_arrow_ring);
static lv_obj_t *home_arrow_img;
LV_IMG_DECLARE(north_arrow_ring);
static lv_obj_t *north_arrow_img;
*/
LV_IMG_DECLARE(satellite);
LV_IMG_DECLARE(bat_full);
LV_IMG_DECLARE(sig_0);
LV_IMG_DECLARE(sig_1);
LV_IMG_DECLARE(sig_2);
LV_IMG_DECLARE(sig_3);
LV_IMG_DECLARE(sig_4);
LV_IMG_DECLARE(sig_5);
LV_IMG_DECLARE(sig_6);

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
    fprintf(stderr, "Error starting the telemetry receive threads.");
    fflush(stderr);
  }

  /* LittlevGL init */
  lv_init();

  /*Initialize the HAL for LittlevGL*/
  hal_init();

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
  lv_style_set_bg_opa(&style, LV_GAUGE_PART_MAIN, 100);
  lv_style_set_bg_color(&style, LV_GAUGE_PART_MAIN, LV_COLOR_BLACK);
  lv_style_set_bg_grad_color(&style, LV_GAUGE_PART_MAIN, LV_COLOR_BLACK);
  lv_style_set_line_color(&style, LV_GAUGE_PART_MAIN, LV_COLOR_WHITE);
  lv_style_set_scale_grad_color(&style, LV_GAUGE_PART_MAIN, LV_COLOR_YELLOW);
  lv_style_set_scale_end_color(&style, LV_GAUGE_PART_MAIN, LV_COLOR_RED);
  lv_style_set_scale_border_width(&style, LV_GAUGE_PART_MAIN, 2);
  lv_style_set_scale_end_border_width(&style, LV_GAUGE_PART_MAIN, 0);
  lv_style_set_scale_border_width(&style, LV_GAUGE_PART_MAIN, 0);
  lv_style_set_border_width(&style, LV_GAUGE_PART_MAIN, 2);
  lv_style_set_pad_inner(&style, LV_GAUGE_PART_MAIN, 15);
  lv_style_set_pad_top(&style, LV_GAUGE_PART_MAIN, 5);
  lv_style_set_pad_left(&style, LV_GAUGE_PART_MAIN, 5);
  lv_style_set_pad_right(&style, LV_GAUGE_PART_MAIN, 5);
  lv_style_set_text_font(&style, LV_GAUGE_PART_MAIN, &lv_font_montserrat_16);

  // Line meter properties
  lv_style_set_bg_opa(&style, LV_LINEMETER_PART_MAIN, 100);
  lv_style_set_bg_color(&style, LV_LINEMETER_PART_MAIN, LV_COLOR_BLACK);
  lv_style_set_bg_grad_color(&style, LV_LINEMETER_PART_MAIN, LV_COLOR_BLACK);
  lv_style_set_line_color(&style, LV_LINEMETER_PART_MAIN, LV_COLOR_WHITE);
  lv_style_set_scale_grad_color(&style, LV_LINEMETER_PART_MAIN, LV_COLOR_YELLOW);
  lv_style_set_scale_end_color(&style, LV_LINEMETER_PART_MAIN, LV_COLOR_RED);
  lv_style_set_scale_border_width(&style, LV_LINEMETER_PART_MAIN, 2);
  lv_style_set_scale_end_border_width(&style, LV_LINEMETER_PART_MAIN, 0);
  lv_style_set_scale_border_width(&style, LV_LINEMETER_PART_MAIN, 0);
  lv_style_set_border_width(&style, LV_LINEMETER_PART_MAIN, 2);
  lv_style_set_pad_inner(&style, LV_LINEMETER_PART_MAIN, 15);
  lv_style_set_pad_top(&style, LV_LINEMETER_PART_MAIN, 5);
  lv_style_set_pad_left(&style, LV_LINEMETER_PART_MAIN, 5);
  lv_style_set_pad_right(&style, LV_LINEMETER_PART_MAIN, 5);
  lv_style_set_text_font(&style, LV_LINEMETER_PART_MAIN, &lv_font_montserrat_16);

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


  /*********************************
   * Create the signal quality group
   *********************************/

  // Create the signal strength down group
  lv_obj_t *sigdown_group = lv_cont_create(lv_scr_act(), NULL);
  lv_obj_set_auto_realign(sigdown_group, true);
  lv_obj_align_origo(sigdown_group, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_cont_set_fit(sigdown_group, LV_FIT_TIGHT);
  lv_cont_set_layout(sigdown_group, LV_LAYOUT_COLUMN_LEFT);
  lv_obj_align(sigdown_group, NULL, LV_ALIGN_IN_TOP_LEFT, 50, 30);
  lv_obj_add_style(sigdown_group, LV_LABEL_PART_MAIN, &label_style);

  // Add the rssi down group
  lv_obj_t *rssi_down_group = lv_cont_create(sigdown_group, NULL);
  lv_obj_set_auto_realign(rssi_down_group, true);
  lv_obj_align_origo(rssi_down_group, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_cont_set_fit(rssi_down_group, LV_FIT_TIGHT);
  lv_cont_set_layout(rssi_down_group, LV_LAYOUT_ROW_BOTTOM);
  lv_obj_align(rssi_down_group, sigdown_group, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
  lv_obj_add_style(rssi_down_group, LV_LABEL_PART_MAIN, &label_style);

  // Add the rssi down icon
  lv_obj_t *rssi_down_img = lv_img_create(rssi_down_group, NULL);
  lv_img_set_src(rssi_down_img, &sig_0);
  lv_obj_align(rssi_down_img, rssi_down_group, LV_ALIGN_IN_TOP_LEFT, 0, 0);
  lv_obj_set_auto_realign(rssi_down_img, true);
  lv_obj_add_style(rssi_down_img, LV_IMG_PART_MAIN, &style);

  // Add the rssi down value label
  lv_obj_t *rssi_down_label = lv_label_create(rssi_down_group, NULL);
  lv_label_set_align(rssi_down_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(rssi_down_label, rssi_down_group, LV_ALIGN_OUT_TOP_RIGHT, 0, 0);
  lv_obj_set_auto_realign(rssi_down_label, true);
  lv_obj_t *dbm_down_label = lv_label_create(rssi_down_group, NULL);
  lv_label_set_align(dbm_down_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(dbm_down_label, rssi_down_label, LV_ALIGN_OUT_TOP_RIGHT, 0, 0);
  lv_obj_set_auto_realign(dbm_down_label, true);
  lv_obj_add_style(dbm_down_label, LV_LABEL_PART_MAIN, &units_style);
  lv_label_set_text(dbm_down_label, "dbm " LV_SYMBOL_DOWN);

  // Add the bitrate label
  lv_obj_t *rx_bitrate_label = lv_label_create(sigdown_group, NULL);
  lv_label_set_align(rx_bitrate_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(rx_bitrate_label, sigdown_group, LV_ALIGN_OUT_TOP_RIGHT, 0, 0);
  lv_obj_set_auto_realign(rx_bitrate_label, true);

  // Add the video stats label
  lv_obj_t *rx_video_stats_label = lv_label_create(sigdown_group, NULL);
  lv_label_set_align(rx_video_stats_label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_align(rx_video_stats_label, sigdown_group, LV_ALIGN_OUT_TOP_RIGHT, 0, 0);
  lv_obj_set_auto_realign(rx_video_stats_label, true);

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

  // Create the battery group
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
  lv_img_set_src(bat_img, &bat_full);
  lv_obj_align(bat_img, volt_label, LV_ALIGN_IN_RIGHT_MID, 100, 100);
  lv_obj_set_auto_realign(bat_img, true);

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
  uint8_t cntr = 0;
  uint8_t cntr2 = 0;
  while(1) {
    lv_task_handler();
#if defined(WIN32)
    lv_tick_inc(5);
    SDL_Delay(5);
#endif
    usleep(5000);
    if (++cntr == 20) {
#if 0
      lv_label_set_text_fmt(label, "Countdown: %d", cntr2);
      lv_obj_align(label, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, 0, 0);
      lv_img_set_angle(att_back_img, cntr2 * 360);
#endif
      cntr = 0;
      ++cntr2;

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
      float remain = 75;
      int temp_bat = 0;
      if (telem.get_value("battery_remaining", remain) && (remain < 30)) {
        temp_bat = 1;
      }
      if (telem.get_value("battery_remaining", remain) && (remain < 15)) {
        temp_bat = 2;
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
      float sats_vis = 10;
      float hdop = 12.3;
      telem.get_value("gps_num_sats", sats_vis);
      telem.get_value("gps_HDOP", hdop);
      // sats_vis < 8  sats_vis < 6   hdop > 15  hdop > 9
      lv_label_set_text_fmt(sats_label, "%2d", int(sats_vis + 0.5));
      lv_label_set_text_fmt(hdop_label, "%5.1f", hdop);

      // Set the downlink stats
      float rx_rssi = 0;
      float rx_video_packet_count = 0;
      float rx_video_bitrate = 8347837.5;
      float rx_video_dropped_packets = 10;
      float rx_video_bad_blocks = 99;
      telem.get_value("rx_video_rssi", rx_rssi);
      telem.get_value("rx_video_packet_count", rx_video_packet_count);
      telem.get_value("rx_video_bitrate", rx_video_bitrate);
      telem.get_value("rx_video_dropped_packets", rx_video_dropped_packets);
      telem.get_value("rx_video_bad_blocks", rx_video_bad_blocks);
      lv_label_set_text_fmt(rssi_down_label, "%6.1f", rx_rssi);
      lv_label_set_text_fmt(rx_bitrate_label, "%4.1f Mbps",
                            rx_video_bitrate * 1e-6);
      lv_label_set_text_fmt(rx_video_stats_label, "%5d/%3d/%2d",
                            rx_video_packet_count, rx_video_dropped_packets, rx_video_bad_blocks);
      if (rx_rssi < -85) {
        lv_img_set_src(rssi_down_img, &sig_0);
      } else if (rx_rssi < -70) {
        lv_img_set_src(rssi_down_img, &sig_1);
      } else if (rx_rssi < -60) {
        lv_img_set_src(rssi_down_img, &sig_2);
      } else if (rx_rssi < -50) {
        lv_img_set_src(rssi_down_img, &sig_3);
      } else if (rx_rssi < -40) {
        lv_img_set_src(rssi_down_img, &sig_4);
      } else if (rx_rssi < -30) {
        lv_img_set_src(rssi_down_img, &sig_5);
      } else if (rx_rssi < -20) {
        lv_img_set_src(rssi_down_img, &sig_6);
      }
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
