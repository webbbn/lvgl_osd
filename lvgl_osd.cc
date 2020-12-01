
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>

#include "telemetry.hh"
#include "lvgl/lvgl.h"
#include "lv_drivers/display/fbdev.h"

#define DISP_BUF_SIZE (LV_VER_RES_MAX * LV_HOR_RES_MAX)
#define CANVAS_WIDTH LV_HOR_RES_MAX
#define CANVAS_HEIGHT LV_VER_RES_MAX

uint32_t custom_tick_get(void);
void lv_ex_canvas_2(void);

static lv_obj_t *label = NULL;
static lv_style_t style;

// Images
LV_IMG_DECLARE(attitude_background);
static lv_obj_t *att_back_img;

/**
 * Create a transparent canvas with Chroma keying and indexed color format (palette).
 */
void osd(void) {

  // Place the image
  att_back_img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(att_back_img, &attitude_background);
  lv_obj_align(att_back_img, NULL, LV_ALIGN_CENTER, 0, -200);
  lv_img_set_pivot(att_back_img, attitude_background.header.w / 2, attitude_background.header.h / 2);

  label = lv_label_create(lv_layer_sys(), NULL);
  lv_label_set_align(label, LV_LABEL_ALIGN_RIGHT);
  lv_obj_set_style_local_bg_opa(label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_COVER);
  lv_obj_set_style_local_bg_color(label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_BLACK);
  lv_obj_set_style_local_text_color(label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
  lv_obj_set_style_local_pad_top(label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, 3);
  lv_obj_set_style_local_pad_bottom(label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, 3);
  lv_obj_set_style_local_pad_left(label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, 3);
  lv_obj_set_style_local_pad_right(label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, 3);
  lv_obj_set_style_local_text_font(label, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_48);
  lv_label_set_text(label, "?");
  lv_obj_align(label, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, 0, 0);

  lv_style_init(&style);
  lv_style_set_radius(&style, LV_STATE_DEFAULT, 5);

  lv_style_set_bg_opa(&style, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_style_set_bg_color(&style, LV_STATE_DEFAULT, LV_COLOR_TRANSP);
  lv_style_set_bg_grad_color(&style, LV_STATE_DEFAULT, LV_COLOR_TRANSP);
  lv_style_set_bg_grad_dir(&style, LV_STATE_DEFAULT, LV_GRAD_DIR_NONE);
  lv_style_set_value_opa(&style, LV_STATE_DEFAULT, LV_OPA_COVER);
  lv_style_set_value_color(&style, LV_STATE_DEFAULT, LV_COLOR_WHITE);
  lv_style_set_text_font(&style, LV_STATE_DEFAULT, &lv_font_montserrat_48);
  lv_obj_add_style(lv_scr_act(), LV_OBJ_PART_MAIN, &style);

  /*Describe the color for the needles*/
  static lv_color_t needle_colors[3];
  needle_colors[0] = LV_COLOR_BLUE;
  needle_colors[1] = LV_COLOR_ORANGE;
  needle_colors[2] = LV_COLOR_PURPLE;
  LV_IMG_DECLARE(img_hand);

  /*Create a gauge*/
  lv_obj_t * gauge1 = lv_gauge_create(lv_scr_act(), NULL);
  lv_gauge_set_needle_count(gauge1, 3, needle_colors);
  lv_obj_set_size(gauge1, 200, 200);
  lv_obj_align(gauge1, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_local_text_font(gauge1, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_16);
  lv_obj_set_style_local_bg_opa(label, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  //lv_obj_add_style(guage1, LV_OBJ_PART_MAIN, &style);
  lv_gauge_set_needle_img(gauge1, &img_hand, 4, 4);

  /*Set the values*/
  lv_gauge_set_value(gauge1, 0, 10);
  lv_gauge_set_value(gauge1, 1, 20);
  lv_gauge_set_value(gauge1, 2, 30);

  /*Create a line meter */
  lv_obj_t * lmeter;
  lmeter = lv_linemeter_create(lv_scr_act(), NULL);
  lv_linemeter_set_range(lmeter, 0, 100);                   /*Set the range*/
  lv_linemeter_set_value(lmeter, 80);                       /*Set the current value*/
  lv_linemeter_set_scale(lmeter, 240, 21);                  /*Set the angle and number of lines*/
  lv_obj_set_size(lmeter, 150, 150);
  lv_obj_align(lmeter, NULL, LV_ALIGN_CENTER, -400, 0);
}

int main(void) {
  fflush(stdout);

  // Create the Telemetry class that controls the telemetry receive threads
  Telemetry telem;
  if (!telem.start("127.0.0.1", 14950, "127.0.0.1", 5800)) {
    fprintf(stderr, "Error starting the telemetry receive threads.");
    fflush(stderr);
  }

  /* LittlevGL init */
  lv_init();

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

  /* Create a Demo */
  osd();

  /* Handle LitlevGL tasks (tickless mode) */
  uint8_t cntr = 0;
  uint8_t cntr2 = 0;
  while(1) {
    lv_task_handler();
    usleep(5000);
    if (++cntr == 20) {
      lv_label_set_text_fmt(label, "Countdown: %d", cntr2);
      lv_obj_align(label, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, 0, 0);
      lv_img_set_angle(att_back_img, cntr2 * 360);
      cntr = 0;
      ++cntr2;
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
