
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

/**
 * Create a transparent canvas with Chroma keying and indexed color format (palette).
 */
void osd(void) {

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
}

int main(void) {
  printf("Running\n");
  fflush(stdout);

  // Create the Telemetry class that controls the telemetry receive threads
  Telemetry telem;
  printf("Telem created\n");
  fflush(stdout);
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
