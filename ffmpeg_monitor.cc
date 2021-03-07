/**
 * @file ffmpeg_monitor.c
 *
 */

/*********************
 *      INCLUDES
 *********************/

#include "ffmpeg_monitor.h"
#if USE_FFMPEG_MONITOR

#include "egl_video.hh"
#include "ffmpeg_decoder.hh"

#include <iostream>
#include <thread>
#include <vector>

/*********************
 *      DEFINES
 *********************/

#ifndef MONITOR_ZOOM
#define MONITOR_ZOOM        1
#endif

#ifndef MONITOR_HOR_RES
#define MONITOR_HOR_RES        LV_HOR_RES
#endif

#ifndef MONITOR_VER_RES
#define MONITOR_VER_RES        LV_VER_RES
#endif

/**********************
 *      TYPEDEFS
 **********************/

typedef struct {
  std::shared_ptr<std::thread> tick_thread;
  std::shared_ptr<std::thread> decode_thread;
  volatile bool sdl_refr_qry;
#if MONITOR_DOUBLE_BUFFERED
  uint32_t * tft_fb_act;
#else
  uint32_t tft_fb[LV_HOR_RES_MAX * LV_VER_RES_MAX];
#endif
} monitor_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/

/***********************
 *   GLOBAL PROTOTYPES
 ***********************/

/**********************
 *  STATIC VARIABLES
 **********************/
static monitor_t monitor;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize the monitor
 */
void monitor_init(const char *url) {

  // Create a display buffer
  static lv_disp_buf_t disp_buf1;
  static lv_color_t buf1_1[LV_HOR_RES_MAX * 120];
  lv_disp_buf_init(&disp_buf1, buf1_1, NULL, LV_HOR_RES_MAX * 120);

  // Create a display
  lv_disp_drv_t disp_drv;
  // Basic initialization
  lv_disp_drv_init(&disp_drv);
  disp_drv.buffer = &disp_buf1;
  disp_drv.flush_cb = monitor_flush;
  lv_disp_drv_register(&disp_drv);

  // video display loop
  monitor.decode_thread = std::make_shared<std::thread>
    ([url] () {
       FFMPEGDecoder decoder(url);
       EGLVideo win(decoder.width(), decoder.height(),
                    monitor.tft_fb, MONITOR_HOR_RES, MONITOR_VER_RES);
       decoder.decode(win);
     });

  // The LVGL tick thread
  monitor.tick_thread = std::make_shared<std::thread>
    ([] () {
       while (1) {
         // Sleep for 5ms, and tell LittelvGL that 5ms were elapsed
         std::this_thread::sleep_for(std::chrono::milliseconds(5));
         lv_tick_inc(5);
       }
     });
}

/**
 * Flush a buffer to the marked area
 * @param drv pointer to driver where this function belongs
 * @param area an area where to copy `color_p`
 * @param color_p an array of pixel to copy to the `area` part of the screen
 */
void monitor_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
  lv_coord_t hres = disp_drv->rotated == 0 ? disp_drv->hor_res : disp_drv->ver_res;
  lv_coord_t vres = disp_drv->rotated == 0 ? disp_drv->ver_res : disp_drv->hor_res;

  //    printf("x1:%d,y1:%d,x2:%d,y2:%d\n", area->x1, area->y1, area->x2, area->y2);

  /*Return if the area is out the screen*/
  if(area->x2 < 0 || area->y2 < 0 || area->x1 > hres - 1 || area->y1 > vres - 1) {
    lv_disp_flush_ready(disp_drv);
    return;
  }

#if MONITOR_DOUBLE_BUFFERED
  monitor.tft_fb_act = (uint32_t *)color_p;

  monitor.sdl_refr_qry = true;

  /*IMPORTANT! It must be called to tell the system the flush is ready*/
  lv_disp_flush_ready(disp_drv);
#else

  int32_t y;
#if LV_COLOR_DEPTH != 24 && LV_COLOR_DEPTH != 32    /*32 is valid but support 24 for backward compatibility too*/
  int32_t x;
  for(y = area->y1; y <= area->y2 && y < disp_drv->ver_res; y++) {
    for(x = area->x1; x <= area->x2; x++) {
      monitor.tft_fb[y * disp_drv->hor_res + x] = lv_color_to32(*color_p);
      color_p++;
    }

  }
#else
  uint32_t w = lv_area_get_width(area);
  for(y = area->y1; y <= area->y2 && y < disp_drv->ver_res; y++) {
    memcpy(&monitor.tft_fb[y * MONITOR_HOR_RES + area->x1], color_p, w * sizeof(lv_color_t));
    color_p += w;
  }
#endif

  monitor.sdl_refr_qry = true;

  /* TYPICALLY YOU DO NOT NEED THIS
   * If it was the last part to refresh update the texture of the window.*/
/*
  if(lv_disp_flush_is_last(disp_drv)) {
    monitor_sdl_refr(NULL);
  }
*/

  /*IMPORTANT! It must be called to tell the system the flush is ready*/
  lv_disp_flush_ready(disp_drv);
#endif
}


// exit with a simple error message
void fail(const char *msg) {
  fprintf(stderr, "\nERROR: %s failed\n", msg);
  exit(1);
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif
