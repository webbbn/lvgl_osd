/**
 * @file mpv_monitor.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "mpv_monitor.h"
#if USE_MPV_MONITOR

#ifndef MONITOR_SDL_INCLUDE_PATH
#  define MONITOR_SDL_INCLUDE_PATH <SDL2/SDL.h>
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include MONITOR_SDL_INCLUDE_PATH
#ifdef USE_MPV
#include <GL/gl.h>
#include <mpv/client.h>
#include <mpv/render_gl.h>
#endif

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
  SDL_Window * window;
  SDL_Renderer * renderer;
  SDL_Texture * texture;
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
static int tick_thread(void *data);
static void window_create(monitor_t * m);
static void window_update();
static void redraw();
int quit_filter(void * userdata, SDL_Event * event);
static void monitor_sdl_clean_up(void);
static void monitor_sdl_init(void);
static void sdl_event_handler(lv_task_t * t);
static void monitor_sdl_refr(lv_task_t * t);
static void *get_proc_address_mpv(void *fn_ctx, const char *name);
static void on_mpv_events(void *ctx);
static void on_mpv_render_update(void *ctx);

/***********************
 *   GLOBAL PROTOTYPES
 ***********************/

/**********************
 *  STATIC VARIABLES
 **********************/
monitor_t monitor;

static volatile bool sdl_inited = false;
static volatile bool sdl_quit_qry = false;

#ifdef USE_MPV
static mpv_handle *mpv;
static mpv_render_context *mpv_gl;
static Uint32 wakeup_on_mpv_render_update;
static Uint32 wakeup_on_mpv_events;
static bool redraw_video_frame = false;
#endif
static SDL_GLContext glcontext;

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
  monitor_sdl_init();
  lv_task_create(sdl_event_handler, 10, LV_TASK_PRIO_HIGH, NULL);

  /*Create a display buffer*/
  static lv_disp_buf_t disp_buf1;
  static lv_color_t buf1_1[LV_HOR_RES_MAX * 120];
  lv_disp_buf_init(&disp_buf1, buf1_1, NULL, LV_HOR_RES_MAX * 120);

  /*Create a display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv); /*Basic initialization*/
  disp_drv.buffer = &disp_buf1;
  disp_drv.flush_cb = monitor_flush;
  lv_disp_drv_register(&disp_drv);

  /* Tick init.
   * You have to call 'lv_tick_inc()' in periodically to inform LittelvGL about
   * how much time were elapsed Create an SDL thread to do this*/
  SDL_CreateThread(tick_thread, "tick", NULL);

#ifdef USE_MPV
  const char *cmd[] = {"loadfile", url, NULL};
  mpv_set_option_string(mpv, "gpu-context", "drm");
  mpv_set_option_string(mpv, "vd-lavc-dr", "yes");
  mpv_set_option_string(mpv, "terminal", "yes");
  mpv_set_option_string(mpv, "untimed", "yes");
  mpv_set_option_string(mpv, "no-cache", "yes");
  mpv_set_option_string(mpv, "video-sync", "audio");
  mpv_set_option_string(mpv, "video-latency-hacks", "yes");
  mpv_set_option_string(mpv, "stream-buffer-size", "4k");
  mpv_set_option_string(mpv, "no-correct-pts", "yes");
  mpv_set_option_string(mpv, "fps", "60");
    
  mpv_set_option_string(mpv, "audio-buffer", "0");
  mpv_set_option_string(mpv, "vd-lavc-threads", "1");
  mpv_set_option_string(mpv, "demuxer-lavf-analyzeduration", "0.1");
  mpv_set_option_string(mpv, "demuxer-lavf-probe-info", "nostreams");
  mpv_set_option_string(mpv, "demuxer-lavf-o-add", "fflags=+nobuffer");
  mpv_command_async(mpv, 0, cmd);
#endif
}

/**
 * Flush a buffer to the marked area
 * @param drv pointer to driver where this function belongs
 * @param area an area where to copy `color_p`
 * @param color_p an array of pixel to copy to the `area` part of the screen
 */
void monitor_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
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
  if(lv_disp_flush_is_last(disp_drv)) {
    monitor_sdl_refr(NULL);
  }

  /*IMPORTANT! It must be called to tell the system the flush is ready*/
  lv_disp_flush_ready(disp_drv);
#endif
}

void mpv_play_video(const char *url) {
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * A task to measure the elapsed time for LVGL
 * @param data unused
 * @return never return
 */
static int tick_thread(void *data) {
  (void)data;

  while (1) {
    SDL_Delay(5);   /*Sleep for 5 millisecond*/
    lv_tick_inc(5); /*Tell LittelvGL that 5 milliseconds were elapsed*/
  }

  return 0;
}

/**
 * Print the memory usage periodically
 * @param param
 */
static void memory_monitor(lv_task_t *param) {
  (void)param; /*Unused*/

  lv_mem_monitor_t mon;
  lv_mem_monitor(&mon);
  printf("used: %6d (%3d %%), frag: %3d %%, biggest free: %6d\n",
         (int)mon.total_size - mon.free_size, mon.used_pct, mon.frag_pct,
         (int)mon.free_biggest_size);
}

/**
 * SDL main thread. All SDL related task have to be handled here!
 * It initializes SDL, handles drawing and the mouse.
 */

static void sdl_event_handler(lv_task_t * t) {
  (void)t;

  /*Refresh handling*/
  SDL_Event event;
  while(SDL_PollEvent(&event)) {
#if USE_MOUSE != 0
    mouse_handler(&event);
#endif

#if USE_MOUSEWHEEL != 0
    mousewheel_handler(&event);
#endif

#if USE_KEYBOARD
    keyboard_handler(&event);
#endif
    if((&event)->type == SDL_WINDOWEVENT) {
      switch((&event)->window.event) {
#if SDL_VERSION_ATLEAST(2, 0, 5)
      case SDL_WINDOWEVENT_TAKE_FOCUS:
#endif
      case SDL_WINDOWEVENT_EXPOSED:
        window_update();
        break;
      default:
        break;
      }
    }

#ifdef USE_MPV
    // Happens when there is new work for the render thread (such as
    // rendering a new video frame or redrawing it).
    if (event.type == wakeup_on_mpv_render_update) {
      uint64_t flags = mpv_render_context_update(mpv_gl);
      if (flags & MPV_RENDER_UPDATE_FRAME) {
        redraw_video_frame = true;
      }
    }
    // Happens when at least 1 new event is in the mpv event queue.
    if (event.type == wakeup_on_mpv_events) {
      // Handle all remaining mpv events.
      while (1) {
        mpv_event *mp_event = mpv_wait_event(mpv, 0);
        if (mp_event->event_id == MPV_EVENT_NONE) {
          break;
        }
      }
    }
    if (redraw_video_frame) {
      redraw();
      redraw_video_frame = false;
    }
#endif
  }

  /*Run until quit event not arrives*/
  if(sdl_quit_qry) {
    monitor_sdl_clean_up();
    exit(0);
  }
}

/**
 * SDL main thread. All SDL related task have to be handled here!
 * It initializes SDL, handles drawing and the mouse.
 */

static void monitor_sdl_refr(lv_task_t * t)
{
  (void)t;

  /*Refresh handling*/
  if(monitor.sdl_refr_qry != false) {
    monitor.sdl_refr_qry = false;
    window_update(&monitor);
  }
}

int quit_filter(void * userdata, SDL_Event * event)
{
  (void)userdata;

  if(event->type == SDL_WINDOWEVENT) {
    if(event->window.event == SDL_WINDOWEVENT_CLOSE) {
      sdl_quit_qry = true;
    }
  }
  else if(event->type == SDL_QUIT) {
    sdl_quit_qry = true;
  }

  return 1;
}

static void monitor_sdl_clean_up(void)
{
  SDL_DestroyTexture(monitor.texture);
  SDL_DestroyRenderer(monitor.renderer);
  SDL_DestroyWindow(monitor.window);

  SDL_Quit();
}

static void monitor_sdl_init(void)
{
#ifdef USE_MPV
  // Initialize the mpv interface
  if (!(mpv = mpv_create())) {
    fprintf(stderr, "Error creating the libmpv interface.\n");
  }
  mpv_set_option_string(mpv, "gpu-context", "drm");
  if (mpv_initialize(mpv) < 0) {
    fprintf(stderr, "Error initializing libmpv.\n");
  }

  // Set some mpv options
  mpv_request_log_messages(mpv, "debug");
#endif

  SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "no");

  /*Initialize the SDL*/
  SDL_Init(SDL_INIT_VIDEO);

  SDL_SetEventFilter(quit_filter, NULL);

  window_create(&monitor);

  sdl_inited = true;
}


static void window_create(monitor_t * m)
{
  m->window = SDL_CreateWindow("FPView",
                               SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                               MONITOR_HOR_RES * MONITOR_ZOOM, MONITOR_VER_RES * MONITOR_ZOOM,
                               SDL_WINDOW_OPENGL
#if LV_FULLSCREEN
                               | SDL_WINDOW_FULLSCREEN_DESKTOP
#endif
                               );

  /*Initialize the frame buffer to gray (77 is an empirical value) */
#if MONITOR_DOUBLE_BUFFERED
  SDL_UpdateTexture(m->texture, NULL, m->tft_fb_act, MONITOR_HOR_RES * sizeof(uint32_t));
#else
  memset(m->tft_fb, 0x44, MONITOR_HOR_RES * MONITOR_VER_RES * sizeof(uint32_t));
#endif
  glcontext = SDL_GL_CreateContext(m->window);
  if (!glcontext) {
    fprintf(stderr, "failed to create SDL GL context");
  }
#ifdef USE_MPV
  // Initialize the mpv interface
  if (!(mpv = mpv_create())) {
    fprintf(stderr, "Error creating the libmpv interface.\n");
  }

 // Initilize MPV on this window.
  int adv_ctrl_flag = 1;
  mpv_opengl_init_params opengl_init_params =
    {
     get_proc_address_mpv, 0, 0
    };
  mpv_render_param params[] = 
    {
     {MPV_RENDER_PARAM_API_TYPE, MPV_RENDER_API_TYPE_OPENGL},
     {MPV_RENDER_PARAM_OPENGL_INIT_PARAMS, &opengl_init_params},
     // Tell libmpv that you will call mpv_render_context_update() on render
     // context update callbacks, and that you will _not_ block on the core
     // ever (see <libmpv/render.h> "Threading" section for what libmpv
     // functions you can call at all when this is active).
     // In particular, this means you must call e.g. mpv_command_async()
     // instead of mpv_command().
     // If you want to use synchronous calls, either make them on a separate
     // thread, or remove the option below (this will disable features like
     // DR and is not recommended anyway).
     {MPV_RENDER_PARAM_ADVANCED_CONTROL, &adv_ctrl_flag},
     {MPV_RENDER_PARAM_INVALID, 0}
    };

  // This makes mpv use the currently set GL context. It will use the callback
  // (passed via params) to resolve GL builtin functions, as well as extensions.
  if (mpv_render_context_create(&mpv_gl, mpv, params) < 0) {
    fprintf(stderr, "Failed to initialize mpv GL context");
  }

  // We use events for thread-safe notification of the SDL main loop.
  // Generally, the wakeup callbacks (set further below) should do as least
  // work as possible, and merely wake up another thread to do actual work.
  // On SDL, waking up the mainloop is the ideal course of action. SDL's
  // SDL_PushEvent() is thread-safe, so we use that.
  wakeup_on_mpv_render_update = SDL_RegisterEvents(1);
  wakeup_on_mpv_events = SDL_RegisterEvents(1);
  if (wakeup_on_mpv_render_update == (Uint32)-1 ||
      wakeup_on_mpv_events == (Uint32)-1) {
    fprintf(stderr, "Could not register events");
  }

  // When normal mpv events are available.
  mpv_set_wakeup_callback(mpv, on_mpv_events, NULL);

  // When there is a need to call mpv_render_context_update(), which can
  // request a new frame to be rendered.
  // (Separate from the normal event handling mechanism for the sake of
  //  users which run OpenGL on a different thread.)
  mpv_render_context_set_update_callback(mpv_gl, on_mpv_render_update, NULL);
#endif
  m->renderer = SDL_CreateRenderer(m->window, -1,
                                   SDL_RENDERER_ACCELERATED | SDL_RENDERER_TARGETTEXTURE);
  m->texture = SDL_CreateTexture(m->renderer,
                                 SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_TARGET,
                                 MONITOR_HOR_RES, MONITOR_VER_RES);
  SDL_SetTextureBlendMode(m->texture, SDL_BLENDMODE_BLEND);
  SDL_SetRenderTarget(m->renderer, NULL);
  SDL_SetRenderDrawBlendMode(m->renderer, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(m->renderer, 0, 0, 0, 0);

  m->sdl_refr_qry = true;
}

static void window_update() {
#if MONITOR_DOUBLE_BUFFERED == 0
  SDL_UpdateTexture(monitor.texture, NULL, monitor.tft_fb, MONITOR_HOR_RES * sizeof(uint32_t));
#else
  if(m->tft_fb_act == NULL) return;
  SDL_UpdateTexture(monitor.texture, NULL, monitor.tft_fb_act, MONITOR_HOR_RES * sizeof(uint32_t));
#endif
#ifndef USE_MPV
  redraw();
#endif
}

static void redraw() {
#ifdef USE_MPV
  int w = LV_HOR_RES_MAX;
  int h = LV_VER_RES_MAX;
  mpv_opengl_fbo fbo = {0, w, h};
  static int flip_y = 1;
  SDL_GetWindowSize(monitor.window, &w, &h);
  mpv_render_param params[] =
    {
     // Specify the default framebuffer (0) as target. This will
     // render onto the entire screen. If you want to show the video
     // in a smaller rectangle or apply fancy transformations, you'll
     // need to render into a separate FBO and draw it manually.
     {MPV_RENDER_PARAM_OPENGL_FBO, &fbo},
     // Flip rendering (needed due to flipped GL coordinate system).
     {MPV_RENDER_PARAM_FLIP_Y, &flip_y},
     {MPV_RENDER_PARAM_INVALID, 0}
    };
  // See render_gl.h on what OpenGL environment mpv expects, and
  // other API details.
  mpv_render_context_render(mpv_gl, params);
  // Re-enable blending, which MPV apparently turns off
#endif
  SDL_RenderCopy(monitor.renderer, monitor.texture, NULL, NULL);
  SDL_GL_SwapWindow(monitor.window);
}

#ifdef USE_MPV
static void *get_proc_address_mpv(void *fn_ctx, const char *name) {
  return SDL_GL_GetProcAddress(name);
}

static void on_mpv_events(void *ctx) {
  SDL_Event event = {.type = wakeup_on_mpv_events};
  SDL_PushEvent(&event);
}

static void on_mpv_render_update(void *ctx) {
  SDL_Event event = {.type = wakeup_on_mpv_render_update};
  SDL_PushEvent(&event);
}
#endif

#endif /*USE_MPV_MONITOR*/
