#pragma once

#include "ffmpeg_monitor.h"
#if USE_FFMPEG_MONITOR

#include <functional>

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <GL/gl.h>
#include <GL/glext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

class EGLVideo {
public:

  EGLVideo(uint32_t width, uint32_t height,
           const uint32_t *osd_fb=0, uint32_t osd_width=0, int32_t osd_height=0,
           std::function<void(uint32_t, uint32_t)> resize_cb = 0);
  ~EGLVideo();
  bool good() { return m_good; }
  bool poll_events();
  void resize_texture(float x, float y);
  bool draw_frame(EGLint img_attrs[2][13]);

private:
  bool m_good;
  uint32_t m_width;
  uint32_t m_height;
  Display *m_x_display;
  Window m_window;
  Atom m_WM_DELETE_WINDOW;
  std::function<void(uint32_t, uint32_t)> m_resize_cb;
  EGLDisplay m_egl_display;
  bool m_running;
  bool m_texture_size_valid;
  float m_texcoord_x1;
  float m_texcoord_y1;
  GLuint m_prog;
  GLuint m_vs;
  GLuint m_fs;
  GLuint m_textures[3];
  EGLContext m_egl_context;
  EGLSurface m_egl_surface;
  const uint32_t *m_osd_fb;
  uint32_t m_osd_width;
  int32_t m_osd_height;
};

#endif /* USE_FFMPEG_MONITOR */
