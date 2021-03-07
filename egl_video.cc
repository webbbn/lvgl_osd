
#include "egl_video.hh"

#if USE_FFMPEG_MONITOR

// configuration section: switch between the many parts that are implemented
// in two or more possible ways in this program
#define SWAP_INTERVAL    0  // 0 = decode and display as fast as possible
                            // 1 = run at VSync framerate (typically 60 Hz)
                            // 2 = run at half VSync framerate (30 Hz)

// request OpenGL 3.3 for Core Profile
#define CORE_PROFILE_MAJOR_VERSION 3
#define CORE_PROFILE_MINOR_VERSION 3
// request OpenGL 3.0 for Compatibility Profile
#define COMP_PROFILE_MAJOR_VERSION 3
#define COMP_PROFILE_MINOR_VERSION 0

PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR;
PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR;
PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES;
PFNGLGENVERTEXARRAYSPROC glGenVertexArrays;
PFNGLBINDVERTEXARRAYPROC  glBindVertexArray;

EGLVideo::EGLVideo(uint32_t width, uint32_t height,
                   const uint32_t *osd_fb, uint32_t osd_width, int32_t osd_height,
                   std::function<void(uint32_t, uint32_t)> resize_cb) :
  m_good(false), m_width(width), m_height(height), m_resize_cb(resize_cb), m_running(true),
  m_texture_size_valid(false), m_texcoord_x1(1.0f), m_texcoord_y1(1.0f),
  m_osd_fb(osd_fb), m_osd_width(osd_width), m_osd_height(osd_height) {

  // Connect to X11
  m_x_display = XOpenDisplay(NULL);
  if (!m_x_display) {
    m_x_display = 0;
    return; // Fail!
  }

  // Create X11 window
  XSetWindowAttributes xattr;
  xattr.override_redirect = True;
  xattr.border_pixel = 0;
  m_window = XCreateWindow(m_x_display, DefaultRootWindow(m_x_display),
                           0, 0, width, height,
                           0, CopyFromParent, InputOutput, CopyFromParent,
                           CWOverrideRedirect | CWBorderPixel, &xattr);
  if (!m_window) {
    return; // Fail!
  }
  XStoreName(m_x_display, m_window, "LVGL FPV");
  XMapWindow(m_x_display, m_window);
  XSelectInput(m_x_display, m_window, ExposureMask | StructureNotifyMask | KeyPressMask);
  Atom m_WM_DELETE_WINDOW = XInternAtom(m_x_display, "WM_DELETE_WINDOW", True);
  XSetWMProtocols(m_x_display, m_window, &m_WM_DELETE_WINDOW, 1);

  // initialize EGL
  m_egl_display =  eglGetDisplay((EGLNativeDisplayType)m_x_display);
  if (m_egl_display == EGL_NO_DISPLAY) {
    return; // Fail!
  }
  if (!eglInitialize(m_egl_display, NULL, NULL)) {
    return; // Fail!
  }
  if (!eglBindAPI(EGL_OPENGL_API)) {
    return; // Fail!
  }

  // create the OpenGL rendering context using EGL
  EGLint visual_attr[] =
    {
     EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
     EGL_RED_SIZE,        8,
     EGL_GREEN_SIZE,      8,
     EGL_BLUE_SIZE,       8,
     EGL_ALPHA_SIZE,      8,
     EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
     EGL_NONE
    };
  EGLConfig cfg;
  EGLint cfg_count;
  if (!eglChooseConfig(m_egl_display, visual_attr, &cfg, 1, &cfg_count) || (cfg_count < 1)) {
    return; // Fail!
  }
  m_egl_surface = eglCreateWindowSurface(m_egl_display, cfg, m_window, NULL);
  if (m_egl_surface == EGL_NO_SURFACE) {
    return; // Fail!
  }
  EGLint ctx_attr[] = {
                       EGL_CONTEXT_OPENGL_PROFILE_MASK,
                       EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
                       EGL_CONTEXT_MAJOR_VERSION, CORE_PROFILE_MAJOR_VERSION,
                       EGL_CONTEXT_MINOR_VERSION, CORE_PROFILE_MINOR_VERSION,
                       EGL_NONE
  };
  m_egl_context = eglCreateContext(m_egl_display, cfg, EGL_NO_CONTEXT, ctx_attr);
  if (m_egl_context == EGL_NO_CONTEXT) {
    return; // Fail!
  }
  eglMakeCurrent(m_egl_display, m_egl_surface, m_egl_surface, m_egl_context);
  eglSwapInterval(m_egl_display, SWAP_INTERVAL);

  // dump OpenGL configuration (for reference)
  printf("OpenGL vendor:   %s\n", glGetString(GL_VENDOR));
  printf("OpenGL renderer: %s\n", glGetString(GL_RENDERER));
  printf("OpenGL version:  %s\n", glGetString(GL_VERSION));

  // look up required EGL and OpenGL extension functions
#define LOOKUP_FUNCTION(type, func)                     \
  func = (type)eglGetProcAddress(#func);                \
  if (!func) { return; /* Fail! */ }
  LOOKUP_FUNCTION(PFNEGLCREATEIMAGEKHRPROC, eglCreateImageKHR);
  LOOKUP_FUNCTION(PFNEGLDESTROYIMAGEKHRPROC, eglDestroyImageKHR);
  LOOKUP_FUNCTION(PFNGLEGLIMAGETARGETTEXTURE2DOESPROC, glEGLImageTargetTexture2DOES);
  LOOKUP_FUNCTION(PFNGLGENVERTEXARRAYSPROC, glGenVertexArrays);
  LOOKUP_FUNCTION(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray);

  // OpenGL shader setup
#define DECLARE_YUV2RGB_MATRIX_GLSL                     \
  "const mat4 yuv2rgb = mat4(\n"                        \
    "    vec4(  1.1644,  1.1644,  1.1644,  0.0000 ),\n" \
    "    vec4(  0.0000, -0.2132,  2.1124,  0.0000 ),\n" \
    "    vec4(  1.7927, -0.5329,  0.0000,  0.0000 ),\n" \
    "    vec4( -0.9729,  0.3015, -1.1334,  1.0000 ));"
  GLuint vao;                   // OpenGL Core Profile requires
  glGenVertexArrays(1, &vao);   // using VAOs even in trivial cases,
  glBindVertexArray(vao);       // so let's set up a dummy VAO
  const char *vs_src =
    "#version 130"
    "\n" "const vec2 coords[4] = vec2[]( vec2(0.,0.), vec2(1.,0.), vec2(0.,1.), vec2(1.,1.) );"
    "\n" "uniform vec2 uTexCoordScale;"
    "\n" "out vec2 vTexCoord;"
    "\n" "void main() {"
    "\n" "  vec2 c = coords[gl_VertexID];"
    "\n" "  vTexCoord = c * uTexCoordScale;"
    "\n" "  gl_Position = vec4(c * vec2(2.,-2.) + vec2(-1.,1.), 0., 1.);"
    "\n" "}";
  const char *fs_src =
    "#version 130"
    "\n" "in vec2 vTexCoord;"
    "\n" "uniform sampler2D uTexY, uTexC, uTexO;"
    "\n" DECLARE_YUV2RGB_MATRIX_GLSL
    "\n" "out vec4 oColor;"
    "\n" "vec4 ovColor, vColor;"
    "\n" "void main() {"
    "\n" "  ovColor = texture(uTexO, vTexCoord).zyxw;"
    "\n" "  vColor = yuv2rgb * vec4(texture(uTexY, vTexCoord).x, texture(uTexC, vTexCoord).xy, 1);"
    "\n" "  if (ovColor.w > 0) {"
    "\n" "    oColor = ovColor;"
    "\n" "  } else {"
    "\n" "    oColor = vColor;"
    "\n" "  }"
    "\n" "}";
  m_prog = glCreateProgram();
  m_vs = glCreateShader(GL_VERTEX_SHADER);
  m_fs = glCreateShader(GL_FRAGMENT_SHADER);
  if (!m_prog) {
    return; // Fail!
  }
  if (!m_vs || !m_fs) {
    return; // Fail!
  }
  glShaderSource(m_vs, 1, &vs_src, NULL);
  glShaderSource(m_fs, 1, &fs_src, NULL);
  GLint ok;
  while (glGetError()) {}
  glCompileShader(m_vs);  glGetShaderiv(m_vs, GL_COMPILE_STATUS, &ok);
  if (glGetError() || (ok != GL_TRUE)) {
    m_good = false;
    return;
  }
  glCompileShader(m_fs); glGetShaderiv(m_fs, GL_COMPILE_STATUS, &ok);
  if (glGetError() || (ok != GL_TRUE)) {
    m_good = false;
    return;
  }
  glAttachShader(m_prog, m_vs);
  glAttachShader(m_prog, m_fs);
  glLinkProgram(m_prog);
  if (glGetError()) {
    m_good = false;
    return;
  }
  glUseProgram(m_prog);
  glUniform1i(glGetUniformLocation(m_prog, "uTexY"), 0);
  glUniform1i(glGetUniformLocation(m_prog, "uTexC"), 1);
  glUniform1i(glGetUniformLocation(m_prog, "uTexO"), 2);

  // OpenGL texture setup
  glGenTextures(3, m_textures);
  for (int i = 0; i < 3;  ++i) {
    glBindTexture(GL_TEXTURE_2D, m_textures[i]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  }
  glBindTexture(GL_TEXTURE_2D, 0);

  // initial window size setup
  GLint vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);

  m_good = true;
}

EGLVideo::~EGLVideo() {
  eglMakeCurrent(m_egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
  eglDestroyContext(m_egl_display, m_egl_context);
  eglDestroySurface(m_egl_display, m_egl_surface);
  eglTerminate(m_egl_display);
  XDestroyWindow(m_x_display, m_window);
  XCloseDisplay(m_x_display);
}

bool EGLVideo::poll_events() {

  // handle X11 events
  while (XPending(m_x_display)) {
    XEvent ev;
    XNextEvent(m_x_display, &ev);
    switch (ev.type) {
    case ClientMessage:
      if (((Atom) ev.xclient.data.l[0]) == m_WM_DELETE_WINDOW) {
        m_running = false;
      }
      break;
    case KeyPress:
      switch (XLookupKeysym(&ev.xkey, 0)) {
      case 'q':
        m_running = false;
        break;
      default:
        break;
      }
      break;
    case ConfigureNotify:
      if (m_resize_cb) {
        m_width = ((XConfigureEvent*)&ev)->width;
        m_height = ((XConfigureEvent*)&ev)->height;
        m_resize_cb(m_width, m_height);
      }
      break;
    default:
      break;
    }
  }

  return m_running;
}

void EGLVideo::resize_texture(float x, float y) {
  if (!m_texture_size_valid) {
    m_texcoord_x1 = x;
    m_texcoord_y1 = y;
    glUniform2f(glGetUniformLocation(m_prog, "uTexCoordScale"), m_texcoord_x1, m_texcoord_y1);
    m_texture_size_valid = true;
  }
}

bool EGLVideo::draw_frame(EGLint img_attrs[2][13]) {

  EGLImage images[2];
  for (uint8_t i = 0; i < 2; i++) {
    images[i] = eglCreateImageKHR(m_egl_display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL,
                                  img_attrs[i]);
    if (!images[i]) {
      return false;
    }
    glActiveTexture(GL_TEXTURE0 + i);
    glBindTexture(GL_TEXTURE_2D, m_textures[i]);
    while (glGetError()) {}
    glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, images[i]);
    if (glGetError()) {
      m_good = false;
      return false;
    }
  }

  glActiveTexture(GL_TEXTURE0 + 2);
  glBindTexture(GL_TEXTURE_2D, m_textures[2]);
  while (glGetError()) {}
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_osd_width, m_osd_height, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, m_osd_fb);
  if (glGetError()) {
    m_good = false;
    return false;
  }

  // draw the frame
  glClear(GL_COLOR_BUFFER_BIT);
  while (glGetError()) {}
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  if (glGetError()) {
    m_good = false;
    return false;
  }

  // display the frame
  eglSwapBuffers(m_egl_display, m_egl_surface);

  // clean up the interop images
  for (int i = 0;  i < 3;  ++i) {
    glActiveTexture(GL_TEXTURE0 + i);
    glBindTexture(GL_TEXTURE_2D, 0);
  }
  eglDestroyImageKHR(m_egl_display, images[0]);
  eglDestroyImageKHR(m_egl_display, images[1]);

  return true;
}

#endif /* USE_FFMPEG_MONITOR */
