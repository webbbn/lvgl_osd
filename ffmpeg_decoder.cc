

#include "egl_video.hh"
#include "ffmpeg_decoder.hh"
#if USE_FFMPEG_MONITOR

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// callback to negotiate the output pixel format.
// we don't negotiate here, we just want VA-API.
static enum AVPixelFormat get_hw_format(AVCodecContext *ctx, const enum AVPixelFormat *pix_fmts) {
  (void)ctx, (void)pix_fmts;
  return AV_PIX_FMT_VAAPI;
}

FFMPEGDecoder::FFMPEGDecoder(const char *url, const char *drm_node) :
  m_good(false), m_input_ctx(0), m_decoder(0), m_decoder_ctx(0), m_hw_device_ctx(0),
  m_frame(0), m_receive_packets(true) {

  // initialize VA-API
  m_drm_fd = -1;
  m_va_display = 0;
  m_drm_fd = open(drm_node, O_RDWR);
  if (m_drm_fd < 0) {
    return; // Fail!
  }
  m_va_display = vaGetDisplayDRM(m_drm_fd);
  if (!m_va_display) {
    return; // Fail!
  }
  int major, minor;
  if (vaInitialize(m_va_display, &major, &minor) != VA_STATUS_SUCCESS) {
    return; // Fail!
  }

  // open input file, video stream and decoder
#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(58, 9, 100)
  av_register_all();
#endif
  if (avformat_open_input(&m_input_ctx, url, NULL, NULL) != 0) {
    return; // Fail!
  }
  if (avformat_find_stream_info(m_input_ctx, NULL) < 0) {
    return; // Fail!
  }
  m_video_stream = av_find_best_stream(m_input_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &m_decoder, 0);
  if (m_video_stream < 0) {
    return; // Fail!
  }
  m_decoder_ctx = avcodec_alloc_context3(m_decoder);
  if (!m_decoder_ctx) {
    return; // Fail!
  }
  if (avcodec_parameters_to_context(m_decoder_ctx, m_input_ctx->streams[m_video_stream]->codecpar)
      < 0) {
    return; // Fail!
  }

  // use av_hwdevice_ctx_alloc() and populate the underlying structure
  // to use the VA-API context ("display") we created before
  m_hw_device_ctx = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_VAAPI);
  if (!m_hw_device_ctx) {
    return; // Fail!
  }
  AVHWDeviceContext *hwctx = reinterpret_cast<AVHWDeviceContext*>(m_hw_device_ctx->data);
  AVVAAPIDeviceContext *vactx = reinterpret_cast<AVVAAPIDeviceContext*>(hwctx->hwctx);
  vactx->display = m_va_display;
  if (av_hwdevice_ctx_init(m_hw_device_ctx) < 0) {
    return; // Fail!
  }
  m_decoder_ctx->get_format = get_hw_format;
  m_decoder_ctx->hw_device_ctx = av_buffer_ref(m_hw_device_ctx);
  if (avcodec_open2(m_decoder_ctx, m_decoder, NULL) < 0) {
    return; // Fail!
  }
  printf("Opened input video stream: %dx%d\n", m_decoder_ctx->width, m_decoder_ctx->height);

  // allocate AVFrame for display
  m_frame = av_frame_alloc();
  if (!m_frame) {
    return; // Fail!
  }

  m_good = true;
}

FFMPEGDecoder::~FFMPEGDecoder() {
  av_frame_free(&m_frame);
  avcodec_free_context(&m_decoder_ctx);
  avformat_close_input(&m_input_ctx);
  av_buffer_unref(&m_hw_device_ctx);
  if (m_drm_fd >= 0) {
    close(m_drm_fd);
  }
  vaTerminate(m_va_display);
}

bool FFMPEGDecoder::decode(EGLVideo &win) {

  // main loop
  bool m_receive_packets = true;
  while (win.poll_events()) {

    // read compressed data from stream and send it to the decoder
    if (m_receive_packets) {
      AVPacket packet;
      bool error = false;
      if (av_read_frame(m_input_ctx, &packet) < 0) {
        error = false; // end of stream
      } else if (packet.stream_index == m_video_stream) {
        if (avcodec_send_packet(m_decoder_ctx, &packet) < 0) {
          error = false;
        } else {
          m_receive_packets = false;
        }
      }
      av_packet_unref(&packet);
      if (error) {
        return false;
      } else if (m_receive_packets) {
        continue;
      }
    }

    // retrieve a frame from the decoder
    int ret = avcodec_receive_frame(m_decoder_ctx, m_frame);
    if ((ret == AVERROR(EAGAIN)) || (ret == AVERROR_EOF)) {
      // no more frames ready from the decoder -> decode new ones
      m_receive_packets = true;
      continue;
    } else if (ret < 0) {
      break; // Fail!
    }

    // convert the frame into a pair of DRM-PRIME FDs
    VASurfaceID va_surface = (uintptr_t)m_frame->data[3];
    VADRMPRIMESurfaceDescriptor prime;
    if (vaExportSurfaceHandle(m_va_display, va_surface,
                              VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
                              VA_EXPORT_SURFACE_READ_ONLY |
                              VA_EXPORT_SURFACE_SEPARATE_LAYERS,
                              &prime) != VA_STATUS_SUCCESS) {
      m_good = false;
      return false; // Fail!
    }
    if (prime.fourcc != VA_FOURCC_NV12) {
      m_good = false;
      return false; // Fail!
    }
    vaSyncSurface(m_va_display, va_surface);

    // check the actual size of the frame
    win.resize_texture((float)((double)m_decoder_ctx->width  / (double)prime.width),
                       (float)((double)m_decoder_ctx->height / (double)prime.height));

    // import the frame into OpenGL
    static const EGLint formats[2] = { DRM_FORMAT_R8, DRM_FORMAT_GR88 };
    if ((prime.layers[0].drm_format != static_cast<uint32_t>(formats[0])) ||
        (prime.layers[1].drm_format != static_cast<uint32_t>(formats[1]))) {
      m_good = false;
      return false; // Fail!
    }
    EGLint img_attrs[2][13] = 
      { {
         EGL_LINUX_DRM_FOURCC_EXT,      formats[0],
         EGL_WIDTH,                     static_cast<EGLint>(prime.width),
         EGL_HEIGHT,                    static_cast<EGLint>(prime.height),
         EGL_DMA_BUF_PLANE0_FD_EXT,     prime.objects[prime.layers[0].object_index[0]].fd,
         EGL_DMA_BUF_PLANE0_OFFSET_EXT, static_cast<EGLint>(prime.layers[0].offset[0]),
         EGL_DMA_BUF_PLANE0_PITCH_EXT,  static_cast<EGLint>(prime.layers[0].pitch[0]),
         EGL_NONE
         },
        {
         EGL_LINUX_DRM_FOURCC_EXT,      formats[1],
         EGL_WIDTH,                     static_cast<EGLint>(prime.width) / 2, // half size for chroma
         EGL_HEIGHT,                    static_cast<EGLint>(prime.height) / 2,
         EGL_DMA_BUF_PLANE0_FD_EXT,     prime.objects[prime.layers[1].object_index[0]].fd,
         EGL_DMA_BUF_PLANE0_OFFSET_EXT, static_cast<EGLint>(prime.layers[1].offset[0]),
         EGL_DMA_BUF_PLANE0_PITCH_EXT,  static_cast<EGLint>(prime.layers[1].pitch[0]),
         EGL_NONE
        } };
    win.draw_frame(img_attrs);
    for (int i = 0;  i < (int)prime.num_objects;  ++i) {
      close(prime.objects[i].fd);
    }
  }
  return true;
}

uint32_t FFMPEGDecoder::width() {
  if (m_decoder_ctx) {
    return m_decoder_ctx->width;
  }
  return 0;
}

uint32_t FFMPEGDecoder::height() {
  if (m_decoder_ctx) {
    return m_decoder_ctx->height;
  }
  return 0;
}

#endif /* USE_FFMPEG_MONITOR */
