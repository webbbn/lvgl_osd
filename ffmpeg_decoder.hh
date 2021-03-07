#pragma once

#include "ffmpeg_monitor.h"
#if USE_FFMPEG_MONITOR

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_vaapi.h>
}

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_drmcommon.h>

#include <drm_fourcc.h>

#define DRM_NODE "/dev/dri/renderD128"

class FFMPEGDecoder {
public:

  FFMPEGDecoder(const char *url, const char *drm_node = DRM_NODE);
  ~FFMPEGDecoder();
  bool decode(EGLVideo &win);
  uint32_t width();
  uint32_t height();

private:
  bool m_good;
  int m_drm_fd;
  int m_video_stream;
  VADisplay m_va_display;
  AVFormatContext *m_input_ctx;
  AVCodec *m_decoder;
  AVCodecContext *m_decoder_ctx;
  AVBufferRef *m_hw_device_ctx;
  AVFrame *m_frame;
  bool m_receive_packets;
};

#endif /* USE_FFMPEG_MONITOR */
