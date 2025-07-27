#ifndef __HDMI_OUT_H__
#define __HDMI_OUT_H__

#include <opencv2/core/mat.hpp>

#ifdef __cplusplus
extern "C" {
#endif

  
  typedef struct _hdmi_out_context{
    char *fbdata;
    int fbfd;
    int fb_data_size;
    int fb_width;
    int fb_height;
    int fb_bpp;
  } hdmi_out_context;

  int hdmi_out_init(hdmi_out_context &hdmi_out_ctx);
  int hdmi_out_show(hdmi_out_context &hdmi_out_ctx, cv::Mat img);
  void hdmi_out_deinit(hdmi_out_context &hdmi_out_ctx);
  
#ifdef __cplusplus
}
#endif

#endif /*__HDMI_OUT_H__*/

