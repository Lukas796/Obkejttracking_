#include "hdmi_out.hpp"

#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>

// #include <sys/fcntl.h>// open function
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/mman.h>
#include <fcntl.h>

#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <unistd.h>

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

int hdmi_out_init(hdmi_out_context &hdmi_out_ctx){
  hdmi_out_ctx.fbfd = open ("/dev/fb0", O_RDWR);
  if (hdmi_out_ctx.fbfd < 0){
    return -1;
  }
  struct fb_var_screeninfo vinfo;
  
  ioctl(hdmi_out_ctx.fbfd, FBIOGET_VSCREENINFO, &vinfo);
  
  hdmi_out_ctx.fb_width = vinfo.xres;
  hdmi_out_ctx.fb_height = vinfo.yres;
  hdmi_out_ctx.fb_bpp = vinfo.bits_per_pixel;
  
  hdmi_out_ctx.fb_data_size = vinfo.xres * vinfo.yres* (vinfo.bits_per_pixel/8);
  
  hdmi_out_ctx.fbdata = (char*) mmap (0, hdmi_out_ctx.fb_data_size, PROT_READ | PROT_WRITE,
				      MAP_SHARED, hdmi_out_ctx.fbfd, (off_t)0);
  
  memset (hdmi_out_ctx.fbdata, 0, hdmi_out_ctx.fb_data_size);
  return 0;
}

int hdmi_out_show(hdmi_out_context &hdmi_out_ctx, cv::Mat img){
  cv::Mat img32;
  cv::cvtColor(img, img32, cv::COLOR_BGR2BGRA);
  cv::Mat resized_image;
  cv::Size fb_img_size(hdmi_out_ctx.fb_width, hdmi_out_ctx.fb_height);
  cv::resize(img32, resized_image, fb_img_size);
  memcpy((void*)hdmi_out_ctx.fbdata, (void*)resized_image.data, hdmi_out_ctx.fb_data_size);
  return 0;
}

void hdmi_out_deinit(hdmi_out_context &hdmi_out_ctx){
    munmap (hdmi_out_ctx.fbdata, hdmi_out_ctx.fb_data_size);
    close (hdmi_out_ctx.fbfd);
}
