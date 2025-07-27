
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#define _BASETSD_H

#include "RgaUtils.h"
#include "postprocess.h"
#include "rknn_api.h"
#include "preprocess.h"
#include "hdmi_out.hpp"

#include <opencv2/videoio.hpp>

static void dump_tensor_attr(rknn_tensor_attr *attr)
{
  std::string shape_str = attr->n_dims < 1 ? "" : std::to_string(attr->dims[0]);
  for (int i = 1; i < attr->n_dims; ++i)
  {
    shape_str += ", " + std::to_string(attr->dims[i]);
  }

  printf("  index=%d, name=%s, n_dims=%d, dims=[%s], n_elems=%d, size=%d, w_stride = %d, size_with_stride=%d, fmt=%s, "
         "type=%s, qnt_type=%s, "
         "zp=%d, scale=%f\n",
         attr->index, attr->name, attr->n_dims, shape_str.c_str(), attr->n_elems, attr->size, attr->w_stride,
         attr->size_with_stride, get_format_string(attr->fmt), get_type_string(attr->type),
         get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz)
{
  unsigned char *data;
  int ret;

  data = NULL;

  if (NULL == fp)
  {
    return NULL;
  }

  ret = fseek(fp, ofst, SEEK_SET);
  if (ret != 0)
  {
    printf("blob seek failure.\n");
    return NULL;
  }

  data = (unsigned char *)malloc(sz);
  if (data == NULL)
  {
    printf("buffer malloc failure.\n");
    return NULL;
  }
  ret = fread(data, 1, sz, fp);
  return data;
}

static unsigned char *load_model(const char *filename, int *model_size)
{
  FILE *fp;
  unsigned char *data;

  fp = fopen(filename, "rb");
  if (NULL == fp)
  {
    printf("Open file %s failed.\n", filename);
    return NULL;
  }

  fseek(fp, 0, SEEK_END);
  int size = ftell(fp);

  data = load_data(fp, 0, size);

  fclose(fp);

  *model_size = size;
  return data;
}
/*-------------------------------------------
                  Main Functions
-------------------------------------------*/
int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("Usage: %s <rknn model>\n", argv[0]);
    return -1;
  }

  rknn_context ctx;
  size_t actual_size = 0;


  const float nms_threshold = NMS_THRESH;      // Default NMS threshold 默认的NMS阈值
  const float box_conf_threshold = BOX_THRESH; // Default confidence threshold 默认的置信度阈值
  struct timeval start_time, stop_time;
  char *model_name = (char *)argv[1];

  // init rga context (RGA = Raster Graphic Accelleration)
  rga_buffer_t src;
  rga_buffer_t dst;
  memset(&src, 0, sizeof(src));
  memset(&dst, 0, sizeof(dst));
  printf("post process config: box_conf_threshold = %.2f, nms_threshold = %.2f\n", box_conf_threshold, nms_threshold);

  /* Create the neural network */
  printf("Loading mode...\n");
  int model_data_size = 0;
  unsigned char *model_data = load_model(model_name, &model_data_size);
  int ret = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
  if (ret < 0){
    printf("rknn_init error ret=%d\n", ret);
    return -1;
  }

  rknn_sdk_version version;
  ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
  if (ret < 0)
  {
    printf("rknn_init error ret=%d\n", ret);
    return -1;
  }
  printf("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);

  rknn_input_output_num io_num;
  ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
  if (ret < 0)
  {
    printf("rknn_init error ret=%d\n", ret);
    return -1;
  }
  printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

  rknn_tensor_attr input_attrs[io_num.n_input];
  memset(input_attrs, 0, sizeof(input_attrs));
  for (int i = 0; i < io_num.n_input; i++)
  {
    input_attrs[i].index = i;
    ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
    if (ret < 0)
    {
      printf("rknn_init error ret=%d\n", ret);
      return -1;
    }
    dump_tensor_attr(&(input_attrs[i]));
  }

  rknn_tensor_attr output_attrs[io_num.n_output];
  memset(output_attrs, 0, sizeof(output_attrs));
  for (int i = 0; i < io_num.n_output; i++)
  {
    output_attrs[i].index = i;
    ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
    dump_tensor_attr(&(output_attrs[i]));
  }

  int channel = 3;
  int width = 0;
  int height = 0;
  if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
  {
    printf("model is NCHW input fmt\n");
    channel = input_attrs[0].dims[1];
    height = input_attrs[0].dims[2];
    width = input_attrs[0].dims[3];
  }
  else
  {
    printf("model is NHWC input fmt\n");
    height = input_attrs[0].dims[1];
    width = input_attrs[0].dims[2];
    channel = input_attrs[0].dims[3];
  }

  printf("model input height=%d, width=%d, channel=%d\n", height, width, channel);

  rknn_input inputs[1];
  memset(inputs, 0, sizeof(inputs));
  inputs[0].index = 0;
  inputs[0].type = RKNN_TENSOR_UINT8;
  inputs[0].size = width * height * channel;
  inputs[0].fmt = RKNN_TENSOR_NHWC;
  inputs[0].pass_through = 0;

  //// Read Image from HDMI-IN
  // source : https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html
  cv::Mat frame;
  //--- INITIALIZE VIDEOCAPTURE
  cv::VideoCapture cap;
  // open the default camera using default API
  // cap.open(0);
  // OR advance usage: select any API backend
  int deviceID = 0;             // 0 = open default camera
  int apiID = cv::CAP_ANY;      // 0 = autodetect default API
  // open selected camera using selected API
  cap.open(deviceID, apiID);
  // check if we succeeded
  if (!cap.isOpened()) {
    printf( "ERROR! Unable to open camera\n");
    return -1;
  }


  gettimeofday(&start_time, NULL);  
  cap.read(frame);
  cv::Mat img;
  cv::cvtColor(frame, img, cv::COLOR_BGR2RGB);
  int img_width = img.cols;
  int img_height = img.rows;
    
  cv::Size target_size(width, height);
  cv::Mat resized_img(target_size.height, target_size.width, CV_8UC3);
  // Calculate scaling 计算缩放比例
  float scale_w = (float)target_size.width / img.cols;
  float scale_h = (float)target_size.height / img.rows;

  // Specify the target size and preprocessing method. By default, LetterBox's preprocessing is used.
  //指定目标大小和预处理方式,默认使用LetterBox的预处理
  BOX_RECT pads;
  memset(&pads, 0, sizeof(BOX_RECT));

  if (img_width != width || img_height != height){
    if(true){
      // Direct scaling uses RGA acceleration 直接缩放采用RGA加速
      printf("resize image by rga\n");
      ret = resize_rga(src, dst, img, resized_img, target_size);
      if (ret != 0){
	fprintf(stderr, "resize with rga error\n");
	return -1;
      }
      cv::imwrite("resize_input.jpg", resized_img);
    }else { 
      // scale with hardware acceleration 
      printf("resize image with letterbox\n");
      float min_scale = std::min(scale_w, scale_h);
      scale_w = min_scale;
      scale_h = min_scale;
      letterbox(img, resized_img, pads, min_scale, target_size);
      cv::imwrite("letterbox_input.jpg", resized_img);
    }
    inputs[0].buf = resized_img.data;
  }
  else{
    inputs[0].buf = img.data;
  }
  gettimeofday(&stop_time, NULL);
  printf("resize once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);
  printf("img width = %d, img height = %d\n", img_width, img_height);
  
  gettimeofday(&start_time, NULL);
  rknn_inputs_set(ctx, io_num.n_input, inputs);

  rknn_output outputs[io_num.n_output];
  memset(outputs, 0, sizeof(outputs));
  for (int i = 0; i < io_num.n_output; i++)
  {
    outputs[i].index = i;
    outputs[i].want_float = 0;
  }

  // Perform reasoning 执行推理
  ret = rknn_run(ctx, NULL);
  ret = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);

  gettimeofday(&stop_time, NULL);
  printf("krnn_run(.) once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);

   
  
  // Post-processing 后处理
  gettimeofday(&start_time, NULL);
  detect_result_group_t detect_result_group;
  std::vector<float> out_scales;
  std::vector<int32_t> out_zps;
  for (int i = 0; i < io_num.n_output; ++i)
  {
    out_scales.push_back(output_attrs[i].scale);
    out_zps.push_back(output_attrs[i].zp);
  }
  post_process((int8_t *)outputs[0].buf, (int8_t *)outputs[1].buf, (int8_t *)outputs[2].buf, height, width,
               box_conf_threshold, nms_threshold, pads, scale_w, scale_h, out_zps, out_scales, &detect_result_group);


  ret = rknn_outputs_release(ctx, io_num.n_output, outputs);

  // Frames and Probabilities 画框和概率
  char text[256];
  for (int i = 0; i < detect_result_group.count; i++)
  {
    detect_result_t *det_result = &(detect_result_group.results[i]);
    sprintf(text, "%s %.1f%%", det_result->name, det_result->prop * 100);
    printf("%s @ (%d %d %d %d) %f\n", det_result->name, det_result->box.left, det_result->box.top,
           det_result->box.right, det_result->box.bottom, det_result->prop);
    int x1 = det_result->box.left;
    int y1 = det_result->box.top;
    int x2 = det_result->box.right;
    int y2 = det_result->box.bottom;
    rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(256, 0, 0, 256), 3);
    putText(frame, text, cv::Point(x1, y1 + 12), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
  }
  printf("Start HDMI out\n");
  //// Write Image to HDMI out
  // 2DO 
  hdmi_out_context hdmi_out_ctx;  
  memset(&hdmi_out_ctx, 0, sizeof(hdmi_out_ctx));
  ret =  hdmi_out_init(hdmi_out_ctx);
  printf("Start HDMI out init finish\n");
  if (0 != ret ){
    fprintf(stderr, "resize with rga error\n");
    return -1;
  }
  hdmi_out_show(hdmi_out_ctx, frame);
  gettimeofday(&stop_time, NULL);
  printf("postprocess once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);
  
  ///  Loop 
  while (true){
    gettimeofday(&start_time, NULL);
    /// read HDMI in and rescale
    cap.read(frame);
    gettimeofday(&start_time, NULL);
    cv::cvtColor(frame, img, cv::COLOR_BGR2RGB);
    gettimeofday(&stop_time, NULL);
    printf("read and colerconvert while once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);
    
    float min_scale = std::min(scale_w, scale_h);
    scale_w = min_scale;
    scale_h = min_scale;
    letterbox(img, resized_img, pads, min_scale, target_size);
    //ret = resize_rga(src, dst, img, resized_img, target_size);

    gettimeofday(&stop_time, NULL);
    printf("letterbox while once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);
    
    //if (ret != 0){
    //  fprintf(stderr, "resize with rga error\n");
    //  return -1;
    //}
    inputs[0].buf = resized_img.data;

    /// run npu 
    rknn_inputs_set(ctx, io_num.n_input, inputs);
    ret = rknn_run(ctx, NULL);
    ret = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);
    post_process((int8_t *)outputs[0].buf, (int8_t *)outputs[1].buf, (int8_t *)outputs[2].buf, height, width,
                 box_conf_threshold, nms_threshold, pads, scale_w, scale_h, out_zps, out_scales, &detect_result_group);
    ret = rknn_outputs_release(ctx, io_num.n_output, outputs);
    gettimeofday(&stop_time, NULL);
    printf("post process while once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);
    
    // Frames and Probabilities 画框和概率
    char text[256];
    for (int i = 0; i < detect_result_group.count; i++) {
      detect_result_t *det_result = &(detect_result_group.results[i]);
      sprintf(text, "%s %.1f%%", det_result->name, det_result->prop * 100);
      printf("%s @ (%d %d %d %d) %f\n", det_result->name, det_result->box.left, det_result->box.top,
	     det_result->box.right, det_result->box.bottom, det_result->prop);
      int x1 = det_result->box.left;
      int y1 = det_result->box.top;
      int x2 = det_result->box.right;
      int y2 = det_result->box.bottom;
      rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(256, 0, 0, 256), 3);
      putText(frame, text, cv::Point(x1, y1 + 12), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
    }
    // write HDMI-OUT
    hdmi_out_show(hdmi_out_ctx, frame);
    
    gettimeofday(&stop_time, NULL);
    printf("while once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);
    
}
  deinitPostProcess();

  // release
  ret = rknn_destroy(ctx);

  if (model_data)
  {
    free(model_data);
  }

  return 0;
}
