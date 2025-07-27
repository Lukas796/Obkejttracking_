#!/bin/bash

export LD_LIBRARY_PATH=./lib/
./rknn_opi_hdmi_track model/RK3588/yolov5s-640-640.rknn   > out.log 2>&1




