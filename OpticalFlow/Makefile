# Optical Flow Makefile
SHELL = /bin/sh

# C++ compiler
CXX = g++

# OpenCV library path
OPENCV_PATH = /usr/lib/

# OpenCV libraries
OPENCV_LIBS = $(OPENCV_PATH)libopencv_core.so $(OPENCV_PATH)libopencv_highgui.so $(OPENCV_PATH)libopencv_video.so $(OPENCV_PATH)libopencv_imgproc.so

SOURCE_PATH = ./

BUILD_PATH = build/

OUTPUT_FILE = OpticalFlow

all: headless

headful: setup
	$(CXX) -c -D OPTFLOW_DISPLAY $(SOURCE_PATH)main.cpp -o $(BUILD_PATH)main.o
	$(CXX) $(BUILD_PATH)main.o $(OPENCV_LIBS) -o $(OUTPUT_FILE)

headless: setup
	$(CXX) -c $(SOURCE_PATH)main.cpp -o $(BUILD_PATH)main.o
	$(CXX) $(BUILD_PATH)main.o $(OPENCV_LIBS) -o $(OUTPUT_FILE)

setup:
	@if [[ -d $(BUILD_PATH) ]]; then rm -rf $(BUILD_PATH)*; else mkdir $(BUILD_PATH); fi

clean:
	@rm -rf ./build $(OUTPUT_FILE)