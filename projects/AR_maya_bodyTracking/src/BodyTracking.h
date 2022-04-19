#pragma once
#ifndef __BODYTRACKING_H__
#define __BODYTRACKING_H__
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "BodyEngine.h"
#include "RenderingUtils.h"
#include "nvAR.h"
#include "nvAR_defs.h"
#include "opencv2/opencv.hpp"

#ifndef M_PI
#define M_PI 3.1415926535897932385
#endif /* M_PI */
#ifndef M_2PI
#define M_2PI 6.2831853071795864769
#endif /* M_2PI */
#ifndef M_PI_2
#define M_PI_2 1.5707963267948966192
#endif /* M_PI_2 */
#define F_PI ((float)M_PI)
#define F_PI_2 ((float)M_PI_2)
#define F_2PI ((float)M_2PI)

#ifdef _MSC_VER
#define strcasecmp _stricmp
#endif /* _MSC_VER */

#define BAIL(err, code) \
  do {                  \
    err = code;         \
    goto bail;          \
  } while (0)

#define DEBUG_RUNTIME


enum {
  myErrNone = 0,
  myErrShader = -1,
  myErrProgram = -2,
  myErrTexture = -3,
};


#if 1
class MyTimer {
 public:
  void start() { t0 = std::chrono::high_resolution_clock::now(); }       /**< Start  the timer. */
  void pause() { dt = std::chrono::high_resolution_clock::now() - t0; }  /**< Pause  the timer. */
  void resume() { t0 = std::chrono::high_resolution_clock::now() - dt; } /**< Resume the timer. */
  void stop() { pause(); }                                               /**< Stop   the timer. */
  double elapsedTimeFloat() const {
    return std::chrono::duration<double>(dt).count();
  } /**< Report the elapsed time as a float. */
 private:
  std::chrono::high_resolution_clock::time_point t0;
  std::chrono::high_resolution_clock::duration dt;
};
#endif


class DoApp {
public:
  enum Err {
    errNone           = BodyEngine::Err::errNone,
    errGeneral        = BodyEngine::Err::errGeneral,
    errRun            = BodyEngine::Err::errRun,
    errInitialization = BodyEngine::Err::errInitialization,
    errRead           = BodyEngine::Err::errRead,
    errEffect         = BodyEngine::Err::errEffect,
    errParameter      = BodyEngine::Err::errParameter,
    errUnimplemented,
    errMissing,
    errVideo,
    errImageSize,
    errNotFound,
    errBodyModelInit,
    errGLFWInit,
    errGLInit,
    errRendererInit,
    errGLResource,
    errGLGeneric,
    errBodyFit,
    errNoBody,
    errSDK,
    errCuda,
    errCancel,
    errCamera
  };
  Err doAppErr(BodyEngine::Err status) { return (Err)status; }
  BodyEngine body_ar_engine;
  DoApp();
  ~DoApp() {};

  // SETTINGS
  bool debug; 
  bool verbose; 
  bool temporal; 
  bool captureOutputs;
  bool offlineMode;
  bool writeOutFiles;
  bool useCudaGraph;
  std::string outDir; 
  std::string inFile; 
  std::string outFile;
  std::string modelPath;
  std::string captureCodec;
  std::string camRes;
  std::string bodyModel;
  unsigned int appMode;
  unsigned int mode;
  unsigned int camindex;

  void stop();
  Err initBodyEngine(const char *modelPath = nullptr);
  Err initCamera(const char *camRes = nullptr);
  Err initOfflineMode(const char *inputFilename = nullptr, const char *outputFilename = nullptr);
  Err acquireFrame();
  Err acquireBodyBox();
  Err acquireBodyBoxAndKeyPoints();
  Err run();
  void drawFPS(cv::Mat &img);
  void DrawBBoxes(const cv::Mat &src, NvAR_Rect *output_bbox);
  void DrawKeyPointLine(const cv::Mat& src, NvAR_Point2f* keypoints, int point1, int point2, int color);
  void DrawKeyPointsAndEdges(const cv::Mat &src, NvAR_Point2f *keypoints, int numKeyPoints, NvAR_Rect* output_bbox);
  void drawKalmanStatus(cv::Mat &img);
  void drawVideoCaptureStatus(cv::Mat &img);
  void processKey(int key);
  void writeVideoAndEstResults(const cv::Mat &frame, NvAR_BBoxes output_bboxes, NvAR_Point2f *keypoints = NULL);
  void writeFrameAndEstResults(const cv::Mat &frame, NvAR_BBoxes output_bboxes, NvAR_Point2f *keypoints = NULL);
  void writeEstResults(std::ofstream &outputFile, NvAR_BBoxes output_bboxes, NvAR_Point2f *keypoints = NULL);
  void getFPS();
  static const char *errorStringFromCode(Err code);

  cv::VideoCapture cap{};
  cv::Mat frame;
  int inputWidth, inputHeight;
  cv::VideoWriter bodyDetectOutputVideo{}, keyPointsOutputVideo{};
  int frameIndex;
  static const char windowTitle[];
  double frameTime;
  // std::chrono::high_resolution_clock::time_point frameTimer;
  MyTimer frameTimer;
  cv::VideoWriter capturedVideo;
  std::ofstream bodyEngineVideoOutputFile;

  BodyEngine::Err nvErr;
  float expr[6];
  bool drawVisualization, captureVideo, captureFrame;
  bool showFPS, showBBox;
  float scaleOffsetXY[4];
};
#endif /* __BODYTRACKING_H__ */