
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

#include "BodyTracking.h"
#include "RenderingUtils.h"
#include "nvAR.h"
#include "nvAR_defs.h"
#include "opencv2/opencv.hpp"


DoApp *gApp = nullptr;

char *g_nvARSDKPath = NULL;


std::string getCalendarTime() {
  // Get the current time
  std::chrono::system_clock::time_point currentTimePoint = std::chrono::system_clock::now();
  // Convert to time_t from time_point
  std::time_t currentTime = std::chrono::system_clock::to_time_t(currentTimePoint);
  // Convert to tm to get structure holding a calendar date and time broken down into its components.
  std::tm brokenTime = *std::localtime(&currentTime);
  std::ostringstream calendarTime;
  // calendarTime << std::put_time(
  //     &brokenTime,
  //     "%Y-%m-%d-%H-%M-%S");  // (YYYY-MM-DD-HH-mm-ss)<Year>-<Month>-<Date>-<Hour>-<Mins>-<Seconds>
  char time_string[24];
  if (0 < strftime(time_string, sizeof(time_string), "%Y-%m-%d-%H-%M-%S] ", &brokenTime))
    calendarTime << time_string;  // (YYYY-MM-DD-HH-mm-ss)<Year>-<Month>-<Date>-<Hour>-<Mins>-<Seconds>
  // Get the time since epoch 0(Thu Jan  1 00:00:00 1970) and the remainder after division is
  // our milliseconds
  std::chrono::milliseconds currentMilliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(currentTimePoint.time_since_epoch()) % 1000;
  // Append the milliseconds to the stream
  calendarTime << "-" << std::setfill('0') << std::setw(3) << currentMilliseconds.count();  // milliseconds
  return calendarTime.str();
}

/********************************************************************************
 * StringToFourcc
 ********************************************************************************/

static int StringToFourcc(const std::string &str) {
  union chint {
    int i;
    char c[4];
  };
  chint x = {0};
  for (int n = (str.size() < 4) ? (int)str.size() : 4; n--;) x.c[n] = str[n];
  return x.i;
}


static const cv::Scalar cv_colors[] = { cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0) };

enum {
  kColorRed = 0,
  kColorGreen = 1,
  kColorBlue = 2
};


void DoApp::processKey(int key) {
  switch (key) {
    case '2':
      body_ar_engine.destroyFeatures();
      body_ar_engine.setAppMode(BodyEngine::mode::keyPointDetection);
      body_ar_engine.createFeatures(modelPath.c_str());
      body_ar_engine.initFeatureIOParams();
      break;
    case '1':
      body_ar_engine.destroyFeatures();
      body_ar_engine.setAppMode(BodyEngine::mode::bodyDetection);
      body_ar_engine.createFeatures(modelPath.c_str());
      body_ar_engine.initFeatureIOParams();
      break;
    case 'C':
    case 'c':
      captureVideo = !captureVideo;
      break;
    case 'S':
    case 's':
      captureFrame = !captureFrame;
      break;
    case 'W':
    case 'w':
      drawVisualization = !drawVisualization;
      break;
    case 'F':
    case 'f':
      showFPS = !showFPS;
      break;
    default:
      break;
  }
}

DoApp::Err DoApp::initBodyEngine(const char *modelPath) {
  if (!cap.isOpened()) return errVideo;

  int numKeyPoints = body_ar_engine.getNumKeyPoints();

  nvErr = body_ar_engine.createFeatures(modelPath);

#ifdef DEBUG
  detector->setOutputLocation(outputDir);
#endif  // DEBUG

#define VISUALIZE
#ifdef VISUALIZE
  if (!writeOutFiles) cv::namedWindow(windowTitle, 1);
#endif  // VISUALIZE

  frameIndex = 0;

  return doAppErr(nvErr);
}

void DoApp::stop() {
  body_ar_engine.destroyFeatures();

  if (offlineMode && writeOutFiles) {
    bodyDetectOutputVideo.release();
    keyPointsOutputVideo.release();
  }
  cap.release();
#ifdef VISUALIZE
  cv::destroyAllWindows();
#endif  // VISUALIZE
}

void DoApp::DrawBBoxes(const cv::Mat &src, NvAR_Rect *output_bbox) {
  cv::Mat frm;
  if (offlineMode)
    frm = src.clone();
  else
    frm = src;

  if (output_bbox)
    cv::rectangle(frm, cv::Point(lround(output_bbox->x), lround(output_bbox->y)),
                  cv::Point(lround(output_bbox->x + output_bbox->width), lround(output_bbox->y + output_bbox->height)),
                  cv::Scalar(255, 0, 0), 2);
  if (offlineMode && writeOutFiles) bodyDetectOutputVideo.write(frm);
}

void DoApp::writeVideoAndEstResults(const cv::Mat &frm, NvAR_BBoxes output_bboxes, NvAR_Point2f* keypoints) {
  if (captureVideo) {
    if (!capturedVideo.isOpened()) {
      const std::string currentCalendarTime = getCalendarTime();
      const std::string capturedOutputFileName = currentCalendarTime + ".mp4";
      getFPS();
      if (frameTime) {
        float fps = (float)(1.0 / frameTime);
        capturedVideo.open(capturedOutputFileName, StringToFourcc(captureCodec), fps,
                           cv::Size(frm.cols, frm.rows));
        if (!capturedVideo.isOpened()) {
          std::cout << "Error: Could not open video: \"" << capturedOutputFileName << "\"\n";
          return;
        }
        if (verbose) {
          std::cout << "Capturing video started" << std::endl;
        }
      } else {  // If frameTime is 0.f, returns without writing the frame to the Video
        return;
      }
      const std::string outputsFileName = currentCalendarTime + ".txt";
      bodyEngineVideoOutputFile.open(outputsFileName, std::ios_base::out);
      if (!bodyEngineVideoOutputFile.is_open()) {
        std::cout << "Error: Could not open file: \"" << outputsFileName << "\"\n";
        return;
      }
      std::string keyPointDetectionMode = (keypoints == NULL) ? "Off" : "On";
      bodyEngineVideoOutputFile << "// BodyDetectOn, KeyPointDetect" << keyPointDetectionMode << "\n ";
      bodyEngineVideoOutputFile
          << "// kNumPeople, (bbox_x, bbox_y, bbox_w, bbox_h){ kNumPeople}, kNumLMs, [lm_x, lm_y]{kNumLMs}\n";
    }
    // Write each frame to the Video
    capturedVideo << frm;
    writeEstResults(bodyEngineVideoOutputFile, output_bboxes, keypoints);
  } else {
    if (capturedVideo.isOpened()) {
      if (verbose) {
        std::cout << "Capturing video ended" << std::endl;
      }
      capturedVideo.release();
      if (bodyEngineVideoOutputFile.is_open()) bodyEngineVideoOutputFile.close();
    }
  }
}

void DoApp::writeEstResults(std::ofstream &outputFile, NvAR_BBoxes output_bboxes, NvAR_Point2f* keypoints) {
  /**
   * Output File Format :
   * BodyDetectOn, KeyPointDetectOn
   * kNumPeople, (bbox_x, bbox_y, bbox_w, bbox_h){ kNumPeople}, kNumKPs, [j_x, j_y]{kNumKPs}
   */

  int bodyDetectOn = (body_ar_engine.appMode == BodyEngine::mode::bodyDetection ||
                      body_ar_engine.appMode == BodyEngine::mode::keyPointDetection)
                         ? 1
                         : 0;
  int keyPointDetectOn = (body_ar_engine.appMode == BodyEngine::mode::keyPointDetection)
                             ? 1
                             : 0;
  outputFile << bodyDetectOn << "," << keyPointDetectOn << "\n";

  if (bodyDetectOn && output_bboxes.num_boxes) {
    // Append number of bodies detected in the current frame
    outputFile << unsigned(output_bboxes.num_boxes) << ",";
    // write outputbboxes to outputFile
    for (size_t i = 0; i < output_bboxes.num_boxes; i++) {
      int x1 = (int)output_bboxes.boxes[i].x, y1 = (int)output_bboxes.boxes[i].y,
          width = (int)output_bboxes.boxes[i].width, height = (int)output_bboxes.boxes[i].height;
      outputFile << x1 << "," << y1 << "," << width << "," << height << ",";
    }
  } else {
    outputFile << "0,";
  }
  if (keyPointDetectOn && output_bboxes.num_boxes) {
    int numKeyPoints = body_ar_engine.getNumKeyPoints();
    // Append number of keypoints
    outputFile << numKeyPoints << ",";
    // Append 2 * number of keypoint values
    NvAR_Point2f *pt, *endPt;
    for (endPt = (pt = (NvAR_Point2f *)keypoints) + numKeyPoints; pt < endPt; ++pt)
      outputFile << pt->x << "," << pt->y << ",";
  } else {
    outputFile << "0,";
  }

  outputFile << "\n";
}

void DoApp::writeFrameAndEstResults(const cv::Mat &frm, NvAR_BBoxes output_bboxes, NvAR_Point2f* keypoints) {
  if (captureFrame) {
    const std::string currentCalendarTime = getCalendarTime();
    const std::string capturedFrame = currentCalendarTime + ".png";
    cv::imwrite(capturedFrame, frm);
    if (verbose) {
      std::cout << "Captured the frame" << std::endl;
    }
    // Write Body Engine Outputs
    const std::string outputFilename = currentCalendarTime + ".txt";
    std::ofstream outputFile;
    outputFile.open(outputFilename, std::ios_base::out);
    if (!outputFile.is_open()) {
      std::cout << "Error: Could not open file: \"" << outputFilename << "\"\n";
      return;
    }
    std::string keyPointDetectionMode = (keypoints == NULL) ? "Off" : "On";
    outputFile << "// BodyDetectOn, KeyPointDetect" << keyPointDetectionMode << "\n";
    outputFile << "// kNumPeople, (bbox_x, bbox_y, bbox_w, bbox_h){ kNumPeople}, kNumLMs, [lm_x, lm_y]{kNumLMs}\n";
    writeEstResults(outputFile, output_bboxes, keypoints);
    if (outputFile.is_open()) outputFile.close();
    captureFrame = false;
  }
}

void DoApp::DrawKeyPointLine(const cv::Mat& src, NvAR_Point2f* keypoints, int point1, int point2, int color) {
  NvAR_Point2f point1_pos = *(keypoints + point1);
  NvAR_Point2f point2_pos = *(keypoints + point2);
  cv::line(src, cv::Point((int)point1_pos.x, (int)point1_pos.y), cv::Point((int)point2_pos.x, (int)point2_pos.y), cv_colors[color], 2);

}

void DoApp::DrawKeyPointsAndEdges(const cv::Mat& src, NvAR_Point2f* keypoints, int numKeyPoints, NvAR_Rect* output_bbox) {
  cv::Mat frm;
  if (writeOutFiles)
    frm = src.clone();
  else
    frm = src;
  NvAR_Point2f *pt, *endPt;
  for (endPt = (pt = (NvAR_Point2f *)keypoints) + numKeyPoints; pt < endPt; ++pt)
    cv::circle(frm, cv::Point(lround(pt->x), lround(pt->y)), 4, cv::Scalar(180, 180, 180), -1);

  if (output_bbox && showBBox)
      cv::rectangle(frm, cv::Point(lround(output_bbox->x), lround(output_bbox->y)),
          cv::Point(lround(output_bbox->x + output_bbox->width), lround(output_bbox->y + output_bbox->height)),
          cv::Scalar(255, 0, 0), 2);

  int pelvis = 0;
  int left_hip = 1;
  int right_hip = 2;
  int torso = 3;
  int left_knee = 4;
  int right_knee = 5;
  int neck = 6;
  int left_ankle = 7;
  int right_ankle = 8;
  int left_big_toe = 9;
  int right_big_toe = 10;
  int left_small_toe = 11;
  int right_small_toe = 12;
  int left_heel = 13;
  int right_heel = 14;
  int nose = 15;
  int left_eye = 16;
  int right_eye = 17;
  int left_ear = 18;
  int right_ear = 19;
  int left_shoulder = 20;
  int right_shoulder = 21;
  int left_elbow = 22;
  int right_elbow = 23;
  int left_wrist = 24;
  int right_wrist = 25;
  int left_pinky_knuckle = 26;
  int right_pinky_knuckle = 27;
  int left_middle_tip = 28;
  int right_middle_tip = 29;
  int left_index_knuckle = 30;
  int right_index_knuckle = 31;
  int left_thumb_tip = 32;
  int right_thumb_tip = 33;

  // center body
  DrawKeyPointLine(frm, keypoints, pelvis, torso, kColorGreen);
  DrawKeyPointLine(frm, keypoints, torso, neck, kColorGreen);
  DrawKeyPointLine(frm, keypoints, neck, pelvis, kColorGreen);

  // right side
  DrawKeyPointLine(frm, keypoints, right_ankle, right_knee, kColorRed);
  DrawKeyPointLine(frm, keypoints, right_knee, right_hip, kColorRed);
  DrawKeyPointLine(frm, keypoints, right_hip, pelvis, kColorRed);
  DrawKeyPointLine(frm, keypoints, right_hip, right_shoulder, kColorRed);
  DrawKeyPointLine(frm, keypoints, right_shoulder, right_elbow, kColorRed);
  DrawKeyPointLine(frm, keypoints, right_elbow, right_wrist, kColorRed);
  DrawKeyPointLine(frm, keypoints, right_shoulder, neck, kColorRed);

  // right side hand and feet
  DrawKeyPointLine(frm, keypoints, right_wrist, right_pinky_knuckle, kColorRed);
  DrawKeyPointLine(frm, keypoints, right_wrist, right_middle_tip, kColorRed);
  DrawKeyPointLine(frm, keypoints, right_wrist, right_index_knuckle, kColorRed);
  DrawKeyPointLine(frm, keypoints, right_wrist, right_thumb_tip, kColorRed);
  DrawKeyPointLine(frm, keypoints, right_ankle, right_heel, kColorRed);
  DrawKeyPointLine(frm, keypoints, right_ankle, right_big_toe, kColorRed);
  DrawKeyPointLine(frm, keypoints, right_big_toe, right_small_toe, kColorRed);

  //left side
  DrawKeyPointLine(frm, keypoints, left_ankle, left_knee, kColorBlue);
  DrawKeyPointLine(frm, keypoints, left_knee, left_hip, kColorBlue);
  DrawKeyPointLine(frm, keypoints, left_hip, pelvis, kColorBlue);
  DrawKeyPointLine(frm, keypoints, left_hip, left_shoulder, kColorBlue);
  DrawKeyPointLine(frm, keypoints, left_shoulder, left_elbow, kColorBlue);
  DrawKeyPointLine(frm, keypoints, left_elbow, left_wrist, kColorBlue);
  DrawKeyPointLine(frm, keypoints, left_shoulder, neck, kColorBlue);

  // left side hand and feet
  DrawKeyPointLine(frm, keypoints, left_wrist, left_pinky_knuckle, kColorBlue);
  DrawKeyPointLine(frm, keypoints, left_wrist, left_middle_tip, kColorBlue);
  DrawKeyPointLine(frm, keypoints, left_wrist, left_index_knuckle, kColorBlue);
  DrawKeyPointLine(frm, keypoints, left_wrist, left_thumb_tip, kColorBlue);
  DrawKeyPointLine(frm, keypoints, left_ankle, left_heel, kColorBlue);
  DrawKeyPointLine(frm, keypoints, left_ankle, left_big_toe, kColorBlue);
  DrawKeyPointLine(frm, keypoints, left_big_toe, left_small_toe, kColorBlue);

  // head
  DrawKeyPointLine(frm, keypoints, neck, nose, kColorGreen);
  DrawKeyPointLine(frm, keypoints, nose, right_eye, kColorGreen);
  DrawKeyPointLine(frm, keypoints, right_eye, right_ear, kColorGreen);
  DrawKeyPointLine(frm, keypoints, nose, left_eye, kColorGreen);
  DrawKeyPointLine(frm, keypoints, left_eye, left_ear, kColorGreen);

  if (writeOutFiles) keyPointsOutputVideo.write(frm);
}

DoApp::Err DoApp::acquireFrame() {
  Err err = errNone;

  // If the machine goes to sleep with the app running and then wakes up, the camera object is not destroyed but the
  // frames we try to read are empty. So we try to re-initialize the camera with the same resolution settings. If the
  // resolution has changed, you will need to destroy and create the features again with the new camera resolution (not
  // done here) as well as reallocate memory accordingly with BodyEngine::initFeatureIOParams()
  cap >> frame;  // get a new frame from camera into the class variable frame.
  if (frame.empty()) {
    // if in Offline mode, this means end of video,so we return
    if (offlineMode) return errVideo;
    // try Init one more time if reading frames from camera
    err = initCamera(camRes.c_str());
    if (err != errNone)
      return err;
    cap >> frame;
    if (frame.empty()) return errVideo;
  }

  return err;
}

DoApp::Err DoApp::acquireBodyBox() {
  Err err = errNone;
  NvAR_Rect output_bbox;

  // get keypoints in  original image resolution coordinate space
  unsigned n = body_ar_engine.acquireBodyBox(frame, output_bbox, 0);

  if (n && verbose) {
    printf("BodyBox: [\n");
    printf("%7.1f%7.1f%7.1f%7.1f\n", output_bbox.x, output_bbox.y, output_bbox.x + output_bbox.width,
           output_bbox.y + output_bbox.height);
    printf("]\n");
  }
  if (captureOutputs) {
    writeFrameAndEstResults(frame, body_ar_engine.output_bboxes);
    writeVideoAndEstResults(frame, body_ar_engine.output_bboxes);
  }
  if (0 == n) return errNoBody;

#ifdef VISUALIZE

  if (drawVisualization) {
    DrawBBoxes(frame, &output_bbox);
  }
#endif  // VISUALIZE
  frameIndex++;

  return err;
}

DoApp::Err DoApp::acquireBodyBoxAndKeyPoints() {
  Err err = errNone;
  int numKeyPoints = body_ar_engine.getNumKeyPoints();
  NvAR_Rect output_bbox;
  std::vector<NvAR_Point2f> keypoints2D(numKeyPoints);
  std::vector<NvAR_Point3f> keypoints3D(numKeyPoints);
  std::vector<NvAR_Quaternion> jointAngles(numKeyPoints);

#ifdef DEBUG_PERF_RUNTIME
  auto start = std::chrono::high_resolution_clock::now();
#endif

  // get keypoints in original image resolution coordinate space
  unsigned n = body_ar_engine.acquireBodyBoxAndKeyPoints(frame, keypoints2D.data(), keypoints3D.data(),
                                                         jointAngles.data(), output_bbox, 0);

#ifdef DEBUG_PERF_RUNTIME
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  std::cout << "box+keypoints time: " << duration.count() << " microseconds" << std::endl;
#endif

  if (n && verbose && body_ar_engine.appMode != BodyEngine::mode::bodyDetection) {
    // printf("KeyPoints: [\n");
    // for (const auto &pt : keypoints2D) {
    //   printf("%7.1f%7.1f\n", pt.x, pt.y);
    // }
    // printf("]\n");

    printf("3d KeyPoints: [\n");
    for (const auto& pt : keypoints3D) {
        printf("%7.1f%7.1f%7.1f  ", pt.x, pt.y, pt.z);
    }
    printf("]\n");
  }
  if (captureOutputs) {
    writeFrameAndEstResults(frame, body_ar_engine.output_bboxes, keypoints2D.data());
    writeVideoAndEstResults(frame, body_ar_engine.output_bboxes, keypoints2D.data());
  }
  if (0 == n) return errNoBody;

#ifdef VISUALIZE

  if (drawVisualization) {
    DrawKeyPointsAndEdges(frame, keypoints2D.data(), numKeyPoints, &output_bbox);
    if (showBBox) {
      DrawBBoxes(frame, &output_bbox);
    }
  }
#endif  // VISUALIZE
  frameIndex++;

  return err;
}

DoApp::Err DoApp::initCamera(const char *camRes) {
  if (cap.open(camindex)) {
    if (camRes) {
      int n;
      n = sscanf(camRes, "%d%*[xX]%d", &inputWidth, &inputHeight);
      switch (n) {
        case 2:
          break;  // We have read both width and height
        case 1:
          inputHeight = inputWidth;
          inputWidth = (int)(inputHeight * (4. / 3.) + .5);
          break;
        default:
          inputHeight = 0;
          inputWidth = 0;
          break;
      }
      if (inputWidth) cap.set(CV_CAP_PROP_FRAME_WIDTH, inputWidth);
      if (inputHeight) cap.set(CV_CAP_PROP_FRAME_HEIGHT, inputHeight);

      inputWidth = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
      inputHeight = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);
      body_ar_engine.setInputImageWidth(inputWidth);
      body_ar_engine.setInputImageHeight(inputHeight);
    }
  } else
    return errCamera;
  return errNone;
}

DoApp::Err DoApp::initOfflineMode(const char *inputFilename, const char *outputFilename) {
  if (cap.open(inputFilename)) {
    inputWidth = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
    inputHeight = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    body_ar_engine.setInputImageWidth(inputWidth);
    body_ar_engine.setInputImageHeight(inputHeight);
  } else {
    printf("ERROR: Unable to open the input video file \"%s\" \n", inputFilename);
    return Err::errVideo;
  }

  if (writeOutFiles) {
    std::string bdOutputVideoName, jdOutputVideoName;
    std::string outputFilePrefix;
    if (outputFilename && strlen(outputFilename) != 0) {
      outputFilePrefix = outputFilename;
    } else {
      size_t lastindex = std::string(inputFilename).find_last_of(".");
      outputFilePrefix = std::string(inputFilename).substr(0, lastindex);
    }
    bdOutputVideoName = outputFilePrefix + "_bbox.mp4";
    jdOutputVideoName = outputFilePrefix + "_pose.mp4";

    if (!bodyDetectOutputVideo.open(bdOutputVideoName, StringToFourcc(captureCodec), cap.get(CV_CAP_PROP_FPS),
                                    cv::Size(inputWidth, inputHeight))) {
      printf("ERROR: Unable to open the output video file \"%s\" \n", bdOutputVideoName.c_str());
      return Err::errGeneral;
    }
    if (!keyPointsOutputVideo.open(jdOutputVideoName, StringToFourcc(captureCodec), cap.get(CV_CAP_PROP_FPS),
        cv::Size(inputWidth, inputHeight))) {
        printf("ERROR: Unable to open the output video file \"%s\" \n", bdOutputVideoName.c_str());
        return Err::errGeneral;
    }
  }
  return Err::errNone;
}

DoApp::DoApp() {
  // Make sure things are initialized properly
  gApp = this;
  drawVisualization = true;
  showFPS = true;
  showBBox = false;
  captureVideo = false;
  captureFrame = false;
  frameTime = 0;
  frameIndex = 0;
  nvErr = BodyEngine::errNone;
  scaleOffsetXY[0] = scaleOffsetXY[2] = 1.f;
  scaleOffsetXY[1] = scaleOffsetXY[3] = 0.f;

  // SETTINGS
  debug = false; 
  verbose = false; 
  temporal = true; 
  captureOutputs = false;
  offlineMode = false;
  writeOutFiles = false;
  useCudaGraph = true;
  captureCodec = "avc1",
  appMode = 1;
  mode = 1; 
  camindex = 0;
}


int chooseGPU() {
  // If the system has multiple supported GPUs then the application
  // should use CUDA driver APIs or CUDA runtime APIs to enumerate
  // the GPUs and select one based on the application's requirements

  //Cuda device 0
  return 0;

}

void DoApp::getFPS() {
  const float timeConstant = 16.f;
  frameTimer.stop();
  float t = (float)frameTimer.elapsedTimeFloat();
  if (t < 100.f) {
    if (frameTime)
      frameTime += (t - frameTime) * (1.f / timeConstant);  // 1 pole IIR filter
    else
      frameTime = t;
  } else {            // Ludicrous time interval; reset
    frameTime = 0.f;  // WAKE UP
  }
  frameTimer.start();
}

void DoApp::drawFPS(cv::Mat &img) {
  getFPS();
  if (frameTime && showFPS) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.1f", 1. / frameTime);
    cv::putText(img, buf, cv::Point(img.cols - 80, img.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 1,
                cv::Scalar(255, 255, 255), 1);
  }
}

void DoApp::drawKalmanStatus(cv::Mat &img) {
  char buf[32];
  snprintf(buf, sizeof(buf), "Stabilization: %s", (body_ar_engine.bStabilizeBody ? "on" : "off"));
  cv::putText(img, buf, cv::Point(10, img.rows - 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1);
}

void DoApp::drawVideoCaptureStatus(cv::Mat &img) {
  char buf[32];
  snprintf(buf, sizeof(buf), "Video Capturing %s", (captureVideo ? "on" : "off"));
  cv::putText(img, buf, cv::Point(10, img.rows - 70), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1);
}

DoApp::Err DoApp::run() {
  DoApp::Err doErr = errNone;

  BodyEngine::Err err = body_ar_engine.initFeatureIOParams();
  if (err != BodyEngine::Err::errNone ) {
    return doAppErr(err);
  }
  while (1) {
    //printf(">> frame %d \n", framenum++);
    doErr = acquireFrame();
    if (frame.empty() && offlineMode) {
      // We have reached the end of the video
      // so return without any error.
      return DoApp::errNone;
    }
    else if (doErr != DoApp::errNone) {
      return doErr;
    }
    if (body_ar_engine.appMode == BodyEngine::mode::bodyDetection) {
      doErr = acquireBodyBox();
    } else if (body_ar_engine.appMode == BodyEngine::mode::keyPointDetection) {
      doErr = acquireBodyBoxAndKeyPoints();
    }
    if ((DoApp::errNoBody == doErr || DoApp::errBodyFit == doErr) && offlineMode) {
      if (writeOutFiles) {
        bodyDetectOutputVideo.write(frame);
        keyPointsOutputVideo.write(frame);
      }
    }
    if (DoApp::errCancel == doErr || DoApp::errVideo == doErr) return doErr;
    if (!frame.empty() && !offlineMode) {
      if (drawVisualization) {
        drawFPS(frame);
        drawKalmanStatus(frame);
        if (captureOutputs && captureVideo) drawVideoCaptureStatus(frame);
      }
      cv::imshow(windowTitle, frame);
    }

    if (!offlineMode) {
      int n = cv::waitKey(1);
      if (n >= 0) {
        static const int ESC_KEY = 27;
        if (n == ESC_KEY) break;
        processKey(n);
      }
    }
  }
  return doErr;
}

const char *DoApp::errorStringFromCode(DoApp::Err code) {
  struct LUTEntry {
    Err code;
    const char *str;
  };
  static const LUTEntry lut[] = {
      {errNone, "no error"},
      {errGeneral, "an error has occured"},
      {errRun, "an error has occured while the feature is running"},
      {errInitialization, "Initializing Body Engine failed"},
      {errRead, "an error has occured while reading a file"},
      {errEffect, "an error has occured while creating a feature"},
      {errParameter, "an error has occured while setting a parameter for a feature"},
      {errUnimplemented, "the feature is unimplemented"},
      {errMissing, "missing input parameter"},
      {errVideo, "no video source has been found"},
      {errImageSize, "the image size cannot be accommodated"},
      {errNotFound, "the item cannot be found"},
      {errBodyModelInit, "body model initialization failed"},
      {errGLFWInit, "GLFW initialization failed"},
      {errGLInit, "OpenGL initialization failed"},
      {errRendererInit, "renderer initialization failed"},
      {errGLResource, "an OpenGL resource could not be found"},
      {errGLGeneric, "an otherwise unspecified OpenGL error has occurred"},
      {errBodyFit, "an error has occurred while body fitting"},
      {errNoBody, "no body has been found"},
      {errSDK, "an SDK error has occurred"},
      {errCuda, "a CUDA error has occurred"},
      {errCancel, "the user cancelled"},
      {errCamera, "unable to connect to the camera"},
  };
  for (const LUTEntry *p = lut; p < &lut[sizeof(lut) / sizeof(lut[0])]; ++p)
    if (p->code == code) return p->str;
  static char msg[18];
  snprintf(msg, sizeof(msg), "error #%d", code);
  return msg;
}

