#include <iostream>
#include <queue>
#include <ctime>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"

#include <maya/MVector.h>
#include <maya/MPoint.h>
#include <maya/MMatrix.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>
#include <maya/MTransformationMatrix.h>

#include "nvAR.h"

#include "mayaConnection.h"
#include "BodyTracking.h"



const char DoApp::windowTitle[] = "BodyTracking App";


/********************************************************************************
 * Command-line arguments
 ********************************************************************************/

bool FLAG_debug = false, FLAG_verbose = false, FLAG_temporal = true, 
     FLAG_captureOutputs = true, FLAG_offlineMode = false, FLAG_useCudaGraph = true, 
     FLAG_writeOutFiles=false;
std::string FLAG_outDir, FLAG_inFile, FLAG_outFile, 
            FLAG_modelPath, FLAG_captureCodec = "avc1",
            FLAG_camRes, FLAG_bodyModel;
unsigned int FLAG_appMode = 1, FLAG_mode = 0, FLAG_camindex=0;

/********************************************************************************
 * Usage
 ********************************************************************************/

static void Usage() {
  printf(
      "BodyTrack [<args> ...]\n"
      "where <args> is\n"
      " --verbose[=(true|false)]          report interesting info\n"
      " --debug[=(true|false)]            report debugging info\n"
      " --temporal[=(true|false)]         temporally optimize body rect and keypoints\n"
      " --use_cuda_graph[=(true|false)]   enable faster execution by using cuda graph to capture engine execution\n"
      " --capture_outputs[=(true|false)]  enables video/image capture and writing body detection/keypoints outputs\n"
      " --offline_mode[=(true|false)]     disables webcam, reads video from file and writes output video results\n"
      " --cam_res=[WWWx]HHH               specify resolution as height or width x height\n"
      " --in_file=<file>                  specify the  input file\n"
      " --codec=<fourcc>                  FOURCC code for the desired codec (default H264)\n"
      " --in=<file>                       specify the  input file\n"
      " --out_file=<file>                 specify the output file\n"
      " --out=<file>                      specify the output file\n"
      " --model_path=<path>               specify the directory containing the TRT models\n"
      " --mode[=0|1]                      Model Mode. 0: High Quality, 1: High Performance\n"
      " --app_mode[=(0|1)]                App mode. 0: Body detection, 1: Keypoint detection "
      "(Default).\n"
      " --benchmarks[=<pattern>]          run benchmarks\n");
}

static bool GetFlagArgVal(const char *flag, const char *arg, const char **val) {
  if (*arg != '-') {
    return false;
  }
  while (*++arg == '-') {
    continue;
  }
  const char *s = strchr(arg, '=');
  if (s == NULL) {
    if (strcmp(flag, arg) != 0) {
      return false;
    }
    *val = NULL;
    return true;
  }
  unsigned n = (unsigned)(s - arg);
  if ((strlen(flag) != n) || (strncmp(flag, arg, n) != 0)) {
    return false;
  }
  *val = s + 1;
  return true;
}

static bool GetFlagArgVal(const char *flag, const char *arg, std::string *val) {
  const char *valStr;
  if (!GetFlagArgVal(flag, arg, &valStr)) return false;
  val->assign(valStr ? valStr : "");
  return true;
}

static bool GetFlagArgVal(const char *flag, const char *arg, bool *val) {
  const char *valStr;
  bool success = GetFlagArgVal(flag, arg, &valStr);
  if (success) {
    *val = (valStr == NULL || strcasecmp(valStr, "true") == 0 || strcasecmp(valStr, "on") == 0 ||
            strcasecmp(valStr, "yes") == 0 || strcasecmp(valStr, "1") == 0);
  }
  return success;
}

bool GetFlagArgVal(const char *flag, const char *arg, long *val) {
  const char *valStr;
  bool success = GetFlagArgVal(flag, arg, &valStr);
  if (success) {
    *val = strtol(valStr, NULL, 10);
  }
  return success;
}

static bool GetFlagArgVal(const char *flag, const char *arg, unsigned *val) {
  long longVal;
  bool success = GetFlagArgVal(flag, arg, &longVal);
  if (success) {
    *val = (unsigned)longVal;
  }
  return success;
}

/********************************************************************************
 * ParseMyArgs
 ********************************************************************************/

static int ParseMyArgs(int argc, char **argv) {
  int errs = 0;
  for (--argc, ++argv; argc--; ++argv) {
    bool help;
    const char *arg = *argv;
    if (arg[0] != '-') {
      continue;
    } else if ((arg[1] == '-') &&
               (GetFlagArgVal("verbose", arg, &FLAG_verbose) || GetFlagArgVal("debug", arg, &FLAG_debug) ||
                GetFlagArgVal("in", arg, &FLAG_inFile) || GetFlagArgVal("in_file", arg, &FLAG_inFile) ||
                GetFlagArgVal("out", arg, &FLAG_outFile) || GetFlagArgVal("out_file", arg, &FLAG_outFile) ||
                GetFlagArgVal("offline_mode", arg, &FLAG_offlineMode) ||
                GetFlagArgVal("write_out_files", arg, &FLAG_writeOutFiles) ||
                GetFlagArgVal("capture_outputs", arg, &FLAG_captureOutputs) ||
                GetFlagArgVal("cam_res", arg, &FLAG_camRes) || GetFlagArgVal("codec", arg, &FLAG_captureCodec) ||
                GetFlagArgVal("model_path", arg, &FLAG_modelPath) ||
                GetFlagArgVal("app_mode", arg, &FLAG_appMode) ||
                GetFlagArgVal("mode", arg, &FLAG_mode) ||
                GetFlagArgVal("camindex", arg, &FLAG_camindex) ||
                GetFlagArgVal("use_cuda_graph", arg, &FLAG_useCudaGraph) ||
                GetFlagArgVal("temporal", arg, &FLAG_temporal))) {
      continue;
    } else if (GetFlagArgVal("help", arg, &help)) {
      Usage();
    } else if (arg[1] != '-') {
      for (++arg; *arg; ++arg) {
        if (*arg == 'v') {
          FLAG_verbose = true;
        } else {
          // printf("Unknown flag: \"-%c\"\n", *arg);
        }
      }
      continue;
    } else {
      // printf("Unknown flag: \"%s\"\n", arg);
    }
  }
  return errs;
}


std::string getMayaXformCmd(const std::vector<NvAR_Point3f>& jointPositions, const std::vector<NvAR_Quaternion>& jointRotations, bool rotations=true)
{
    std::string mayaCmd = "";
    
    // calibration matrix, makes the char smaller and rotated Y up
    MTransformationMatrix calibrationTransform;
    MQuaternion fitRot;
    // rotate 180 on Z and scale points down by 10
    fitRot.setAxisAngle(MVector(0.0, 0.0, 1.0), M_PI);
    double scaleFactor[] = {0.1, 0.1, 0.1};
    calibrationTransform.setScale(scaleFactor, MSpace::kWorld);
    calibrationTransform.setRotationQuaternion(fitRot.x, fitRot.y, fitRot.z, fitRot.w);

    uint jCount = 0;
    std::string jntName;
    NvAR_Point3f keypoint;
    for ( uint i = 0; i < jointPositions.size(); ++i )
    {
        // joint positions
        keypoint = jointPositions[i];
        MPoint point( keypoint.x, keypoint.y, keypoint.z);
        point *= calibrationTransform.asMatrix();

        jntName = "joint" + std::to_string(jCount);

        // xform -ws -t 0.0 0.0 0.0 joint0;
        mayaCmd += "xform -ws -t ";
        mayaCmd += std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + " ";
        mayaCmd += jntName;
        mayaCmd += ";\n";

        // joint rotation
        if (rotations)
        {   
            // when calibrating, rotate the only the root joint to the calibration matrix as all other rotations are local
            MQuaternion q(jointRotations[i].x, jointRotations[i].y, jointRotations[i].z, jointRotations[i].w);
            if ( i == 0 ){
              q = q.asMatrix() * calibrationTransform.asMatrix();
            }
            MEulerRotation jRot = q.asEulerRotation();
            
            // xform -ro 0.0 0.0 0.0 joint0;
            mayaCmd += "xform -ro ";
            mayaCmd += std::to_string(jRot.x * (180.0/M_PI)) + " ";
            mayaCmd += std::to_string(jRot.y * (180.0/M_PI)) + " ";
            mayaCmd += std::to_string(jRot.z * (180.0/M_PI)) + " ";
            mayaCmd += jntName;
            mayaCmd += ";\n";
        }

        jCount++;
    }
    
    // printf("\n");
    // printf(mayaCmd.c_str());
    return mayaCmd;
}


std::string getMayaXformCmd2D(const std::vector<NvAR_Point2f>& keypoints2D)
{
    std::string mayaCmd = "";
    uint jCount = 0;
    for (const auto& pt : keypoints2D)
    {
        // printf("%7.1f%7.1f\n", pt.x, pt.y);
        // xform -ws -t 0 100 0 "joint3"
        mayaCmd += "xform -ws -t ";
        mayaCmd += std::to_string(pt.x) + " " + std::to_string(pt.y) + " 0 ";
        mayaCmd += "joint" + std::to_string(jCount);
        mayaCmd += ";\n";
        jCount++;
    }
    return mayaCmd;
}


std::string getMayaKeyframeCmd()
{
    std::string mayaCmd = "setKeyframe ";;

    std::string jntName;
    uint jCount = 0;
    std::vector<std::string> keyChannels = {"tx", "ty", "tz", "rx", "ry", "rz"};
    for ( uint i = 0; i < 34; ++i )
    {        
        jntName = "\"joint" + std::to_string(jCount);
        // set keyframes
        for ( auto channelName : keyChannels ){
            mayaCmd += jntName;
            mayaCmd += ".";
            mayaCmd += channelName;
            mayaCmd += "\" ";
        }
        jCount++;
    }
    mayaCmd += ";\n";

    return mayaCmd;
}



int main(int argc, char **argv) 
{
    MayaConnection mayaConnection;
    mayaConnection.connect();
    if (!mayaConnection.isConnected())
    {
        printf("Couldn't connect with Maya!  Press any key to exit...");
        std::getchar();
        return 0;
    }

    // Parse the arguments
    if (0 != ParseMyArgs(argc, argv)) return -100;

    DoApp app;
    DoApp::Err doErr = DoApp::Err::errNone;

    // settings
    app.debug = FLAG_debug;
    app.verbose = FLAG_verbose;
    app.temporal = FLAG_temporal;
    app.captureOutputs = FLAG_captureOutputs;
    app.offlineMode = FLAG_offlineMode;
    app.writeOutFiles = FLAG_writeOutFiles;
    app.useCudaGraph = FLAG_useCudaGraph;
    app.outDir = FLAG_outDir;
    app.inFile = FLAG_inFile;
    app.outFile = FLAG_outFile;
    app.modelPath = FLAG_modelPath;
    app.captureCodec = FLAG_captureCodec;
    app.camRes = FLAG_camRes;
    app.bodyModel = FLAG_bodyModel;
    app.appMode = FLAG_appMode;
    app.mode = FLAG_mode;
    app.camindex = FLAG_camindex;

    printf("----------SETTINGS----------\n");
    printf("verbose: %d\n", app.verbose);
    printf("temporal: %d\n", app.temporal);
    printf("captureOutputs: %d\n", app.captureOutputs);
    printf("offlineMode: %d\n", app.offlineMode);
    printf("writeOutFiles: %d\n", app.writeOutFiles);
    printf("useCudaGraph: %d\n", app.useCudaGraph);
    printf("outDir: %s\n", app.outDir.c_str());
    printf("inFile: %s\n", app.inFile.c_str());
    printf("outFile: %s\n", app.outFile.c_str());
    printf("modelPath: %s\n", app.modelPath.c_str());
    printf("captureCodec: %s\n", app.captureCodec.c_str());
    printf("camRes: %s\n", app.camRes.c_str());
    printf("bodyModel: %s\n", app.bodyModel.c_str());
    printf("appMode: %d\n", app.appMode);
    printf("mode: %d\n", app.mode);
    printf("camindex: %d\n", app.camindex);
    printf("-----------------------------\n");

    app.body_ar_engine.setAppMode(BodyEngine::mode(FLAG_appMode));

    app.body_ar_engine.setMode(FLAG_mode);

    if (FLAG_verbose) printf("Enable temporal optimizations in detecting body and keypoints = %d\n", FLAG_temporal);
    app.body_ar_engine.setBodyStabilization(FLAG_temporal);

    if (FLAG_useCudaGraph) printf("Enable capturing cuda graph = %d\n", FLAG_useCudaGraph);
    app.body_ar_engine.useCudaGraph(FLAG_useCudaGraph);

    doErr = DoApp::errBodyModelInit;
    if (FLAG_modelPath.empty()) 
    {
        printf("WARNING: Model path not specified. Please set --model_path=/path/to/trt/and/body/models, "
        "SDK will attempt to load the models from NVAR_MODEL_DIR environment variable, "
        "please restart your application after the SDK Installation. \n");
    }
    if (!FLAG_bodyModel.empty())
        app.body_ar_engine.setBodyModel(FLAG_bodyModel.c_str());

    if (FLAG_offlineMode) 
    {
        if (FLAG_inFile.empty()) 
        {
            doErr = DoApp::errMissing;
            printf("ERROR: %s, please specify input file using --in_file or --in \n", app.errorStringFromCode(doErr));
            goto bail;
        }
        doErr = app.initOfflineMode(FLAG_inFile.c_str(), FLAG_outFile.c_str());
    } 
    else 
    {
        doErr = app.initCamera(FLAG_camRes.c_str());
    }
    BAIL_IF_ERR(doErr);

    doErr = app.initBodyEngine(FLAG_modelPath.c_str());
    BAIL_IF_ERR(doErr);

    /* 
    // doErr = app.run();
    */
    BodyEngine::Err err = app.body_ar_engine.initFeatureIOParams();
    if (err != BodyEngine::Err::errNone ) {
        return app.doAppErr(err);
    }

    uint counter = 0;
    bool record = false;
    bool videoLoop = true;
    bool keyframe = false;
    uint currentFrameNum = 0;
    while (1) 
    {
        // get video frame
        doErr = app.acquireFrame();
        if (app.frame.empty() && app.offlineMode) 
        {
            // We have reached the end of the video
            
            // loop video
            if ( videoLoop )
            {
                app.cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                continue;
            }
            else  // return without any error.
            {
                return DoApp::errNone;
            } 
        }
        else if (doErr != DoApp::errNone) 
        {
            return doErr;
        }
        

        if (!app.frame.empty() && !app.writeOutFiles) 
        {
            if (app.drawVisualization) 
            {
                app.drawFPS(app.frame);
                app.drawKalmanStatus(app.frame);
                if (app.captureOutputs && app.captureVideo) app.drawVideoCaptureStatus(app.frame);
            }
        }

        // send to maya
        if (record) 
        {
            DoApp::Err err = DoApp::errNone;
            int numKeyPoints = app.body_ar_engine.getNumKeyPoints();
            NvAR_Rect output_bbox;
            std::vector<NvAR_Point2f> keypoints2D(numKeyPoints);
            std::vector<NvAR_Point3f> keypoints3D(numKeyPoints);
            std::vector<NvAR_Quaternion> jointAngles(numKeyPoints);

            // get keypoints in original image resolution coordinate space
            unsigned n = app.body_ar_engine.acquireBodyBoxAndKeyPoints(
                app.frame, 
                keypoints2D.data(), 
                keypoints3D.data(),
                jointAngles.data(), 
                output_bbox, 
                0
            );

            // draw keypoints on video
            app.DrawKeyPointsAndEdges(app.frame, keypoints2D.data(), numKeyPoints, &output_bbox);

            // std::string mayaCmd = getMayaCmdFrom2DData(keypoints2D);
            std::string mayaCmd = getMayaXformCmd(keypoints3D, jointAngles);
            if (keyframe)
            {   
                std::string setCurrentTimeCmd = "currentTime ";
                
                currentFrameNum = (uint)app.cap.get(cv::CAP_PROP_POS_FRAMES);
                setCurrentTimeCmd += std::to_string(currentFrameNum);
                setCurrentTimeCmd += ";\n";

                // add currentTime and setKeyframe cmds to the full command to send to maya
                mayaCmd.insert(0, setCurrentTimeCmd);
                mayaCmd += getMayaKeyframeCmd();
            }
            
            // send the command to maya
            // printf(mayaCmd.c_str());
            mayaConnection.send(mayaCmd);

            ++counter;
        }
    

        // display the image
        if (!app.frame.empty() && !app.writeOutFiles)
            cv::imshow(app.windowTitle, app.frame);

        // process KEYBOARD INPUT
        if (!app.writeOutFiles) 
        {   
            char key = cv::waitKey(1);
            if (key >= 0) 
            {
                // keyboard overrides
                if (key == 'r') 
                {
                    record = !record;
                    if(record)
                        std::cout << "recording ON" << std::endl;
                    else
                        std::cout << "recording OFF" << std::endl;
                }
                else if (key == 'k')
                {
                    keyframe = !keyframe;
                }
                else
                {
                    // Exit application
                    static const int ESC_KEY = 27;
                    if (key == ESC_KEY) break;

                    app.processKey(key);
                }
            }
        }
    }
    BAIL_IF_ERR(doErr);
    bail:
    if(doErr)
        printf("ERROR: %s\n", app.errorStringFromCode(doErr));
    app.stop();
    return (int)doErr;
}
