/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>
#include <fstream>

#include <zmq.h>

using namespace std;
using namespace cv;


/**
 */
static void help() {
    cout << "Detects a tag and sends its position via ZMQ push socket." << endl;
    cout << "Listen to 0.0.0.0:7777" << endl << endl;
    cout << "-conf <configurationFile> # Main configuration file to set mandatory options" << endl << endl;
    cout << "Parameters: " << endl;
    cout << "-d <dictionary> # DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2, "
         << "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
         << "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
         << "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16" << endl;
    cout << "[-v <videoFile>] # Input from video file, if ommited, input comes from camera" << endl;
    cout << "[-ci <int>] # Camera id if input doesnt come from video (-v). Default is 0" << endl;
    cout << "[-c <cameraParams>] # Camera intrinsic parameters. Needed for camera pose" << endl;
    cout << "[-l <markerLength>] # Marker side lenght (in meters). Needed for correct"
         << "scale in camera pose, default 0.1" << endl;
    cout << "[-dp <detectorParams>] # File of marker detector parameters" << endl;
    cout << "[-r] # show rejected candidates too" << endl;
    cout << "[-t] # Detect from top" << endl << endl;
    cout << "Example of configuration: " << endl;
    cout << "%%YAML:1.0" << endl;
    cout << "camera_name: \"HD Pro Webcam C920\"  # Get the name from \"v4l2-ctl --list-devices\"" << endl;
    cout << "camera_calib: \"c920-2.yml\"   # Path is relative to this file location" << endl << endl;

    cout << "# Marker to be tracked" << endl;
    cout << "marker_dictionary: 0" << endl;
    cout << "marker_id: 1" << endl;
    cout << "marker_length: 0.35" << endl << endl;

    cout << "# Camera position" << endl;
    cout << "camera_top: true" << endl;

}



/**
 */
static bool isParam(string param, int argc, char **argv) {
    for(int i = 0; i < argc; i++)
        if(string(argv[i]) == param) return true;
    return false;
}


/**
 */
static string getParam(string param, int argc, char **argv, string defvalue = "") {
    int idx = -1;
    for(int i = 0; i < argc && idx == -1; i++)
        if(string(argv[i]) == param) idx = i;
    if(idx == -1 || (idx + 1) >= argc)
        return defvalue;
    else
        return argv[idx + 1];
}



/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}



/**
 */
static bool readDetectorParameters(string filename, aruco::DetectorParameters &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params.adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params.adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params.adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params.adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params.minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params.maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params.polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params.minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params.minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params.minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params.doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params.cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params.cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params.cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params.markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params.perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params.perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params.maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params.minOtsuStdDev;
    fs["errorCorrectionRate"] >> params.errorCorrectionRate;
    return true;
}



/**
 */
int main(int argc, char *argv[]) {
    FileStorage conf = NULL;
    bool has_conf = false;
    int marker_id = 0;
    float cx, cy, cz;
    float gx, gy, gz;
    float frameTime = 0;

    float pos_x = 0, pos_y = 0, pos_z = 0;
    float angle = 0;

    bool fromTop = false;
    if (isParam("-t", argc, argv)) fromTop = true;

    // Initializing ZMQ
    void *ctx = zmq_ctx_new();
    void *controller = zmq_socket (ctx, ZMQ_PUSH);
    int rc = zmq_bind (controller, "tcp://*:7777");
    assert (rc == 0);

    if(!isParam("-d", argc, argv) && !isParam("-conf", argc, argv)) {
        help();
        return 0;
    }

    if (isParam("-conf", argc, argv)) {
      conf = FileStorage(getParam("-conf", argc, argv), FileStorage::READ);
      has_conf = true;

      fromTop = (int)conf["camera_top"] != 0;
    }

    int dictionaryId = 0;
    if (has_conf) {
      conf["marker_dictionary"] >> dictionaryId;
    } else {
      dictionaryId = atoi(getParam("-d", argc, argv).c_str());
    }
    aruco::Dictionary dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    bool showRejected = false;
    if(isParam("-r", argc, argv)) showRejected = true;

    bool estimatePose = false;
    Mat camMatrix, distCoeffs;
    if (has_conf) {
        String filename = conf["camera_calib"];
        string confPath = getParam("-conf", argc, argv);
        string fullPath = confPath.substr(0, confPath.find_last_of("/"));
        fullPath.append("/");
        fullPath.append(filename);

        bool readOk = readCameraParameters(fullPath, camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
        estimatePose = true;
    } else if(isParam("-c", argc, argv)) {
        bool readOk = readCameraParameters(getParam("-c", argc, argv), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
        estimatePose = true;
    }

    float markerLength = 0;
    if (has_conf) {
      markerLength = conf["marker_length"];
    } else {
      markerLength = (float)atof(getParam("-l", argc, argv, "0.1").c_str());
    }

    aruco::DetectorParameters detectorParams;
    if(isParam("-dp", argc, argv)) {
        bool readOk = readDetectorParameters(getParam("-dp", argc, argv), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }
    detectorParams.doCornerRefinement = true; // do corner refinement in markers

    VideoCapture inputVideo;
    int waitTime;
    if(isParam("-v", argc, argv)) {
        inputVideo.open(getParam("-v", argc, argv));
        waitTime = 0;
    } else {
        int camId = 0;
        if(isParam("-ci", argc, argv)) camId = atoi(getParam("-ci", argc, argv).c_str());
        inputVideo.open(camId);
        waitTime = 10;
    }

    ofstream logfile;
    bool haslog = false;
    if (isParam("-L", argc, argv)) {
      logfile.open(getParam("-L", argc, argv).c_str());
      logfile << "x, y, z, angle" << endl;
      haslog = true;
    }

    inputVideo.set(CV_CAP_PROP_FOURCC,CV_FOURCC('Y','C','Y','V'));
    inputVideo.set(CAP_PROP_FPS, 30);
    inputVideo.set(CAP_PROP_FRAME_WIDTH,640);
    inputVideo.set(CAP_PROP_FRAME_HEIGHT,480);

    double totalTime = 0;
    int totalIterations = 0;

    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Mat > rvecs, tvecs;

        // detect markers and estimate pose
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        if(estimatePose && ids.size() > 0) {
          aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
                                           tvecs);
        }

        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if(estimatePose) {
                for(unsigned int i = 0; i < ids.size(); i++)
                    aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                    markerLength * 0.5f);
            } else {
              cout << "Warning: camera calib not available, no pose calculation!" << endl;
            }
        }

        if(showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        float t = getTickCount();
        float fps = getTickFrequency()/(t-frameTime);
        frameTime = t;
        {
          std::ostringstream str;
          str << "Fps:" << fps;
          putText(imageCopy, str.str(), Point(10,30), CV_FONT_HERSHEY_PLAIN, 3, Scalar(0,0,250));
        }


        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;


        Mat ground = Mat::zeros(3, 1, CV_64F);
        Mat groundRot = Mat::zeros(3, 3, CV_64F);
        Mat copter = Mat::zeros(3, 1, CV_64F);

        int detect = 0;

        char buffer[255];
        // Assuming there is only one tag, sending its position via ZMQ
        if (ids.size()>0 && marker_id && ids[0] == marker_id && estimatePose) {

          angle = atan2((corners[0][0].y-corners[0][2].y), (corners[0][2].x-corners[0][0].x))*180/M_PI;
          angle += 45;

          pos_x = tvecs[0].at<double>(0);
          pos_y = tvecs[0].at<double>(1);
          pos_z = tvecs[0].at<double>(2);

          if (fromTop) {
            //pos_x = pos_x;
            pos_y = -pos_y;
            pos_z = -pos_z;
            angle = -angle;
          }

          printf("Detected marker %d: %f, %f, %f, %f\n", ids[0], pos_x, pos_y, pos_z, angle);
          sprintf(buffer, "{\"pos\": [%f, %f, %f], \"angle\": %f, \"detect\": true}", pos_x, pos_y, pos_z, angle);
          zmq_send(controller, buffer, strlen(buffer), ZMQ_DONTWAIT);

          if (haslog) {
            logfile << pos_x << ", " << pos_y << ", " << pos_z << ", " << angle << endl;
          }
        } else {
          sprintf(buffer, "{\"pos\": [%f, %f, %f], \"angle\": %f, \"detect\": false}", pos_x, pos_y, pos_z, angle);
          zmq_send(controller, buffer, strlen(buffer), ZMQ_DONTWAIT);
          if (haslog) {
            logfile << "0, 0, 0, 0" << endl;
          }
        }

        // Calulate difference between ground and copter
        for (int i=0; i<ids.size(); i++) {
          if (ids[i] == 0) {
            cx = tvecs[i].at<double>(0);
            cy = tvecs[i].at<double>(1);
            cz = tvecs[i].at<double>(2);
            printf("%f\t%f\t%f\n", cx, cy, cz);
            copter = tvecs[i].clone();
            detect ++;
          } else if (ids[i] == 1) {
            gx = tvecs[i].at<double>(0);
            gy = tvecs[i].at<double>(1);
            gz = tvecs[i].at<double>(2);
            printf("%f\t%f\t%f\n", gx, gy, gz);
            ground = tvecs[i].clone();
            Rodrigues(rvecs[i], groundRot);

            detect ++;
          }
        }

        if (detect >= 2) {
          Mat diff = copter - ground;
          cout << diff << endl;
          diff = groundRot * diff;
        }

    }

    if (haslog) {
      logfile.close();
    }

    return 0;
}
