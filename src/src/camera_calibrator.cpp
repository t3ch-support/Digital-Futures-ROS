
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <ctime>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <raspicam/raspicam_cv.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Calibration Image";

/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

/**
 */
static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags,
                             const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if(flags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if(flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}

void setCameraParams (raspicam::RaspiCam_Cv &Camera ) {
    Camera.set ( cv::CAP_PROP_FRAME_WIDTH,  640);
    Camera.set ( cv::CAP_PROP_FRAME_HEIGHT, 480);
    Camera.set ( cv::CAP_PROP_BRIGHTNESS, 50);
    Camera.set ( cv::CAP_PROP_CONTRAST, 50);
    Camera.set ( cv::CAP_PROP_SATURATION, 50);
    Camera.set ( cv::CAP_PROP_GAIN, 50);
    Camera.set ( cv::CAP_PROP_FPS, 0);
    Camera.set ( cv::CAP_PROP_FORMAT, CV_8UC1 );
}


class CameraCalibrator
{
    ros::NodeHandle nh_;
    // image_transport::ImageTransport it_;
    // image_transport::Subscriber image_sub_;
    raspicam::RaspiCam_Cv Camera;

    float board_square_length;
    float board_marker_length;
    int board_squaresX;
    int board_squaresY;
    int dictionary_id;
    std::string out;

    bool showChessboardCorners = true;
    int calibrationFlags = 0;
    float aspectRatio = 1;
    bool refindStrategy = true;

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::Board> board;
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard;

    vector< vector< vector< Point2f > > > allCorners;
    vector< vector< int > > allIds;
    vector< Mat > allImgs;
    Size imgSize;

  public:
    CameraCalibrator()
    {
        ros::NodeHandle _nh("~");
        // image_sub_ = it_.subscribe("/webcam/image_raw", 1, &CameraCalibrator::convertImage, this);
        setCameraParams(Camera);
        if ( !Camera.open() ) {
            ROS_ERROR("Error opening camera");
        }else{
        ROS_INFO_STREAM("Connected to camera =" << Camera.getId());
        }

        cv::namedWindow(OPENCV_WINDOW);

        _nh.getParam("board_marker_length", board_marker_length);
        ROS_INFO_STREAM("Marker Length: " << board_marker_length);

        _nh.getParam("board_square_length", board_square_length);
        ROS_INFO_STREAM("Square Length: " << board_square_length);

        _nh.getParam("board_squaresX", board_squaresX);
        ROS_INFO_STREAM("Squares X: " << board_squaresX);

        _nh.getParam("board_squaresY", board_squaresY);
        ROS_INFO_STREAM("Squares Y: " << board_squaresY);

        _nh.getParam("dictionary_id", dictionary_id);
        ROS_INFO_STREAM("Dictionary ID: " << dictionary_id);

        _nh.getParam("output_path", out);
        ROS_INFO_STREAM("Output Path: " << out);

        detectorParams = cv::aruco::DetectorParameters::create();

        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));
        charucoboard = cv::aruco::CharucoBoard::create(board_squaresX, board_squaresY, board_square_length, board_marker_length, dictionary);
        board = charucoboard.staticCast<cv::aruco::Board>();
        calibrateCamera();
    }

    ~CameraCalibrator()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }

    void calibrateCamera(){
        bool configDone = false;
        while(Camera.grab() && !configDone){
            Mat image, imageCopy;
            Camera.retrieve(image);
            vector< int > ids;
            vector< vector< cv::Point2f > > corners, rejected;

            // detect markers
            cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
            
            // refind strategy to detect more markers
            if(refindStrategy) cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

            // interpolate charuco corners
            Mat currentCharucoCorners, currentCharucoIds;
            if(ids.size() > 0)
                aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners,
                                                currentCharucoIds);

            // draw results
            if(ids.size() > 0){
                cv::aruco::drawDetectedMarkers(image, corners);
            }

            if(currentCharucoCorners.total() > 0) {
                cv::aruco::drawDetectedCornersCharuco(image, currentCharucoCorners, currentCharucoIds);
            }

            cv::putText(image, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                    Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

            cv::imshow(OPENCV_WINDOW, image);
            char key = (char)cv::waitKey(10);
            if(key == 'c' && ids.size() > 0) {
                ROS_INFO_STREAM("Frame Captured");
                allCorners.push_back(corners);
                allIds.push_back(ids);
                allImgs.push_back(image);
                imgSize = image.size();
            }
            
            if(key == 27){
                //finish calibration  
                ROS_INFO_STREAM("Generating Configuration File");
                if(allIds.size() < 1) {
                    ROS_ERROR("Not enough captures for calibration");
                    return; 
                }else{
                    Mat cameraMatrix, distCoeffs;
                    vector< Mat > rvecs, tvecs;
                    double repError;

                    if(calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
                        cameraMatrix = Mat::eye(3, 3, CV_64F);
                        cameraMatrix.at< double >(0, 0) = aspectRatio;
                    }

                    // prepare data for calibration
                    vector< vector< Point2f > > allCornersConcatenated;
                    vector< int > allIdsConcatenated;
                    vector< int > markerCounterPerFrame;
                    markerCounterPerFrame.reserve(allCorners.size());
                    for(unsigned int i = 0; i < allCorners.size(); i++) {
                        markerCounterPerFrame.push_back((int)allCorners[i].size());
                        for(unsigned int j = 0; j < allCorners[i].size(); j++) {
                            allCornersConcatenated.push_back(allCorners[i][j]);
                            allIdsConcatenated.push_back(allIds[i][j]);
                        }
                    }

                    // calibrate camera using aruco markers
                    double arucoRepErr;
                    arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                                            markerCounterPerFrame, board, imgSize, cameraMatrix,
                                                            distCoeffs, noArray(), noArray(), calibrationFlags);

                    // prepare data for charuco calibration
                    int nFrames = (int)allCorners.size();
                    vector< Mat > allCharucoCorners;
                    vector< Mat > allCharucoIds;
                    vector< Mat > filteredImages;
                    allCharucoCorners.reserve(nFrames);
                    allCharucoIds.reserve(nFrames);

                    for(int i = 0; i < nFrames; i++) {
                        // interpolate using camera parameters
                        Mat currentCharucoCorners, currentCharucoIds;
                        aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard,
                                                        currentCharucoCorners, currentCharucoIds, cameraMatrix,
                                                        distCoeffs);

                        allCharucoCorners.push_back(currentCharucoCorners);
                        allCharucoIds.push_back(currentCharucoIds);
                        filteredImages.push_back(allImgs[i]);
                    }

                    if(allCharucoCorners.size() < 4) {
                        cerr << "Not enough corners for calibration" << endl;
                        
                    }

                    // calibrate camera using charuco
                    repError =
                        aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
                                                    cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

                    bool saveOk =  saveCameraParams(out, imgSize, aspectRatio, calibrationFlags,
                                                    cameraMatrix, distCoeffs, repError);
                    if(!saveOk) {
                        cerr << "Cannot save output file" << endl;
                        
                    }else{
                        ROS_INFO_STREAM("Saved configuration file");
                    }

                    cout << "Rep Error: " << repError << endl;
                    cout << "Rep Error Aruco: " << arucoRepErr << endl;
                    cout << "Calibration saved to " << out << endl;

                    // show interpolated charuco corners for debugging
                    if(showChessboardCorners) {
                        for(unsigned int frame = 0; frame < filteredImages.size(); frame++) {
                            Mat imageCopy = filteredImages[frame].clone();
                            if(allIds[frame].size() > 0) {

                                if(allCharucoCorners[frame].total() > 0) {
                                    aruco::drawDetectedCornersCharuco( imageCopy, allCharucoCorners[frame],
                                                                    allCharucoIds[frame]);
                                }
                            }

                            imshow("out", imageCopy);
                            char key = (char)waitKey(0);
                            if(key == 27) break;
                        }
                    }
                }
                configDone = true;
            }
        }

        ros::shutdown();
    } 
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_calibrator");
  CameraCalibrator ic;
  ros::spin();
  return 0;
}