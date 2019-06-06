#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

static const std::string OPENCV_WINDOW = "Processed Image";

using namespace std;


static bool readCameraParameters(string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

void getQuaternion(cv::Mat R, double Q[])
{
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    if (trace > 0.0) 
    {
        double s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
        Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
        Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
    } 
    else 
    {
        int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
        Q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
        Q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
    }
}

// class ImageProcessor
// {
//   ros::NodeHandle nh_;
//   image_transport::ImageTransport it_;
//   image_transport::Subscriber image_sub_;
//   image_transport::Publisher image_pub_;
//   ros::Publisher vis_markers_pub_;
//   std::string camera_calibration;
//   float markerLength;
//   cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
//   cv::Ptr<cv::aruco::Dictionary> dictionary;
//   cv::Mat camMatrix, distCoeffs;
  
//   bool charuco_board = false;
//   cv::Ptr<cv::aruco::Board> board;
//   cv::Ptr<cv::aruco::CharucoBoard> charucoboard;
//   float axisLength;

//   public:
//     ImageProcessor()
//       : it_(nh_)
//     {
//       ros::NodeHandle _nh("~");
//       image_sub_ = it_.subscribe("/webcam/image_raw", 1, &ImageProcessor::convertImage, this);
//       image_pub_ = it_.advertise("/holocv/output_video", 1);
//       vis_markers_pub_ = nh_.advertise<geometry_msgs::PoseArray> ("/holocv/markers", 10);
      
//       cv::namedWindow(OPENCV_WINDOW);

//       _nh.getParam("camera_calibration", camera_calibration);
//       ROS_INFO_STREAM("HoloCV: Provided camera_calibration: '" << camera_calibration << "'");
//       _nh.getParam("marker_length", markerLength);
//       ROS_INFO_STREAM("HoloCV: Marker length is: " << markerLength);
//       _nh.getParam("charuco_board", charuco_board);
//       ROS_INFO_STREAM("CharucoBoard Detection: " << charuco_board);

//       int dictionary_id;
//       _nh.getParam("dictionary_id", dictionary_id);
//       int corner_refinement_method;
//       _nh.getParam("corner_refinement_method", corner_refinement_method);

//       // Load calibration
//       detectorParams = cv::aruco::DetectorParameters::create();
//       detectorParams->cornerRefinementMethod = corner_refinement_method;
//       dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));
//       bool readOk = readCameraParameters(camera_calibration, camMatrix, distCoeffs);
//       if(!readOk) {
//           ROS_ERROR("Invalid camera file");
//       }

//       if(charuco_board){
//         float board_square_length;
//         float board_marker_length;
//         int board_squaresX;
//         int board_squaresY;
//         _nh.getParam("board_marker_length", board_square_length);
//         _nh.getParam("board_square_length", board_square_length);
//         _nh.getParam("board_squaresX", board_squaresX);
//         _nh.getParam("board_squaresY", board_squaresY);

//         axisLength = 0.5f * ((float)min(board_squaresX, board_squaresY) * (board_square_length));
//         charucoboard = cv::aruco::CharucoBoard::create(board_squaresX, board_squaresY, board_square_length, board_marker_length, dictionary);
//         board = charucoboard.staticCast<cv::aruco::Board>();
//       }
//     }

//     ~ImageProcessor()
//     {
//       cv::destroyWindow(OPENCV_WINDOW);
//     }

//     void convertImage(const sensor_msgs::ImageConstPtr& msg)
//     {
//       cv_bridge::CvImagePtr cv_ptr;
//       try
//       {
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//       }
//       catch (cv_bridge::Exception& e)
//       {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//       }
//       // Detect markers
//       if(charuco_board){
//         detectBoard(cv_ptr);
//       }else{
//         bool success = detectMarkers(cv_ptr);
//       }
      
//     }

//     void detectBoard(cv_bridge::CvImagePtr incomingImage){
//       // Detect markers and estimate pose
//       vector< int > ids, charucoIds;
//       vector< vector< cv::Point2f > > corners, rejected;
//       cv::Vec3d rvecs, tvecs;
//       vector< cv::Point2f > charucoCorners;

//       cv::aruco::detectMarkers(incomingImage->image, dictionary, corners, ids, detectorParams, rejected);

//       cv::aruco::refineDetectedMarkers(incomingImage->image, board, corners, ids, rejected,
//                                          camMatrix, distCoeffs);

//       int interpolatedCorners = 0;
//         if(ids.size() > 0)
//             interpolatedCorners =
//                 cv::aruco::interpolateCornersCharuco(corners, ids, incomingImage->image, charucoboard,
//                                                  charucoCorners, charucoIds, camMatrix, distCoeffs);
//       bool validPose = false;
//         if(camMatrix.total() != 0)
//             validPose = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
//                                                         camMatrix, distCoeffs, rvecs, tvecs);


//       // draw results
//       if(ids.size() > 0){
//         cv::aruco::drawDetectedMarkers(incomingImage->image, corners, ids);
//       }
//       if(interpolatedCorners > 0) {
//             cv::Scalar color;
//             color = cv::Scalar(255, 0, 0);
//             cv::aruco::drawDetectedCornersCharuco(incomingImage->image, charucoCorners, charucoIds, color);
//         }
//       if(validPose)
//             cv::aruco::drawAxis(incomingImage->image, camMatrix, distCoeffs, rvecs, tvecs, axisLength);        
      

//       geometry_msgs::Pose pose;
//       // Assign translation vectors to pose
//       pose.position.x = tvecs[0];
//       pose.position.y = tvecs[1];
//       pose.position.z = tvecs[2];

//       cv::Mat rot_cv(3, 3, CV_32FC1);
//       cv::Rodrigues(rvecs, rot_cv);
//       double Q[4];
//       getQuaternion(rot_cv, Q);
      
//       pose.orientation.x = Q[0];
//       pose.orientation.y = Q[1];
//       pose.orientation.z = Q[2];
//       pose.orientation.w = Q[3];
      
//       geometry_msgs::PoseArray poseArray;
//       poseArray.header.frame_id = "main";
//       poseArray.header.stamp = ros::Time::now();
//       poseArray.poses.push_back(pose);
//       vis_markers_pub_.publish(poseArray);

//       displayImage(incomingImage);
//     }

//     bool detectMarkers(cv_bridge::CvImagePtr incomingImage){


//       // Detect markers and estimate pose
//       vector< int > ids;
//       vector< vector< cv::Point2f > > corners, rejected;
//       vector< cv::Vec3d > rvecs, tvecs;

//       cv::aruco::detectMarkers(incomingImage->image, dictionary, corners, ids, detectorParams, rejected);
//       if(ids.size() > 0){
//         cv::aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
//         cv::aruco::drawDetectedMarkers(incomingImage->image, corners, ids);
//         for(unsigned int i = 0; i < ids.size(); i++){
//           cv::aruco::drawAxis(incomingImage->image, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
//         }
//       }

//       std::map<int, geometry_msgs::Pose> idPoseMap;
//       for(int i = 0; i<ids.size(); i++){

//         geometry_msgs::Pose pose;
//         // Assign translation vectors to pose
//         pose.position.x = tvecs[i][0];
//         pose.position.y = tvecs[i][1];
//         pose.position.z = tvecs[i][2];

//         cv::Mat rot_cv(3, 3, CV_32FC1);
//         cv::Rodrigues(rvecs[i], rot_cv);
//         double Q[4];
//         getQuaternion(rot_cv, Q);
        
//         pose.orientation.x = Q[0];
//         pose.orientation.y = Q[1];
//         pose.orientation.z = Q[2];
//         pose.orientation.w = Q[3];

//         idPoseMap.insert(std::make_pair(ids[i], pose));
//       }


//       std::map<int, geometry_msgs::Pose>::iterator it = idPoseMap.begin();
//       geometry_msgs::PoseArray poseArray;
//       poseArray.header.frame_id = "main";
//       poseArray.header.stamp = ros::Time::now();
//       while(it != idPoseMap.end()){
//         poseArray.poses.push_back(it->second);
//         it++;
//       }
//       vis_markers_pub_.publish(poseArray);

//       displayImage(incomingImage);
//     }

//     void displayImage(cv_bridge::CvImagePtr processedImage){
//       cv::imshow(OPENCV_WINDOW, processedImage->image);
//       cv::waitKey(3);
//       // Output modified video stream as ROS sensor_msgs::Image
//       image_pub_.publish(processedImage->toImageMsg());
//     }
// };

int main(int argc, char** argv)
{
  ros::init(argc, argv, "digital_futures");
  //ImageProcessor ic;
  ros::spin();
  return 0;
}
