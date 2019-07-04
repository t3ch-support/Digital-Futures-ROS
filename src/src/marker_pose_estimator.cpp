// Standard
#include <string>
#include <math.h> 
//ROS
#include <ros/ros.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

// Camera
#include <raspicam/raspicam_cv.h>

// ROS Msgs
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

// ROS Positioning
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>



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

class ImageProcessor
{
    ros::NodeHandle nh_;
    raspicam::RaspiCam_Cv Camera;
    ros::Publisher pose_pub;
    ros::Publisher id_pub;
    int vidWidth = 640;
    int vidHeight = 480;
    std::string camera_calibration;
    float markerLength;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Mat camMatrix, distCoeffs;
    int dictionary_id;
    int corner_refinement_method;
    std::string out;
    std::string robot_id;

    geometry_msgs::PoseStamped lastPose;

    public:
    ImageProcessor()
    {
        ros::NodeHandle _nh("~");
        
        // Load Params from launch file
        _nh.getParam("robot_id", robot_id);
        ROS_INFO_STREAM("Robot ID: '" << robot_id);
        _nh.getParam("camera_calibration", camera_calibration);
        ROS_INFO_STREAM("Provided camera_calibration: '" << camera_calibration << "'");
        _nh.getParam("marker_length", markerLength);
        ROS_INFO_STREAM("Marker length is: " << markerLength);
        _nh.getParam("dictionary_id", dictionary_id);
        ROS_INFO_STREAM("Dictionrary ID: " << dictionary_id);
        _nh.getParam("corner_refinement_method", corner_refinement_method);
        ROS_INFO_STREAM("Corner Refinement Method: " << corner_refinement_method);
        _nh.getParam("output_path", out);
        ROS_INFO_STREAM("Output Path: " << out);

        // Publishers
        pose_pub = nh_.advertise<geometry_msgs::PoseStamped> ("/digital_futures/"+robot_id+"/robot_pose", 10);
        id_pub = nh_.advertise<std_msgs::Int16>("/digital_futures/"+robot_id+"/marker_id", 1);

        cv::namedWindow(OPENCV_WINDOW);

        // Initialize Camera
        setCameraParams(Camera);
        if ( !Camera.open() ) {
            ROS_ERROR("Error opening camera");
        }else{
            ROS_INFO_STREAM("Connected to camera =" << Camera.getId());
        }

        // Load calibration
        try{
            detectorParams = cv::aruco::DetectorParameters::create();
        }catch( cv::Exception& e ){
            const char* err_msg = e.what();
            ROS_INFO_STREAM("Detector Params Exception caught: " << err_msg);
        }
        detectorParams->cornerRefinementMethod = corner_refinement_method;
        
        // Load custom dictionary
        cv::FileStorage fsr(out + "dic_save.yml", cv::FileStorage::READ);
        int mSize, mCBits;
        cv::Mat bits;
        fsr["MarkerSize"] >> mSize;
        fsr["MaxCorrectionBits"] >> mCBits;
        fsr["ByteList"] >> bits;
        fsr.release();        
        dictionary = cv::makePtr<cv::aruco::Dictionary>(bits, mSize, mCBits);
        // dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

        // Load Camera Parameters
        bool readOk = readCameraParameters(camera_calibration, camMatrix, distCoeffs);
        if(!readOk) ROS_ERROR("Invalid camera file");

        // Start detection loop
        detectMarkers();
    }

    ~ImageProcessor()
    {
      //cv::destroyWindow(OPENCV_WINDOW);
    }

   

    bool detectMarkers(){
        while(Camera.grab()){       
            cv::Mat image;
            Camera.retrieve(image);

            // Detect markers and estimate pose
            vector< int > ids;
            vector< vector< cv::Point2f > > corners, rejected;
            vector< cv::Vec3d > rvecs, tvecs;

            cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
            int closestMarkerId = 0;
            double minDistance = 1000;
            int arrayLoc = 0;
            if(ids.size() > 0){                
                cv::aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
                cv::aruco::drawDetectedMarkers(image, corners, ids);
                for(unsigned int i = 0; i < ids.size(); i++){
                    // Determine which marker is closest to center of image
                    cv::Point2f markerCenter((corners[i][0].x + corners[i][2].x)/2, (corners[i][0].y + corners[i][2].y)/2);
                    //cv::circle( image, markerCenter, 10, cv::Scalar( 255, 0, 0 ),5, cv::LineTypes::FILLED, 0);
                    double distance = findDistanceToCenter(markerCenter);
                    if(distance < minDistance){
                        minDistance = distance;
                        closestMarkerId = ids[i];
                        arrayLoc = i;
                    }

                    cv::aruco::drawAxis(image, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
                }

            }
            if(minDistance != 1000){
                //ROS_INFO_STREAM("MarkerID: " << closestMarkerId);
                std_msgs::Int16 markerId;
                markerId.data = closestMarkerId;
                id_pub.publish(markerId);

            
                // Process Marker Pose
                geometry_msgs::PoseStamped pose;
           

                cv::Mat rot_cv(3, 3, CV_32FC1);
                cv::Rodrigues(rvecs[arrayLoc], rot_cv);
                rot_cv = rot_cv.t(); // rotation of inverse
                
                //cv::Mat cameraRotationVector;
                //cv::Rodrigues(rot_cv.t(), cameraRotationVector);
                cv::Mat tvec_mat(tvecs[arrayLoc]);
                
                cv::Mat cameraTranslationVector = -rot_cv*tvec_mat;

                // Assign translation vectors to pose
                
                // pose.pose.position.x = tvecs[arrayLoc][0];
                // pose.pose.position.y = tvecs[arrayLoc][1];
                // pose.pose.position.z = tvecs[arrayLoc][2];
                pose.pose.position.x = cameraTranslationVector.at<double>(0);
                pose.pose.position.y = cameraTranslationVector.at<double>(1);
                pose.pose.position.z = cameraTranslationVector.at<double>(2);
                double Q[4];
                getQuaternion(rot_cv, Q);
                
                pose.pose.orientation.x = Q[0];
                pose.pose.orientation.y = Q[1];
                pose.pose.orientation.z = Q[2];
                pose.pose.orientation.w = Q[3];
                
                std::string _id = std::to_string(closestMarkerId);
                pose.header.frame_id = "marker_frame_"+_id;
                pose.header.stamp = ros::Time::now();
                pose_pub.publish(pose);

                // Publish Transform
                static tf2_ros::TransformBroadcaster br;
                geometry_msgs::TransformStamped transformStamped;
                ROS_INFO_STREAM("MarkerID: " << "marker_frame_" << _id);
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "marker_frame_"+_id;
                transformStamped.child_frame_id = robot_id;
                transformStamped.transform.translation.x = pose.pose.position.x;
                transformStamped.transform.translation.y = pose.pose.position.y;
                transformStamped.transform.translation.z = pose.pose.position.z;
                tf2::Quaternion q;
                q.setX(pose.pose.orientation.x);
                q.setY(pose.pose.orientation.y);
                q.setZ(pose.pose.orientation.z);
                q.setW(pose.pose.orientation.w);
                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();
                br.sendTransform(transformStamped);
            }
            displayImage(image);
        }
    }
    double findDistanceToCenter(cv::Point2f pt){
        int widthX = pt.x-(vidWidth/2);
        int heightY = pt.y-(vidHeight/2);
        double distance = sqrt( widthX * widthX + heightY * heightY);
        return distance;
    }
    void displayImage(cv::Mat processedImage){
        cv::Mat dst;               // dst must be a different Mat
        cv::flip(processedImage, dst, 1);     // because you can't flip in-place (leads to segfault)
        cv::imshow(OPENCV_WINDOW, dst);
        cv::waitKey(3);
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "digital_futures");
  ImageProcessor ic;
  ros::spin();
  return 0;
}
