#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <ctime>
#include <ros/ros.h>
#include <sstream>

using namespace cv;


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "marker_generator");

    ros::NodeHandle n;
    ros::NodeHandle _nh("~");

    int board_or_single;
    int single_count;
    int single_size;
    int board_square_length;
    int board_marker_length;
    int board_squaresX;
    int board_squaresY;
    int dictionary_id;
    int border_bits;
    std::string out;


    _nh.getParam("board_or_single", board_or_single);
    ROS_INFO_STREAM("Board or Single: " << board_or_single);

    _nh.getParam("single_size", single_size);
    ROS_INFO_STREAM("Single Size: " << single_size);

    _nh.getParam("single_count", single_count);
    ROS_INFO_STREAM("Single Count: " << single_count);

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

    _nh.getParam("border_bits", border_bits);
    ROS_INFO_STREAM("Border Bits: " << border_bits);

    _nh.getParam("output_path", out);
    ROS_INFO_STREAM("Output Path: " << out);


    int margins = board_square_length - board_marker_length;

    // Custom dictionary
    int number= 10, dimension=7;
    Ptr<aruco::Dictionary> dictionary = cv::aruco::generateCustomDictionary(single_count, dimension);
    cv::Mat store=dictionary->bytesList;
    cv::FileStorage fs(out+"dic_save.yml", cv::FileStorage::WRITE);
    fs << "MarkerSize" << dictionary->markerSize;
    fs << "MaxCorrectionBits" << dictionary->maxCorrectionBits;
    fs << "ByteList" << dictionary->bytesList;
    fs.release();


    // Ptr<aruco::Dictionary> dictionary =
    //     aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

    Size imageSize;
    imageSize.width = board_squaresX * board_square_length + 2 * margins;
    imageSize.height = board_squaresY * board_square_length + 2 * margins;
    if(board_or_single == 0){
        Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(board_squaresX, board_squaresY, (float)board_square_length,
                                                                (float)board_marker_length, dictionary);

        // show created board
        Mat boardImage;
        board->draw(imageSize, boardImage, margins, border_bits);

        imshow("board", boardImage);
        waitKey(0);

        imwrite(out+"charuco_board.jpg", boardImage);
    }else{
        Mat markerImage;
        ROS_INFO_STREAM(single_count);
        for(int i = 0; i<single_count; i++){
            ROS_INFO_STREAM(i);
            aruco::drawMarker(dictionary, i, single_size, markerImage, 1);
            imshow("marker", markerImage);
            waitKey(0);

            std::stringstream ss;
            ss << out << "marker" << i << ".jpg";
            std::string outputName = ss.str();
            ROS_INFO_STREAM(outputName);
            
            imwrite(outputName, markerImage);
            
            
        }
    }
    return 0;
}
