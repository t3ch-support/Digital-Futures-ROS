#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <ctime>
#include <ros/ros.h>

using namespace cv;


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "charuco_board_generator");

    ros::NodeHandle n;
    ros::NodeHandle _nh("~");

    int board_square_length;
    int board_marker_length;
    int board_squaresX;
    int board_squaresY;
    int dictionary_id;
    int border_bits;
    std::string out;

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


    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

    Size imageSize;
    imageSize.width = board_squaresX * board_square_length + 2 * margins;
    imageSize.height = board_squaresY * board_square_length + 2 * margins;

    Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(board_squaresX, board_squaresY, (float)board_square_length,
                                                            (float)board_marker_length, dictionary);

    // show created board
    Mat boardImage;
    board->draw(imageSize, boardImage, margins, border_bits);

    imshow("board", boardImage);
    waitKey(0);

    imwrite(out, boardImage);

    return 0;
}

// Manual compile code
// g++ -std=c++17 main.cpp -o ../bin/charucoBoardGenerator `pkg-config --cflags --libs opencv jsoncpp`
// ./charucoBoardGenerator "charucoBoard.jpg" -d=0 -w=5 -h=8 -sl=350 -ml=275 si=true