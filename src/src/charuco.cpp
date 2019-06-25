 void detectBoard(){
        ROS_INFO_STREAM("Starting board detection");
        while(Camera.grab()){       
            cv::Mat image;
            Camera.retrieve(image);
            // Detect markers and estimate pose
            vector< int > ids, charucoIds;
            vector< vector< cv::Point2f > > corners, rejected;
            cv::Vec3d rvecs, tvecs;
            vector< cv::Point2f > charucoCorners;

            cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

            cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected,
                                                camMatrix, distCoeffs);

            int interpolatedCorners = 0;
                if(ids.size() > 0)
                    interpolatedCorners =
                        cv::aruco::interpolateCornersCharuco(corners, ids, image, charucoboard,
                                                        charucoCorners, charucoIds, camMatrix, distCoeffs);
            bool validPose = false;
                if(camMatrix.total() != 0)
                    validPose = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
                                                                camMatrix, distCoeffs, rvecs, tvecs);
            // draw results
            if(ids.size() > 0){
                cv::aruco::drawDetectedMarkers(image, corners, ids);
            }
            if(interpolatedCorners > 0) {
                    cv::Scalar color;
                    color = cv::Scalar(255, 0, 0);
                    cv::aruco::drawDetectedCornersCharuco(image, charucoCorners, charucoIds, color);
                }
            if(validPose){

                    cv::aruco::drawAxis(image, camMatrix, distCoeffs, rvecs, tvecs, axisLength);
                    geometry_msgs::PoseStamped pose;
                    // Assign translation vectors to pose
                    pose.pose.position.x = tvecs[0];
                    pose.pose.position.y = tvecs[1];
                    pose.pose.position.z = tvecs[2];

                    cv::Mat rot_cv(3, 3, CV_32FC1);
                    cv::Rodrigues(rvecs, rot_cv);
                    
                    double Q[4];
                    getQuaternion(rot_cv, Q);
                    
                    pose.pose.orientation.x = Q[0];
                    pose.pose.orientation.y = Q[1];
                    pose.pose.orientation.z = Q[2];
                    pose.pose.orientation.w = Q[3];
                    
                    pose.header.frame_id = "aruco_camera";
                    pose.header.stamp = ros::Time::now();
                    // If pose array use this
                    // geometry_msgs::PoseArray poseArray;
                    // poseArray.header.frame_id = "/holo_cam/pose";
                    // poseArray.header.stamp = ros::Time::now();
                    // poseArray.poses.push_back(pose);
                    pose_pub.publish(pose);
                    lastPose = pose;

                    // Publish pose as transform
                    static tf2_ros::TransformBroadcaster br;
                    geometry_msgs::TransformStamped transformStamped;

                    transformStamped.header.stamp = ros::Time::now();
                    transformStamped.header.frame_id = "aruco_camera";
                    transformStamped.child_frame_id = "marker_frame";
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
            // geometry_msgs::Pose pose;
            // // Assign translation vectors to pose
            // pose.position.x = tvecs[0];
            // pose.position.y = tvecs[1];
            // pose.position.z = tvecs[2];

            // cv::Mat rot_cv(3, 3, CV_32FC1);
            // cv::Rodrigues(rvecs, rot_cv);
            // double Q[4];
            // getQuaternion(rot_cv, Q);
            
            // pose.orientation.x = Q[0];
            // pose.orientation.y = Q[1];
            // pose.orientation.z = Q[2];
            // pose.orientation.w = Q[3];
            
            // geometry_msgs::PoseArray poseArray;
            // poseArray.header.frame_id = "main";
            // poseArray.header.stamp = ros::Time::now();
            // poseArray.poses.push_back(pose);
            // pose_pub.publish(poseArray);

            displayImage(image);
        }
        ROS_INFO_STREAM("Ended");

    }