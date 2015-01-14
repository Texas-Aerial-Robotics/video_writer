/* video_writer_node.h
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#ifndef VIDEO_WRITER_NODE_H_
#define VIDEO_WRITER_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

/**
 * This class publishes frames from an input video file to a sensor_msgs/Image topic at the frame rate of the video
 * The video is read using OpenCV and converted to the ros Image message using cv_bridge
 */
class VideoWriterNode
{
    public:
        /**
         * Constructor
         *
         * @param nh
         * ROS NodeHandle object
         */
        VideoWriterNode(ros::NodeHandle &nh);

        /**
         * Destructor
         */
        virtual ~VideoWriterNode();

        /**
         * Callback for image subscriber
         *
         * @param image
         * sensor_msgs Image received from topic
         */
        void imageCallback(const sensor_msgs::ImageConstPtr &image);


    private:
        /**
         * ROS NodeHandle object
         */
        ros::NodeHandle nh_;

        /**
         * ImageTransport object
         */
        image_transport::ImageTransport it_;
        
        /**
         * Image subscriber
         */
        image_transport::Subscriber image_subscriber_;

        /**
         * VideoWriter object
         */
        cv::VideoWriter video_writer_;

        /**
         * Full path to output file
         */
        std::string output_file_name_;

        /**
         * Image encoding type
         */
        std::string encoding_;
};

#endif
