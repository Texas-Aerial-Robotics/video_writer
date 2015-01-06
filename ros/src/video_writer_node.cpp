#include <video_writer/video_writer_node.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

VideoWriterNode::VideoWriterNode(ros::NodeHandle &nh) : nh_(nh), it_(nh)
{
    std::string input_topic;
    if (!nh_.getParam("input_topic", input_topic))
    {
        ROS_ERROR("Input topic needs to be specified");
        exit(0);
    }

    if (!nh_.getParam("output_file", output_file_name_))
    {
        ROS_ERROR("Output file needs to be specified");
        exit(0);
    }

    image_subscriber_ = it_.subscribe(input_topic, 0, &VideoWriterNode::imageCallback, this);
}

VideoWriterNode::~VideoWriterNode()
{
}

void VideoWriterNode::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    if (!video_writer_.isOpened())
    {
        cv::Size image_size(image->width, image->height);
        double frame_rate;
        std::string fourcc;
        std::string encoding_;
        nh_.param<double>("frame_rate", frame_rate, 30.0);
        nh_.param<std::string>("fourcc", fourcc, "DIVX");
        nh_.param<std::string>("encoding", encoding_, image->encoding);
        
        video_writer_.open(output_file_name_, CV_FOURCC(fourcc.at(0), fourcc.at(1), fourcc.at(2), fourcc.at(3)),
                           frame_rate, image_size, sensor_msgs::image_encodings::isColor(encoding_));
        if (!video_writer_.isOpened())
        {
            ROS_ERROR("Could not open file %s", output_file_name_.c_str());
            ros::shutdown();
        }
        ROS_INFO("Started recording video %s", output_file_name_.c_str());
        ROS_INFO("Frame Rate: %.2f", frame_rate);
        ROS_INFO("Video Codec: %s ", fourcc.c_str());
        ROS_INFO("Image Encoding: %s", encoding_.c_str());
    }
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image, encoding_);
    bool convert;
    nh_.param<bool>("convert", convert, false);
    if (convert)
    {
        cv::Mat copy;
        cv::cvtColor(cv_image->image, copy, CV_BGR2RGB); 
        video_writer_.write(copy);
    }
    else
    {
        video_writer_.write(cv_image->image);
    }
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "video_writer");

    ros::NodeHandle n("~");

    ROS_INFO("[video_writer] node started");

    VideoWriterNode video_writer_node(n); 

    ros::spin();

    return 0;
}
