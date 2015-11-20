#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "search.hpp"
#include "move_head.hpp"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:
	ImageConverter() : it_(nh_)
	{
		image_sub_ = it_.subscribe("/camera/front/image_raw", 1, &ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		static int count = 0;
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		cv::Mat img = cv_ptr->image;
		int pos[2];
		search(img, pos);
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60 && pos[0] != -1 && pos[1] != -1)
			cv::circle(cv_ptr->image, cv::Point(pos[0], pos[1]), 10, CV_RGB(0, 0, 255));
		else {
			printf("not found %d\n", count);
			count++;
		}

		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);

		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
