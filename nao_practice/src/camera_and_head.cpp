#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "search.hpp"
#include "move_head.hpp"
#include "detect_face.hpp"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:
	int difx;
	int dify;
	bool found;
	ImageConverter() : it_(nh_)
	{
		found = false;
		image_sub_ = it_.subscribe("/camera/front/image_raw", 1, &ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		cv::namedWindow("image1", CV_WINDOW_AUTOSIZE);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
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
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60 && pos[0] != -1 && pos[1] != -1) {
			cv::circle(cv_ptr->image, cv::Point(pos[0], pos[1]), 10, CV_RGB(0, 0, 255), 3);
			printf("found\n");
			difx = img.cols/2 - pos[0];
			dify = img.rows/2 - pos[1];
			found = true;
		} else {
			printf("not found\n");
			found = false;
		}
		cv::imshow("image1", img);
		cv::waitKey(3);

		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv)
{
	int dir = 1;
	double angleyaw = 0;
	double anglepitch = 0;
	ros::init(argc, argv, "image_converter");
	
	RobotHead head;
	ImageConverter ic;
	while (ros::ok()) {
		if (head.getState().isDone()) {
			if (ic.found) {
				if (ic.difx > 0) angleyaw += 0.1;
				if (ic.difx < 0) angleyaw -= 0.1;
				if (ic.dify > 0) anglepitch -= 0.1;
				if (ic.dify < 0) anglepitch += 0.1;
			} else {
				anglepitch = 0;
				angleyaw = dir*0.2;
				dir = -dir;
			}
			if (angleyaw > 2.0 || angleyaw < -2.0) angleyaw = 0;
			if (anglepitch > 0.5 || anglepitch < -0.6) anglepitch = 0;
			head.startTrajectory(head.headExtentionTrajectory(anglepitch, angleyaw));
		}
		ros::spinOnce();
		usleep(50000);
	}
	return 0;
}
