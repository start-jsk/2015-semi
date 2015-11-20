#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

void search(cv::Mat& img, int pos[2])
{
	int sum = 0;
	int ax = 0;
	int ay = 0;
/*	for (int y = 0; y < img.rows; y++) {
		for (int x = 0; x < img.cols; x++) {
			cv::Vec3b &p = img.at<cv::Vec3b>(y, x);
			if (p[0] < 30 && p[1] < 30 && p[2] > 200) {
				sum++;
				ax += x;
				ay += y;
			}
		}
	}
	if (sum < 50) {
		pos[0] = -1;
		pos[1] = -1;
		return;
	}
	ax = (int)((double)ax/(double)sum);
	ay = (int)((double)ay/(double)sum);
	pos[0] = ax;
	pos[1] = ay;*/
	int width = img.cols;
	int height  = img.rows;

	cv::Mat hsv_image;
	cv::cvtColor(img, hsv_image, CV_BGR2HSV);
	unsigned char hue, sat, val;
	cv::Mat red_image = cv::Mat(cv::Size(width, height), CV_8UC1);
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			hue = hsv_image.at<cv::Vec3b>(y, x)[0];
			sat = hsv_image.at<cv::Vec3b>(y, x)[1];
			val = hsv_image.at<cv::Vec3b>(y, x)[2];

			if ((hue < 3 || hue > 173) && sat > 170) {
				sum++;
				ax += x;
				ay += y;
			}
		}
	}
	if (sum < 50) {
		pos[0] = -1;
		pos[1] = -1;
		return;
	}
	pos[0] = (int)((double)ax/(double)sum);
	pos[1] = (int)((double)ay/(double)sum);
}


