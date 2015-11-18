#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

int detect_hand(cv::Mat& img)
{
	double scale = 4.0;
	cv::Mat gray, smallImg(cv::saturate_cast<int>(img.rows/scale), cv::saturate_cast<int>(img.cols/scale), CV_8UC1);
	cv::cvtColor(img, gray, CV_BGR2GRAY);
	cv::resize(gray, smallImg, smallImg.size(), 0, 0, cv::INTER_LINEAR);
	cv::equalizeHist(smallImg, smallImg);

	std::string cascadeName = "./haarcascade_frontalface_alt.xml";
	cv::CascadeClassifier cascade;
	if (!cascade.load(cascadeName))
		return -1;
	
	std::vector<cv::Rect> faces;
	cascade.detectMultiScale(smallImg, faces, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

	std::vector<cv::Rect>::const_iterator r = faces.begin();
	for (; r != faces.end(); ++r) {
		cv::Point center;
		int radius;
		center.x = cv::saturate_cast<int>((r->x + r->width*0.5)*scale);
		center.y = cv::saturate_cast<int>((r->y + r->height*0.5)*scale);
		radius = cv::saturate_cast<int>((r->width + r->height) * 0.25 * scale);
		cv::circle(img, center, 10, cv::Scalar(80, 80, 255), 3, 8, 0);
	}
}
