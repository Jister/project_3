#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "project_3/Image_info.h"

using namespace cv;
using namespace std;

bool left_flag = false;
bool right_flag = false;
Mat left_image;
Mat right_image;

class imageDetect
{
private:
	Mat image;
	Mat image_threshold;
public:
	bool image_valid;
	float ROI_x;
	float ROI_y;

	imageDetect(Mat src)
	{
		image = src;
		image_threshold = Mat(image.size(), CV_8UC1);
	};
	~imageDetect(){};

	void getThresholdImage()
	{
		GaussianBlur(image, image, Size(5, 5), 0, 0);  
		for (int i = 0; i < image.rows; i++)
		{
			for (int j = 0; j < image.cols; j++)
			{
				int ValB = image.at<Vec3b>(i,j)[0];
				int ValG = image.at<Vec3b>(i,j)[1];
				int ValR = image.at<Vec3b>(i,j)[2];
				if ((ValR-ValB)>=60 && (ValG-ValB)>=40)
				{
					image_threshold.at<uchar>(i,j) = 255;
				}else
				{
					image_threshold.at<uchar>(i,j) = 0;
				}
			}
		}
		/*image operation*/
		// Mat element = getStructuringElement(MORPH_RECT, Size(4,4));
		// morphologyEx(image_threshold, image_threshold, MORPH_OPEN, element);
		// morphologyEx(image_threshold, image_threshold, MORPH_CLOSE, element);   
		// erode(image_threshold, image_threshold, element);
		// dilate(image_threshold, image_threshold, element);
		imshow("image_threshold",image_threshold);
		waitKey(1);
	};

	void getPosition()
	{
		/*find contours*/
		vector<vector<Point> > contours;
		Rect rb;
		findContours(image_threshold, contours, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_NONE);
		
		/*erase unqualified contours, and find the max contour*/
		float maxarea = 0;  
		vector<vector<Point> >::iterator maxAreaIdx;  
		vector<vector<Point> >::iterator it = contours.begin();
		while(it != contours.end())
		{
			if (contourArea(*it) < 500)
			{
				it = contours.erase(it);
			}
			else
			{
				if(contourArea(*it) > maxarea)
				{
					maxarea = contourArea(*it);
					maxAreaIdx = it;
				}			
				++it;
			}
		}

		if(maxarea > 0)
		{
			image_valid = true;
			rb = boundingRect(Mat(*maxAreaIdx));
			rectangle(image, rb, Scalar(0, 255, 0), 3);

			ROI_x = (rb.x + rb.x + rb.width) / 2;
			ROI_y = (rb.y + rb.y + rb.height) / 2;

		}else
		{
			image_valid = false;
		}
			
	};

	
		
};

void leftImageCallback(const sensor_msgs::Image &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		left_image = cv_ptr->image;
		left_flag = true;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		left_flag = false;
		return;
	}

	
}

void rightImageCallback(const sensor_msgs::Image &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		right_image =  cv_ptr->image;
		right_flag = true;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		right_flag = false;
		return;
	}
}
	

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_detect_stereo");
	ros::NodeHandle n;
	left_sub = node.subscribe("/zed/left/image_rect_color", 1, &leftImageCallback);
	right_sub = node.subscribe("/zed/right/image_rect_color", 1, &rightImageCallback);
	ros::Publisher image_info_pub = n.advertise<project_3::Image_info>("camera/stereo/pose",1);
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		if(left_flag && right_flag)
		{
			imageDetect left(left_image);
			imageDetect right(right_image);

			left.getThresholdImage();
			left.getPosition();
			right.getThresholdImage();
			right.getPosition();

		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
