#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include "project_3/Image_info.h"

using namespace cv;
using namespace std;

bool flag = false;
Mat rgb_image;
Mat depth_image;

tf::Transform world_to_camera;

class imageDetect
{
public:
	Mat image;
	Mat image_threshold;
	bool image_valid;
	Point2f uvPos;

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

			uvPos.x = (rb.x + rb.x + rb.width) / 2;
			uvPos.y = (rb.y + rb.y + rb.height) / 2;

		}else
		{
			image_valid = false;
		}
			
	}		
}; 

void rgbImageCallback(const sensor_msgs::Image &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		rgb_image = cv_ptr->image;
		flag = true;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		flag = false;
		return;
	}	
}

void depthImageCallback(const sensor_msgs::Image &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		depth_image = cv_ptr->image;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}	
}

void odomCallback(const nav_msgs::Odometry &msg)
{
	world_to_camera.setIdentity();

	tf::Quaternion q;
	tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
	world_to_camera.setRotation(q);
}
	

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_detect_stereo");
	ros::NodeHandle n;
	ros::Subscriber rgb_sub = n.subscribe("/zed/rgb/image_rect_color", 1, &rgbImageCallback);
	ros::Subscriber depth_sub = n.subscribe("/zed/depth/depth_registered", 1, &depthImageCallback);
	ros::Subscriber odom_sub = n.subscribe("/zed/odom", 1, &odomCallback);
	ros::Publisher image_info_pub = n.advertise<project_3::Image_info>("camera/stereo/pose",1);
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		if(flag)
		{
			imageDetect left(rgb_image);
			left.getThresholdImage();
			left.getPosition();

			imshow("source",left.image);
			ROS_INFO("%d, %d", (int)left.uvPos.x,(int)left.uvPos.y);
			waitKey(1);
			if(left.image_valid)
			{	
				float depth = depth_image.at<float>((int)left.uvPos.y,(int)left.uvPos.x);
				float x,y;
				x = (left.uvPos.x - rgb_image.cols/2) * depth / 350.0;
				y = (left.uvPos.y - rgb_image.rows/2) * depth / 350.0;
				//ROS_INFO("Camera x: %f  y: %f z: %f", depth, -x, -y);
			
				tf::Vector3 p(depth, -x, -y);
				p = world_to_camera * p;
				//ROS_INFO("World x: %f  y: %f z: %f", p.getX(), p.getY(), p.getZ());
				project_3::Image_info msg;
				msg.header.stamp =  ros::Time::now();
				msg.valid = true;
				msg.x = p.getX();
				msg.y = p.getY();
				msg.z = p.getZ();
				image_info_pub.publish(msg);
			}	
			flag = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
