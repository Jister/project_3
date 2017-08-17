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

//左相机内参数矩阵  
float leftIntrinsic[3][3] = {350.007,       0,             353.108,  
                             0,             350.007,       176.841,  
                             0,             0,             1};  
//左相机畸变系数  
float leftDistortion[1][5] = {0.0, 0.0, 0.0, 0.0, 0.0};  
//左相机旋转矩阵  
//Rx = RX (in rad),
//Ry = CV,
//Rz = RZ.
//Tx = - baseline,
//Ty = 0,
//Tz = 0. 
float leftRot_V[3] = {0.00575019, 0.0122058, -0.000346486};
//左相机平移向量  
float leftTranslation[1][3] = {-120.0, 0.0, 0.0};  
  
//右相机内参数矩阵  
float rightIntrinsic[3][3] ={350.035,       0,             350.161,  
                             0,             350.035,       188.171,  
                             0,             0,             1};  
//右相机畸变系数  
float rightDistortion[1][5] =  {0, 0, 0, 0, 0};    
//右相机旋转矩阵 
float rightRot_V[3] = {0.0, 0.0, 0.0};
//右相机平移向量  
float rightTranslation[1][3] = {0.0, 0.0, 0.0};  


bool left_flag = false;
bool right_flag = false;
Mat left_image;
Mat right_image;

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

//************************************  
// Description: 根据左右相机中成像坐标求解空间坐标  
// Method:    uv2xyz   
// Parameter: Point2f uvLeft  
// Parameter: Point2f uvRight  
// Returns:   cv::Point3f  
  
//************************************  
Point3f uv2xyz(Point2f uvLeft,Point2f uvRight)  
{  
    //  [u1]      |X|                     [u2]      |X|  
    //Z*|v1| = Ml*|Y|                   Z*|v2| = Mr*|Y|  
    //  [ 1]      |Z|                     [ 1]      |Z|  
    //            |1|                               |1|  
    Mat mleftRot_V = Mat(3,1,CV_32F,leftRot_V);
    Mat mLeftRotation;
    Rodrigues(mleftRot_V, mLeftRotation);
    Mat mLeftTranslation = Mat(3,1,CV_32F,leftTranslation);  
    Mat mLeftRT = Mat(3,4,CV_32F);//左相机M矩阵  
    hconcat(mLeftRotation,mLeftTranslation,mLeftRT);  
    Mat mLeftIntrinsic = Mat(3,3,CV_32F,leftIntrinsic);  
    Mat mLeftM = mLeftIntrinsic * mLeftRT;  
    cout<<"左相机M矩阵 = "<<endl<<mLeftM<<endl;  
  	
  	Mat mrightRot_V = Mat(3,1,CV_32F,rightRot_V);
    Mat mRightRotation;
    Rodrigues(mrightRot_V, mRightRotation);
    Mat mRightTranslation = Mat(3,1,CV_32F,rightTranslation);  
    Mat mRightRT = Mat(3,4,CV_32F);//右相机M矩阵  
    hconcat(mRightRotation,mRightTranslation,mRightRT);  
    Mat mRightIntrinsic = Mat(3,3,CV_32F,rightIntrinsic);  
    Mat mRightM = mRightIntrinsic * mRightRT;  
    cout<<"右相机M矩阵 = "<<endl<<mRightM<<endl;  
  
    //最小二乘法A矩阵  
    Mat A = Mat(4,3,CV_32F);  
    A.at<float>(0,0) = uvLeft.x * mLeftM.at<float>(2,0) - mLeftM.at<float>(0,0);  
    A.at<float>(0,1) = uvLeft.x * mLeftM.at<float>(2,1) - mLeftM.at<float>(0,1);  
    A.at<float>(0,2) = uvLeft.x * mLeftM.at<float>(2,2) - mLeftM.at<float>(0,2);  
  
    A.at<float>(1,0) = uvLeft.y * mLeftM.at<float>(2,0) - mLeftM.at<float>(1,0);  
    A.at<float>(1,1) = uvLeft.y * mLeftM.at<float>(2,1) - mLeftM.at<float>(1,1);  
    A.at<float>(1,2) = uvLeft.y * mLeftM.at<float>(2,2) - mLeftM.at<float>(1,2);  
  
    A.at<float>(2,0) = uvRight.x * mRightM.at<float>(2,0) - mRightM.at<float>(0,0);  
    A.at<float>(2,1) = uvRight.x * mRightM.at<float>(2,1) - mRightM.at<float>(0,1);  
    A.at<float>(2,2) = uvRight.x * mRightM.at<float>(2,2) - mRightM.at<float>(0,2);  
  
    A.at<float>(3,0) = uvRight.y * mRightM.at<float>(2,0) - mRightM.at<float>(1,0);  
    A.at<float>(3,1) = uvRight.y * mRightM.at<float>(2,1) - mRightM.at<float>(1,1);  
    A.at<float>(3,2) = uvRight.y * mRightM.at<float>(2,2) - mRightM.at<float>(1,2);  
  
    //最小二乘法B矩阵  
    Mat B = Mat(4,1,CV_32F);  
    B.at<float>(0,0) = mLeftM.at<float>(0,3) - uvLeft.x * mLeftM.at<float>(2,3);  
    B.at<float>(1,0) = mLeftM.at<float>(1,3) - uvLeft.y * mLeftM.at<float>(2,3);  
    B.at<float>(2,0) = mRightM.at<float>(0,3) - uvRight.x * mRightM.at<float>(2,3);  
    B.at<float>(3,0) = mRightM.at<float>(1,3) - uvRight.y * mRightM.at<float>(2,3);  
  
    Mat XYZ = Mat(3,1,CV_32F);  
    //采用SVD最小二乘法求解XYZ  
    solve(A,B,XYZ,DECOMP_SVD);  
    cout<<"空间坐标为 = "<<endl<<XYZ<<endl;  
  
    //世界坐标系中坐标  
    Point3f world;  
    world.x = XYZ.at<float>(0,0);  
    world.y = XYZ.at<float>(1,0);  
    world.z = XYZ.at<float>(2,0);  
  
    return world;  
}  

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
	ros::Subscriber left_sub = n.subscribe("/zed/left/image_rect_color", 1, &leftImageCallback);
	ros::Subscriber right_sub = n.subscribe("/zed/right/image_rect_color", 1, &rightImageCallback);
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

			imshow("image_threshold_left",left.image_threshold);
			imshow("image_threshold_right",right.image_threshold);
			imshow("source left",left.image);
			imshow("source right",right.image);
			waitKey(1);

			if(left.image_valid && right.image_valid)
			{
				Point3f pos;
				pos = uv2xyz(left.uvPos, right.uvPos);
				ROS_INFO("%f    %f    %f", pos.x, pos.y, pos.z);
			}
	

			left_flag = false;
			right_flag = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
