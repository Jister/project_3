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

//Gopro B203 1080
// float intrinsic[3][3] = {372.640550, 0.0, 315.749910, 0.0, 375.972344, 190.860416, 0, 0, 1};
// float distortion[1][5] = {-0.257675, 0.073781, -0.001009, -0.000553, 0.000000};

//Gopro 2 B203 1080
float intrinsic[3][3] = {296.737699, 0.0, 326.258343, 0.0, 299.462349, 195.294956, 0, 0, 1};
float distortion[1][5] = {-0.228952, 0.045584, -0.000883, 0.000103, 0.000000};

//Gopro B203 1080S
// float intrinsic[3][3] = {246.470673, 0.0, 317.837861, 0.0, 227.432647, 200.511899, 0, 0, 1};
// float distortion[1][5] = {-0.150726, 0.019614, 0.005480, 0.005425, 0.000000};

class imageDetect
{
private:
	Mat image;
	Mat image_threshold;
	Mat image_contour;
public:
	bool image_valid;
	bool theta_valid;
	float ROI_x;
	float ROI_y;
	float theta;
	Mat image_ROI;

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

			Rect rect;
			rect.x = ROI_x - rb.width/4;
			rect.y = ROI_y - rb.height/4;
			rect.width = rb.width/2;
			rect.height = rb.height/2;
			Mat ROI_image = Mat(Size(rect.width, rect.height), CV_8UC3);
		}else
		{
			image_valid = false;
		}
	
		if(image_valid)
		{
			Mat result(image.size() , CV_8U , cv::Scalar(0)) ; 
			Mat result_2(image.size() , CV_8U , cv::Scalar(0)) ; 
			drawContours(result , contours , -1 , cv::Scalar(255) , 1) ;  

			vector<Vec2f> lines;
			vector<Vec2f> lines_filtered;
			vector<Vec4i> lines_h; 
			vector<Vec4i> lines_v; 
			vector<float> thetas;
			HoughLines(result, lines, 1, CV_PI/180, 75, 0, 0 );  
			vector<cv::Vec2f>::const_iterator it_l= lines.begin();

			while (it_l!=lines.end())
			{
				// ROS_INFO("angle:%d",(int)((*it_l)[1]*57.3));
				// ROS_INFO("dis:%d",(int)((*it_l)[0]));
				bool check = false;
				if(lines_filtered.size() > 0)
				{
					vector<cv::Vec2f>::const_iterator ii= lines_filtered.begin();
					while (ii!=lines_filtered.end())
					{
						if(fabs((*ii)[1] - (*it_l)[1]) < CV_PI/4 && fabs((*ii)[0] - (*it_l)[0])<rb.width/2)
						{
							check = true;
							break;
						}
						++ii;
					}
				}

				if(!check)
				{
					lines_filtered.push_back((*it_l));
				}
				
				++it_l;
			}

			vector<cv::Vec2f>::const_iterator ii= lines_filtered.begin();
			while (ii!=lines_filtered.end())
			{
				float rho= (*ii)[0];   // 表示距离
				float theta= (*ii)[1]; // 表示角度
				Vec4i temp_line;

				if (theta < CV_PI/4. || theta > 3*CV_PI/4.) // 若检测为垂直线
				{ 
					// 得到线与第一行的交点
					Point pt1(rho/cos(theta),0);
					// 得到线与最后一行的交点
					Point pt2((rho-result_2.rows*sin(theta))/cos(theta),result_2.rows);
					// 调用line函数绘制直线
					line(result_2, pt1, pt2, cv::Scalar(255), 1);
					temp_line[0] = pt1.x;
					temp_line[1] = pt1.y;
					temp_line[2] = pt2.x;
					temp_line[3] = pt2.y;
					lines_v.push_back(temp_line);
					thetas.push_back(theta);
				} else // 若检测为水平线
				{ 
					// 得到线与第一列的交点
					Point pt1(0,rho/sin(theta));
					// 得到线与最后一列的交点
					Point pt2(result_2.cols,(rho-result_2.cols*cos(theta))/sin(theta));
					// 调用line函数绘制直线
					line(result_2, pt1, pt2, cv::Scalar(255), 1);
					temp_line[0] = pt1.x;
					temp_line[1] = pt1.y;
					temp_line[2] = pt2.x;
					temp_line[3] = pt2.y;
					lines_h.push_back(temp_line);
				}
				++ii;
			}

			vector<Point> corner_point;
			for(int i = 0; i < lines_v.size(); i++)
			{
				for(int j = 0; j < lines_h.size(); j++)
				{
					Point pt;	
					float x0 = lines_v[i][0];
					float y0 = lines_v[i][1];
					float x1 = lines_v[i][2];
					float y1 = lines_v[i][3];
					float x2 = lines_h[j][0];
					float y2 = lines_h[j][1];
					float x3 = lines_h[j][2];
					float y3 = lines_h[j][3];
					
					pt.y = ( (y0-y1)*(y3-y2)*x0 + (y3-y2)*(x1-x0)*y0 + (y1-y0)*(y3-y2)*x2 + (x2-x3)*(y1-y0)*y2 ) / ( (x1-x0)*(y3-y2) + (y0-y1)*(x3-x2) );
					pt.x = x2 + (x3-x2)*(pt.y-y2) / (y3-y2);
		
					corner_point.push_back(pt);
					circle(image, pt, 10,  Scalar(0,0,255), 2, 8, 0);
				}
			}

			float ratio = (float)rb.width / (float)rb.height;
			if((ratio > 0.8 || ratio < 1.2))
			{
				theta_valid = true;
				//theta = (thetas[0] + thetas[1]) / 2;
			}else
			{
				theta_valid = false;
			}
			// imshow("counters" , result) ;
			// vector<Vec4i> lines;  
			// HoughLinesP(result, lines, 1, CV_PI/180, 50, 30, 50 );  
			// for( size_t i = 0; i < lines.size(); i++ )  
			// {  
			//     line( result_2, Point(lines[i][0], lines[i][1]),  
			//         Point(lines[i][2], lines[i][3]), Scalar(255), 1);  
			// }  
			//ROS_INFO("%d",(int)corner_point.size());	 
			image_contour = result_2;
			imshow("src" , image) ;  
			//imshow("line", image_contour);
			waitKey(1);
		}
			
	};

	void getCorner()
	{
		Mat dst, dst_norm;  
		dst = Mat::zeros(image_contour.size(), CV_32FC1);  
		/// Detector parameters  
		int blockSize = 2;  
		int apertureSize = 3;  
		double k = 0.01;  
		/// Detecting corners  
		cornerHarris( image_contour, dst, blockSize, apertureSize, k, BORDER_DEFAULT );  
		/// Normalizing  
		normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );  
		/// Drawing a circle around corners  
		for( int j = 0; j < dst_norm.rows ; j++ )  
		{ 
			for( int i = 0; i < dst_norm.cols; i++ )  
			{  
				if( (int) dst_norm.at<float>(j,i) >150 )  
				{   
					circle( dst_norm, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );   
					circle(image,Point( i, j ), 5,  Scalar(255,0,0), -1, 8, 0 );  
				}  
			}   
		}      
		/// Showing the result  
		imshow( "corners_window", dst_norm );  
		imshow( "source_window", image );   
		waitKey(1);
	};
		
};

int main(int argc, char **argv)

{
	ros::init(argc, argv, "image_detect");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Publisher image_pub = it.advertise("camera/image_raw",1);
	ros::Publisher image_info_pub = n.advertise<project_3::Image_info>("camera/pose",1);
	ros::Rate loop_rate(10);

	Mat frame;
	VideoCapture cap(0);  

	Mat cameraMatrix = Mat(3,3,CV_32FC1,intrinsic);
	Mat distCoeffs = Mat(1,5,CV_32FC1,distortion);
	Mat R = Mat::eye(3, 3, CV_32F);
	Mat mapx = Mat(Size(640,360), CV_32FC1);
	Mat mapy = Mat(Size(640,360), CV_32FC1);
	initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, Size(640,360), CV_32FC1, mapx, mapy);

	while(!cap.isOpened() && ros::ok())  
	{  
		ROS_INFO("Cannot connect to camera......");
		ros::spinOnce();
		loop_rate.sleep();
	}  

	while(ros::ok())
	{
		cap>>frame;  
		Mat image_resized;
		Mat image_rect;
		resize(frame, image_resized, Size(640,360));
		
		image_rect = image_resized.clone();
		remap(image_resized, image_rect, mapx, mapy, INTER_LINEAR);

		//frame = imread("/home/chenjie/catkin_ws/src/project_3/images/4.png");
		sensor_msgs::ImagePtr msg;  
		if(!frame.empty())  
		{  
			//imshow("source", frame);
			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_rect).toImageMsg();    
			image_pub.publish(msg);  

			//imshow("origin",image_rect);
			imageDetect test(image_rect);
			test.getThresholdImage();
			test.getPosition();

			if(test.image_valid)
			{
				project_3::Image_info img_msg;
				img_msg.header.stamp = ros::Time::now();
				img_msg.valid = true;
				//img_msg.ROI = cv_bridge::CvImage(std_msgs::Header(), "bgr8", test.image_ROI).toImageMsg();
				img_msg.x = test.ROI_x;
				img_msg.y = test.ROI_y;
				//ROS_INFO("X: %f  Y: %f", img_msg.x, img_msg.y);
				img_msg.theta_valid = test.theta_valid;
				img_msg.theta = test.theta;
				img_msg.center_y = image_rect.rows / 2.0;
				img_msg.center_x = image_rect.cols / 2.0;

				image_info_pub.publish(img_msg);
			}else
			{
				project_3::Image_info img_msg;
				img_msg.header.stamp = ros::Time::now();
				img_msg.valid = false;
				image_info_pub.publish(img_msg);
			}
		}else
		{
			ROS_INFO("Empty image......");
			project_3::Image_info img_msg;
			img_msg.header.stamp = ros::Time::now();
			img_msg.valid = false;
			image_info_pub.publish(img_msg);
		}  

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
