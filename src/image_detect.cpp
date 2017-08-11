#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace cv;
using namespace std;

class imageDetect
{
private:
	Mat image;
	Mat image_threshold;
	Mat image_contour;
public:
	imageDetect(Mat src)
	{
		image = src;
		image_threshold = Mat(image.size(), CV_8UC1);
	};
	~imageDetect(){};

	void getThresholdImage()
	{
		GaussianBlur(image, image, Size(7, 7), 0, 0);  
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
		// Mat element = getStructuringElement(MORPH_RECT, Size(1,1));
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
		vector<Rect> rb;
		findContours(image_threshold, contours, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_NONE);
		
		/*erase unqualified contours*/
		vector<vector<Point> >::iterator it = contours.begin();
		while(it != contours.end())
		{
			if (contourArea(*it) < 500)
			{
				it = contours.erase(it);
			}
			else
			{

				Rect r = boundingRect(Mat(*it));
				int d = r.width / r.height;
				if ( d < 0.5 || d > 1.5)
					it = contours.erase(it);
				else {
					rb.push_back(r);
					rectangle(image, r, Scalar(0, 255, 0), 3);
					++it;
				}
			}
		}

		Mat result(image.size() , CV_8U , cv::Scalar(0)) ; 
		Mat result_2(image.size() , CV_8U , cv::Scalar(0)) ; 
		drawContours(result , contours , -1 , cv::Scalar(255) , 3) ;  

		// vector<cv::Vec2f> lines;
		// HoughLines(result, lines, 1, CV_PI/180, 150, 0, 0 );  
		// vector<cv::Vec2f>::const_iterator it_l= lines.begin();
		// while (it_l!=lines.end())
		// {
		// 	float rho= (*it_l)[0];   // 表示距离
		// 	float theta= (*it_l)[1]; // 表示角度

		// 	if (theta < CV_PI/4. || theta > 3*CV_PI/4.) // 若检测为垂直线
		// 	{ 
		// 		// 得到线与第一行的交点
		// 		cv::Point pt1(rho/cos(theta),0);
		// 		// 得到线与最后一行的交点
		// 		cv::Point pt2((rho-result_2.rows*sin(theta))/cos(theta),result_2.rows);
		// 		// 调用line函数绘制直线
		// 		cv::line(result_2, pt1, pt2, cv::Scalar(255), 1);
		// 	} else // 若检测为水平线
		// 	{ 
		// 		// 得到线与第一列的交点
		// 		cv::Point pt1(0,rho/sin(theta));
		// 		// 得到线与最后一列的交点
		// 		cv::Point pt2(result_2.cols,(rho-result_2.cols*cos(theta))/sin(theta));
		// 		// 调用line函数绘制直线
		// 		cv::line(result_2, pt1, pt2, cv::Scalar(255), 1);
		// 	}
		// 	++it_l;
		// }

		vector<Vec4i> lines;  
		HoughLinesP(result, lines, 1, CV_PI/180, 100, 20, 100 );  
		for( size_t i = 0; i < lines.size(); i++ )  
		{  
		    line( result_2, Point(lines[i][0], lines[i][1]),  
		        Point(lines[i][2], lines[i][3]), Scalar(255), 1);  
		}  
			 
		image_contour = result_2;
		imshow("resultImage" , image_contour) ;   
		waitKey(1);
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
		waitKey(0);
	};
		
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_detect");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	Mat image = imread("/home/chenjie/catkin_ws/src/project_3/images/1.jpg");
	imageDetect test(image);
	test.getThresholdImage();
	test.getPosition();
	test.getCorner();

	
	//while(ros::ok())
	//{

	//	imshow("test",image);
	//	waitKey(1);
	//	ros::spinOnce();
	//	loop_rate.sleep();
	//}
	return 0;
}