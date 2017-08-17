#include <ros/ros.h>
#include <cv.h>  
#include <highgui.h>   
void myMouseCallback(int event,int x,int y,int flags,void* param)  
{  
    IplImage* img = (IplImage*)param;  
    IplImage* img1 = cvCloneImage(img);  
    CvFont font;  
    char text[20];  
    uchar* ptr;  
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, .8, .8, 0, 1, 8);  
      
    if(event == CV_EVENT_LBUTTONDOWN)  
    {  
        ptr = cvPtr2D(img1, y, x, NULL);  
        sprintf(text,"(%d,%d,%d)",ptr[2],ptr[1],ptr[0]);  
           
        cvPutText(img1,text,cvPoint(x,y),&font,CV_RGB(255,0,255));  
        cvShowImage("MyWindow",img1);  
    }  
}  
int main( int argc, char* argv[] )   
{   
    ros::init(argc, argv, "image_detect");
    IplImage* img = cvLoadImage("/home/ubuntu/Pictures/1.png");  
    cvNamedWindow("MyWindow");  
      
    cvSetMouseCallback("MyWindow",myMouseCallback,(void*)img);  
    cvShowImage("MyWindow",img);  
    cvWaitKey(0);  
    cvDestroyAllWindows();  
    cvReleaseImage(&img);  
    return 0;  
}  