#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";


void colorDetection(IplImage img)
{
    Mat matImg(img);

	//Split image into planes (B, G, R)
	vector<Mat> planes;
	split( matImg, planes );

	int histSize = 256;

	// Set ranges for B,G,R
	float range[] = { 0, 256 } ; //the upper boundary is exclusive
	const float* histRange = { range };
	
	bool uniform = true; 
	bool accumulate = false;

	// Calculate histograms for B,G,R
	Mat bHist, gHist, rHist;
	calcHist(&planes[0], 1, 0, Mat(), bHist, 1, &histSize, &histRange, uniform, accumulate);
	calcHist(&planes[1], 1, 0, Mat(), gHist, 1, &histSize, &histRange, uniform, accumulate);
	calcHist(&planes[2], 1, 0, Mat(), rHist, 1, &histSize, &histRange, uniform, accumulate);


	// Draw histograms (for testing)
	int histW = 512; 
	int histH = 400;
	int binW = cvRound( (double) histW/histSize );

	Mat histImage( histH, histW, CV_8UC3, Scalar( 0,0,0) );

	normalize(bHist, bHist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	normalize(gHist, gHist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	normalize(rHist, rHist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

	for( int i = 1; i < histSize; i++ )
	{
	  line( histImage, Point( binW*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
		               Point( binW*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
		               Scalar( 255, 0, 0), 2, 8, 0  );
	  line( histImage, Point( binW*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
		               Point( binW*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
		               Scalar( 0, 255, 0), 2, 8, 0  );
	  line( histImage, Point( binW*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
		               Point( binW*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
		               Scalar( 0, 0, 255), 2, 8, 0  );
	}
	namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
	imshow("calcHist Demo", histImage );
}




/*
 * Perform Canny Edge Detection on the image
 */
void edgeDetection(IplImage img)
{
    Mat matImg(img);

	//noise reduction with gaussian filter
	Mat edges;
	int ksize = 5;
	blur(matImg, edges, Size(ksize, ksize));

	// set tresholds
	int lowThreshold = 100;
	int upperThreshold = lowThreshold*3;

	// Canny Edge Detection
	Canny(edges, edges, lowThreshold, upperThreshold, ksize);
	Mat finalImg;
	finalImg.create( matImg.size(), matImg.type() );
	finalImg = Scalar::all(0);
	img.copyTo(finalImg, edges);

	imshow("Edge map", finalImg);

	//identifyObject(edges);
}


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  IplImage* img;

  ImageConverter()
    : it_(nh_)
  {

	image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
	image_sub_depth_ = it_.subscribe("/camera/depth/image_rect", 1, &ImageConverter::imageCb_depth, this);

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);	//convert to grayscale
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //cv_ptr has image now, identify object
	img = new IplImage(cv_ptr->image);
	edgeDetection(img);

    cv::imshow(WINDOW, cv_ptr->image);
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

