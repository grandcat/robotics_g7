#include <ros/ros.h>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

//static const char WINDOW[] = "Image window";
idImg testImg[];



struct idImg 
{
	Mat img;		
	int obj;
}

struct imgHist 
{
	Mat bHist;
	Mat gHist;
	Mat rHist;
}



/*
 * Try to identify the object by comparing edge detection to train images
 */
void identifyObject(Mat edges)
{

}


/*
 * Train on some images
 */
void trainOnImages()
{

	testImg[0] = { .img = imread("/testimages/1_1.jpg"), .obj = 1 };
	testImg[1] = { .img = imread("/testimages/2_1.jpg"), .obj = 2 };
	testImg[2] = { .img = imread("/testimages/3_1.jpg"), .obj = 3 };
	testImg[3] = { .img = imread("/testimages/3_2.jpg"), .obj = 3 };
	testImg[4] = { .img = imread("/testimages/3_3.jpg"), .obj = 3 };
	testImg[5] = { .img = imread("/testimages/4_1.jpg"), .obj = 4 };
	testImg[6] = { .img = imread("/testimages/4_2.jpg"), .obj = 4 };
	testImg[7] = { .img = imread("/testimages/4_3.jpg"), .obj = 4 };

}



void colorDetection(Mat matImg)
{
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
void edgeDetection(Mat img)
{
	//noise reduction with gaussian filter
	Mat edges;
	int ksize = 5;
	blur(img, edges, Size(ksize, ksize));

	// set tresholds
	int lowThreshold = 100;
	int upperThreshold = lowThreshold*3;

	// Canny Edge Detection
	Canny(edges, edges, lowThreshold, upperThreshold, ksize);
	Mat finalImg;
	finalImg.create( img.size(), img.type() );
	finalImg = Scalar::all(0);
	img.copyTo(finalImg, edges);

	imshow("Edge map", finalImg);

	identifyObject(edges);
}



int main( int argc, char** argv )
{
	ros::init(argc, argv, "recognition");
	ros::NodeHandle n;

	// Load an image
	Mat src = imread( argv[1] );

	if( !src.data )
	{ return -1; }

	/// Convert the image to grayscale
	cvtColor( src, src, CV_BGR2GRAY );

	/// Show the image
	edgeDetection(src);

	/// Wait until user exit program by pressing a key
	waitKey(0);

	return 0;
}
