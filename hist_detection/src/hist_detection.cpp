/*
 * tracking.cpp
 *
 *  Created on: Oct 2, 2013
 *      Author: robo
 */

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "math.h"
#include <iostream>
#include <stdio.h>
#include "headers/hist_detection.h"
#include <ros/ros.h>
#include <string.h>
#include <sstream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;


void init()
{
	char link[512];
	for(int i = 1; i <= nb_images; i++)
	{
		sprintf(link,"/home/robo/ros_workspace/hist_detection/src/images/%d.jpg",i);

		Mat src, dst;

		/// Load image
		src = imread(link, 1 );

		if( !src.data )
		{ return; }

		/// Separate the image in 3 places ( B, G and R )
		vector<Mat> bgr_planes;
		split( src, bgr_planes );

		/// Establish the number of bins
		int histSize = 256;

		/// Set the ranges ( for B,G,R) )
		float range[] = { 0, 256 } ;
		const float* histRange = { range };

		bool uniform = true; bool accumulate = false;

		Mat b_hist, g_hist, r_hist;

		/// Compute the histograms:
		calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
		calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
		calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

		// Draw the histograms for B, G and R
		int hist_w = 512; int hist_h = 400;
		int bin_w = cvRound( (double) hist_w/histSize );
		Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

		/// Normalize the result tso [ 0, histImage.rows ]
		/*
		normalize(b_hist, b_hist, 0, 255, NORM_MINMAX, -1, Mat() );
		normalize(g_hist, g_hist, 0, 255, NORM_MINMAX, -1, Mat() );
		normalize(r_hist, r_hist, 0, 255, NORM_MINMAX, -1, Mat() );
		 */
		normalize(b_hist, b_hist, norm_hist, 0, NORM_L2, -1, Mat() );
		normalize(g_hist, g_hist, norm_hist, 0, NORM_L2, -1, Mat() );
		normalize(r_hist, r_hist, norm_hist, 0, NORM_L2, -1, Mat() );


		hist[i-1][0] = b_hist;
		hist[i-1][1] = g_hist;
		hist[i-1][2] = r_hist;

		/*
	  /// Draw for each channel
	  for( int i = 1; i < histSize; i++ )
	  {
	      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
	                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
	                       Scalar( 255, 0, 0), 2, 8, 0  );
	      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
	                       Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
	                       Scalar( 0, 255, 0), 2, 8, 0  );
	      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
	                       Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
	                       Scalar( 0, 0, 255), 2, 8, 0  );
	  }

	  /// Display
	  namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
	  imshow("calcHist Demo", histImage );

	  waitKey();
		 */
	}
}


double uniformRandom()
{
	return (double)(rand())/(double)(RAND_MAX);
}


double normalRandom()
{
	// Box-Muller transform
	double u1 = uniformRandom();
	double u2 = uniformRandom();
	return cos(2*M_PI*u2)*sqrt(-2.*log(u1));
}


void object_coordinate()
{
	x = 0;
	y = 0;
	for(int i = 0; i < N; i++)
	{
		x += Px[i];
		y += Py[i];
	}
	x = x/N+sx/2;
	y = y/N+sy/2;

	int table[nb_images+1] = {0,0,0,0,0};
	for(int i = 0; i < N; i++)
	{
		if(n[i] == 10)
		{
			table[4]++;
		}
		else
		{
			table[n[i]%4]++; // !!!!!!!!!!!!!!!!
		}
	}
	int index = 0;
	for(int i = 1; i < nb_images; i++)
	{
		if(table[index] < table[i])
		{
			index = i;
		}
	}
	if(table[4] > 200)
	{
		index = 4;
	}
	object = index;
}


void showCoord(IplImage* img)
{
	if(object != 4)
	{
		for(int i = 0; i <= 10; i++)
		{
			for(int j = 0; j <= 10; j++)
			{
				if((((x+j-5) < height) & ((y+i-5) < width)) & (((x+j-5) >= 0) & ((y+i-5) >= 0)))
				{
					CvScalar s;

					if(object == 0)
					{
						s.val[0]=0;
						s.val[1]=26;
						s.val[2]=204;
					}
					if(object == 1)
					{
						s.val[0]=1;
						s.val[1]=203;
						s.val[2]=16;
					}
					if(object == 2)
					{
						s.val[0]=0;
						s.val[1]=242;
						s.val[2]=255;
					}
					if(object == 3)
					{
						s.val[0]=0;
						s.val[1]=0;
						s.val[2]=0;
					}


					cvSet2D(img,x+j-5,y+i-5,s);
					//img->imageData[((x+j-5)*img->widthStep)+(y+i-5)] = 130;
				}
			}
		}

		CvScalar s;
		s.val[0]=0;
		s.val[1]=0;
		s.val[2]=0;
		cvRectangle(img,cvPoint(y-sx/2,x-sy/2),cvPoint(y+sx/2,x+sy/2),s);
	}
}


void showParticle(IplImage* img)
{
	for(int i = 0; i < N; i++)
	{
		if((((Px[i]+sx/2) < height) & ((Py[i]+sy/2) < width)) & (((Px[i]+sx/2) >= 0) & ((Py[i]+sy/2) >= 0)))
		{
			CvScalar s;
			s.val[0]=0;
			s.val[1]=0;
			s.val[2]=0;
			cvSet2D(img,Px[i]+sx/2,Py[i]+sy/2,s);
			//img->imageData[((Px[i]+sx/2)*img->widthStep)+(Py[i]+sy/2)] = 130;
		}
	}
}


double evaluate(int x, int y, IplImage* img,int n_particule)
{
	double x_rect = x;
	double y_rect = y;

	if(x < 0) {x_rect = 0;}
	if(y < 0) {y_rect = 0;}
	if((x+sy-1) >= height) {x_rect = height-sy;}
	if((y+sx-1) >= width) {y_rect = width-sx;}

	/* sets the Region of Interest*/
	cvSetImageROI(img, cvRect(y_rect,x_rect, sx, sy));
	/* create destination image */
	//IplImage *img2 = cvCreateImage(cvGetSize(img),img->depth,img->nChannels);

	/*
	 * do the main processing with subimage here.
	 */


	/// Separate the image in 3 places ( B, G and R )
	vector<Mat> bgr_planes;
	split( img, bgr_planes );
	/// Establish the number of bins
	int histSize = 256;
	/// Set the ranges ( for B,G,R) )
	float range[] = { 0, 256 } ;
	const float* histRange = { range };
	bool uniform = true; bool accumulate = false;
	Mat b_hist, g_hist, r_hist;
	/// Compute the histograms:
	calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
	calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
	calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );
	/// Normalize the result tso [ 0, histImage.rows ]
	/*
	normalize(b_hist, b_hist, 0, 255, NORM_MINMAX, -1, Mat() );
	normalize(g_hist, g_hist, 0, 255, NORM_MINMAX, -1, Mat() );
	normalize(r_hist, r_hist, 0, 255, NORM_MINMAX, -1, Mat() );
	 */
	normalize(b_hist, b_hist, norm_hist, 0, NORM_L2, -1, Mat() );
	normalize(g_hist, g_hist, norm_hist, 0, NORM_L2, -1, Mat() );
	normalize(r_hist, r_hist, norm_hist, 0, NORM_L2, -1, Mat() );


	double max_error = 0;
	for(int i = 0; i < nb_images; i++)
	{
		// Compare
		double error_b = compareHist(hist[i][0],b_hist,comp);
		double error_g = compareHist(hist[i][1],g_hist,comp);
		double error_r = compareHist(hist[i][2],r_hist,comp);
		//errors[i] = 1-sqrt(error_b*error_b+error_g*error_g+error_r*error_r)/sqrt(3);
		errors[i] = 1/sqrt(error_b*error_b+error_g*error_g+error_r*error_r);
		//errors[i] = sqrt(error_b*error_b+error_g*error_g+error_r*error_r);
		if(errors[i] > max_error)
		{
			max_error = errors[i];
			n[n_particule] = i;

			if(errors[i] < threshold_proba)
			{
				n[n_particule] = 10;
			}
		}
	}

	/* copy subimage */
	//cvCopy(frame, img2, NULL);
	/* always reset the Region of Interest */
	cvResetImageROI(img);


	return max_error;
}


void init_part()
{
	for(int i = 0; i < N; i++)
	{
		Px[i] = rand() % height;
		Py[i] = rand() % width;
		W[i] = 1;
	}
}


void particle_filter(IplImage* img)
{
	// Diffusion
	double diffusion_x [N];
	double diffusion_y [N];

	for(int i = 0; i < N; i++)
	{
		diffusion_x[i] = sigma_diffusion*normalRandom();
		Px[i] += diffusion_x[i];
		if(Px[i] < 0) {Px[i] = 0;}
		if(Px[i] >= height-sx+1) {Px[i] = height-sx;}

		diffusion_y[i] = sigma_diffusion*normalRandom();
		Py[i] += diffusion_y[i];
		if(Py[i] < 0) {Py[i] = 0;}
		if(Py[i] >= width-sy+1) {Py[i] = width-sy;}
	}

	// Weighting
	double norm = 0;
	for(int i = 0; i < N; i++)
	{
		W[i] = evaluate(Px[i],Py[i],img,i);
		norm += W[i];
	}
	for(int i = 0; i < N; i++)
	{
		W[i] = W[i]/norm;
	}

	// Resampling
	double cdf[N];
	cdf[0] = W[0];
	for(int i = 1; i < N; i++)
	{
		cdf[i] = cdf[i-1] + W[i];
	}

	double r = uniformRandom()/N;
	for(int i = 0; i < N; i++)
	{
		for(int j = 0; j < N; j++)
		{
			if(cdf[j] >= r)
			{
				Px[i] = Px[j];
				Py[i] = Py[j];
				n[i] = n[j];
				break;
			}
		}
		W[i] = (double)1/N;
		r += (double)1/N;
	}

	// Update object coordinate
	object_coordinate();
}


class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;


public:
	IplImage* img;


	ImageConverter()
	: it_(nh_)
	{
		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
	}


	~ImageConverter()
	{
	}


	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		img = new IplImage(cv_ptr->image);

		// Filter
		particle_filter(img);

		// Print
		showParticle(img);

		if(current_object == object)
		{
			if(cmpt < detector)
			{
				cmpt++;
			}
			else
			{
				showCoord(img);
			}
		}
		else
		{
			current_object = object;
			cmpt = 0;
		}


		cvNamedWindow("Window",1); cvShowImage("Window", img);
		//cvWaitKey();
		cvWaitKey(10);
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "Hist_detection");
	ros::NodeHandle nh;

	// Init
	init();

	ImageConverter ic;
	ros::spin();

	cvDestroyAllWindows();

	return 0;
}
