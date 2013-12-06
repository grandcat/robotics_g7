#include "contour_filter.h"

class Contour_Filter
{
	private:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		ros::Publisher obj_pub_;
		image_transport::Publisher img_pub_;

		std::vector<DetectedObject> objects;

	public:

		static const int SIZE_OF_IMAGE_ARRAY = 5;
		cv::Mat im_array[SIZE_OF_IMAGE_ARRAY];	//circular array to store 5 images
		int array_ptr;			//circular array pointer
		int nr_of_stored_images; //to check if the image array is "full"

	Contour_Filter(): it_(nh_), array_ptr(0), nr_of_stored_images(0)
	{

		//image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &Contour_Filter::contour_filter, this);
		image_sub_ = it_.subscribe("/camera/depth/image_rect", 1, &Contour_Filter::depth_contour_filter, this);
		//img_pub_ = it_.advertise("/contour_filter/filtered_image", 1);
		//obj_pub_ = nh_.advertise<contour_filter::Objects>("/recognition/detect", 1);
	}

	~Contour_Filter()
	{
		cv::destroyAllWindows();
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	cv::Mat remove_noise(const cv::Mat& src, int iter = 0, int kernel_size = 9)
	{
		//only start to remove the noise if we have enough previous images saved
		if (nr_of_stored_images < SIZE_OF_IMAGE_ARRAY)
		{
			im_array[array_ptr] = src;
			array_ptr++;
			nr_of_stored_images++;
			if (array_ptr > SIZE_OF_IMAGE_ARRAY-1)
				array_ptr = 0;
			//std::cout<<"ptr: "<<array_ptr<<"stored_images: "<<nr_of_stored_images<<std::endl;
			return src; //simply return the current image and don't remove the noise
		}

		//we have enough images stored to remove the background noise by comparing previous images background
		cv::Mat noise_free_im = src.clone();
		for (int im_i = 0; im_i < SIZE_OF_IMAGE_ARRAY; im_i++ )
		{
			for (int i = 0; i< src.rows; i++)
			{
				for (int j=0; j < src.cols; j++)
				{
					/*
					std::cout<<"min( "<<noise_free_im.at<float>(i,j)<<", "<<im_array[im_i].at<float>(i,j)<<"): ";
					if ( isnan(noise_free_im.at<float>(i,j)) || isnan(im_array[im_i].at<float>(i,j)) )
						std::cout<< 0.0/0.0 <<std::endl;
					else
						std::cout<< min( noise_free_im.at<float>(i,j), im_array[im_i].at<float>(i,j)  ) <<std::endl;
					*/
					if ( isnan(noise_free_im.at<float>(i,j)) || isnan(im_array[im_i].at<float>(i,j)) )
						noise_free_im.at<float>(i,j) = 0;
					else
						noise_free_im.at<float>(i,j) = std::min( noise_free_im.at<float>(i,j), im_array[im_i].at<float>(i,j)  );
				}
			}
		}

		//save current image for future noise elimination
		im_array[array_ptr] = src;
		array_ptr++;
		if (array_ptr > SIZE_OF_IMAGE_ARRAY-1)
			array_ptr = 0;

		//remove extra noise by using the closing operation ( dilation(erosion(image) )
		//and then do some extra dilation to increase the size of everything
		if (iter != 0)
		{
			cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size), cv::Point(-1,-1) ); //create the convolution matrix
			cv::morphologyEx(noise_free_im, noise_free_im, CV_MOP_OPEN, element); //OPEN(src) := dilate( erode(src) )
			cv::dilate(noise_free_im, noise_free_im, element, cv::Point(-1, -1), iter, 1, 1);
		}

		return noise_free_im;

	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//src is a depth image
	void shadow_filter(const cv::Mat &src, cv::Mat &dst)
	{
		dst = src.clone();

		int m_size = 7;
		float threshold = 0.02;
		//float threshold = 0.05;
		for(int y_c=0; y_c<src.rows; y_c++){
		   for(int x_c=0; x_c<src.cols; x_c++){
			   //center is at (y_c,x_c) , and we are guaranteed to be able to move floor(m_size/2) pixel away from it
			   //if (x_c > 0 && y_c > 0 && x_c<src.cols-1 && y_c<src.rows-1){
			   if (x_c > (int)(m_size/2)-1 && y_c > (int)(m_size/2)-1 && x_c<src.cols-(int)(m_size/2)-1 && y_c<src.rows-(int)(m_size/2)-1){
				   //find the max and min depth value inside the 3x3 mask
				   float max = 0.0;
				   float min = 4711.0; //a "lagom" big number
				   for (int y = y_c-(int)(m_size/2); y < y_c+(int)(m_size/2); y++){
					   for (int x = x_c-(int)(m_size/2); x < x_c+(int)(m_size/2); x++){
						   if (src.at<float>(y,x) > max)
							   max = src.at<float>(y,x);
						   if (src.at<float>(y,x) < min)
							   min = src.at<float>(y,x);
					   }
				   }
				   //if we have detected a edge in the depth image, create a shadow
				   if ( (max-min) > threshold && min > 0)
				   {
					   float mean = (max+min)/2;
					   for (int y = y_c-(int)(m_size/2); y < y_c+(int)(m_size/2); y++){
						   for (int x = x_c-(int)(m_size/2); x < x_c+(int)(m_size/2); x++){
							   if (src.at<float>(y,x) > mean)
								   dst.at<float>(y,x)  = 0;
						   }
					   }
				   }
			   }
			   //done with one mask

		   }
		}
		//done with the outher loop
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void depth_contour_filter(const sensor_msgs::ImageConstPtr& msg)
	{
                //ros::Duration(0.5).sleep();

		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			//cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

//http://docs.opencv.org/doc/tutorials/imgproc/shapedescriptors/find_contours/find_contours.html
		cv::Mat src, src_gray, src_gray_blur;
		src = cv_ptr->image; //get the source image from the pointer

		//remove top part of image (set to black)
		int y_remove = 100; float max_depth = 0.75;
		for(int i=0; i<src.rows; i++)
		   for(int j=0; j<src.cols; j++)
		   {
			   if ( isnan(src.at<float>(i,j)) || i <= y_remove || src.at<float>(i,j) >= max_depth )
				   src.at<float>(i,j) = 0;
		   }




		cv::Mat shadow_img;
		//cv::Mat src_filtered = remove_noise(src);
		//shadow_filter(src_filtered, shadow_img);
		shadow_filter(src, shadow_img);


		cv::Mat shadow_bin_raw;//(src.size(), CV_8U);
		// threshold to make it binary
		threshold(shadow_img, shadow_bin_raw, 0, 255, CV_THRESH_BINARY);
		cv::Mat shadow_bin;
		shadow_bin_raw.convertTo(shadow_bin,CV_8U);

		//Does some erosion and dilation to remove some of the pixels
		cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
		cv::erode(shadow_bin, shadow_bin, element);
		cv::dilate(shadow_bin, shadow_bin, element);

		/// Find contours
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat shadow_bin_temp, sobel_img_temp;
		shadow_bin.copyTo(shadow_bin_temp); //findContours will change the input image, that's why we create a copy.

		findContours( shadow_bin_temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );


		//clear the object vector, and creates a temporary vector storing all detectable things
		objects.clear();
		objects.reserve(contours.size());

		for(unsigned int i = 0; i < contours.size(); ++i )
		{
			DetectedObject newObj;
			double area = contourArea(contours[i]);
			if (area < minArea || area > maxArea) //if area is not "lagom", ignore it
				continue;

			newObj.rotatedRect = minAreaRect( cv::Mat(contours[i]) );

			double ratio = (double)std::max( newObj.rotatedRect.size.height , newObj.rotatedRect.size.width ) /
						 (double)std::min( newObj.rotatedRect.size.height , newObj.rotatedRect.size.width );
			if (ratio > minRatio) //if it's a too narrow rectangle, ignore it
				continue;

			newObj.contours_poly = contours[i];
			newObj.boundRect = boundingRect( cv::Mat(contours[i]) );

			/// Get the mass centers using the moment of each contour
			cv::Moments mu = moments( contours[i], false );

			newObj.mc = cv::Point( (int)mu.m10/mu.m00 , (int)mu.m01/mu.m00 );
			objects.push_back(newObj); //put the new object in the vector called objects
		}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//
//		  //crop out the objects from the image
//		  cv::Mat crop_img = cv::Mat::zeros( src_image.size(), src_image.type());
//		  cv::Mat mask = cv::Mat::zeros( src_image.size(), src_image.type());
//		  cv::Scalar color_white = cv::Scalar( 255, 255, 255);
//		  for(std::vector<DetectedObject>::iterator it = objects.begin(); it != objects.end(); ++it)
//		  {
//			  cv::Rect ROI(it->boundRect.x, it->boundRect.y, it->boundRect.width , it->boundRect.height);
//			  rectangle(mask, ROI.tl(), ROI.br(), color_white, CV_FILLED); //mask
//
//			  // Cut out ROI and store it in crop_img
//			  src_image.copyTo(crop_img, mask);
//		  }
//		  //for some reason the source image get overwritten. Retrieve it back!
//		  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//		  src_image = cv_ptr->image.clone();



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		if (FLAG_SHOW_IMAGE)
		{
			cv::imshow("Original", src);
			//cv::imshow("src filtered", src_filtered);
			//cv::imshow("shadow filter", shadow_img);
			//cv::imshow("shadow binary", shadow_bin);

			cv::Mat drawing = cv::Mat::zeros( src.size(), CV_8UC3 );
			cv::Scalar color_red = cv::Scalar( 0, 0, 255 );
			cv::Scalar color_blue = cv::Scalar( 255, 0, 0);
			cv::Scalar color_white = cv::Scalar( 255, 255, 255);


			cv::Point2f rect_points[4];
			for(std::vector<DetectedObject>::iterator it = objects.begin(); it != objects.end(); ++it)
			{
				//draw the mass centers
				line(drawing, it->mc, it->mc, color_white, 10, 8, 0);

				rectangle( drawing, it->boundRect.tl(), it->boundRect.br(), color_red, 2, 8, 0 );

				//draw rotatedRect
				it->rotatedRect.points( rect_points );
				for( int j = 0; j < 4; j++ )
					line( drawing, rect_points[j], rect_points[(j+1)%4], color_blue, 1, 8 );

				//http://opencv-users.1802565.n2.nabble.com/draw-only-one-contour-using-drawContours-td7404400.html
				std::vector<std::vector<cv::Point> > T = std::vector<std::vector<cv::Point> >(1,it->contours_poly);
				for (unsigned int i = 0; i < T.size(); i++)
				{
					drawContours( drawing, T,i, color_red, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );

				}

			}

//			cv::Mat draw = cv::Mat::zeros( 500,500, CV_8UC3 );
//			int contour_id = 1;
//			for(int i = 0; i< contours[contour_id].size(); i++)
//			{
//			    std::cout << contours[contour_id][i] << std::endl;
//			//Now draw a every point as a circle with radius = 1
//			    cv::circle(draw,contours[contour_id][i],1,cv::Scalar(100,100,255));
//			}





			imshow( "Found objects", drawing );

			cv::waitKey(3);
		}



	}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void contour_filter(const sensor_msgs::ImageConstPtr& msg)
	{
                //ros::Duration(0.5).sleep();

		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

//http://docs.opencv.org/doc/tutorials/imgproc/shapedescriptors/find_contours/find_contours.html
		cv::Mat src, src_gray, src_gray_blur;
		src = cv_ptr->image; //get the source image from the pointer

		//remove top part of image (set to black)
		int y_remove = 100;
		for(int i=0; i<y_remove; i++)
		   for(int j=0; j<src.cols; j++)
		   {
			   src.at<cv::Vec3b>(i,j)[0] = 0;
			   src.at<cv::Vec3b>(i,j)[1] = 0;
			   src.at<cv::Vec3b>(i,j)[2] = 0;
		   }

		cv::cvtColor(src, src_gray, CV_BGR2GRAY); //change color image to gray-scale
		cv::blur( src_gray, src_gray_blur, cv::Size(5,5) ); //blur the gray-scale image

		cv::Mat canny_img, sobel_img;
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		/// Detect edges using canny
		int min_thresh = 0;
		int max_thresh = 255;
		cv::Canny( src_gray_blur, canny_img, min_thresh, max_thresh, 5 );

		//Detect edges using sobel filter
		int ddepth = -1; //same depth as source
		int dx = 1;
		int dy = 1;
		int ksize = 3;
		cv::Sobel(src_gray_blur, sobel_img, ddepth, dx, dy, ksize);

		/// Find contours
		findContours( sobel_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		std::vector< std::vector<cv::Point> > contours_obj;
		std::vector<cv::Point> cur_contours_poly;
		contours_obj.reserve(contours.size()); //reserve space
		for(unsigned int i = 0; i < contours.size(); i++ )
		{
			approxPolyDP( cv::Mat(contours[i]), cur_contours_poly, 3, true );
			double area = contourArea(cur_contours_poly);
			//std::cout<<"Area of contour nr "<<i<<": "<<area<<std::endl;
			if (area >= minArea && area<= maxArea) //save the closed approximated contour if it's area is "lagom"
				contours_obj.push_back(cur_contours_poly);
		}

		/// Draw polygonal contour + bonding rectangles
		cv::Mat drawing = cv::Mat::zeros( src.size(), CV_8UC3 );
		for(unsigned int i = 0; i< contours_obj.size(); i++ )
		{
			cv::Scalar color = cv::Scalar( 0, 0, 255 );
			drawContours( drawing, contours_obj, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
			//drawContours( drawing, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
		}




		//cv::imshow("src", src);
		cv::imshow("gray-scale", src_gray_blur);
		//cv::imshow("canny edge", canny_img);
		cv::imshow("sobel edge", sobel_img);
		cv::imshow("contours", drawing);
		cv::waitKey(3);





	}

};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "contour_filter");
	if (argc != 1)
	{
		if (atoi(argv[1]) == 1)
		{
			FLAG_SHOW_IMAGE = true;
		}
	}



	Contour_Filter cf;
	puts("Contour filter is up and running!");
	puts("Message is being sent to /recognition/detect");
	ros::spin();

	return 0;
}

