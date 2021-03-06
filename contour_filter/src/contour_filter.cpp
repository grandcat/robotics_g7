#include "contour_filter.hpp"


void Contour_Filter::show_detected_objects(const sensor_msgs::ImageConstPtr& msg)
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


	cv::Mat src;
	src = cv_ptr->image; //get the source image from the pointer

	//crop out the objects from the image
	cv::Mat crop_img = cv::Mat::zeros( src.size(), src.type());
	cv::Mat mask = cv::Mat::zeros( src.size(), src.type());
	cv::Scalar color_white = cv::Scalar( 255, 255, 255);
	for(std::vector<DetectedObject>::iterator it = objects.begin(); it != objects.end(); ++it)
	{

		//Drawing the found contours of the objects as the ROI
		std::vector<cv::Point> contour_copy = it->contours_poly;

		moveContour(contour_copy, DEPTH_TO_COLOR_DX, DEPTH_TO_COLOR_DY); //move the contour of depth image to match the color image

		std::vector<std::vector<cv::Point> > T = std::vector<std::vector<cv::Point> >(1,contour_copy);
		for (unsigned int i = 0; i < T.size(); i++)
		{
			drawContours( mask, T,i, color_white, CV_FILLED, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
		}

		// Cut out ROI and store it in crop_img
		src.copyTo(crop_img, mask);
	}
	//The source image gets overwritten. Retrieve it back!
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	src = cv_ptr->image.clone();


	//publish image
	cv_bridge::CvImagePtr img_ptr = cv_ptr;
	crop_img.copyTo(img_ptr->image);
	img_pub_.publish(img_ptr->toImageMsg());


	if(FLAG_SHOW_IMAGE){
		imshow("Found objects in color image", crop_img);}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Contour_Filter::moveContour(std::vector<cv::Point>& contour, int dx, int dy)
{
	for (size_t i=0; i<contour.size(); i++)
	{
		contour[i].x += dx;
		contour[i].y += dy;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Mat Contour_Filter::remove_noise(const cv::Mat& src, int iter = 0, int kernel_size = 9)
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
cv::Mat Contour_Filter::shadow_filter(const cv::Mat &src)
{
	cv::Mat dst = src.clone();

	int m_size = SHADOW_FILTER_MASK_SIZE;
	float threshold_value = SHADOW_FILTER_THRESHOLD;
	for(int y_c=0; y_c<src.rows; y_c++){
	   for(int x_c=0; x_c<src.cols; x_c++){
		   //center is at (y_c,x_c) , and we are guaranteed to be able to move floor(m_size/2) pixel away from it
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
			   if ( (max-min) > threshold_value && min > 0)
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
	//done with the outer loop


	//create a binary representation
	cv::Mat shadow_bin_raw, shadow_bin;
	// threshold to make it binary
	threshold(dst, shadow_bin_raw, 0, 255, CV_THRESH_BINARY);
	shadow_bin_raw.convertTo(shadow_bin,CV_8U);


	//Does some erosion and dilation to remove some of the pixels
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(SHADOW_FILTER_ERODE_DILATE_SIZE, SHADOW_FILTER_ERODE_DILATE_SIZE));
	cv::erode(shadow_bin, shadow_bin, element);
	cv::dilate(shadow_bin, shadow_bin, element);

	return shadow_bin;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<cv::Point> > Contour_Filter::detect_good_contours(cv::Mat &src_binary)
{
	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > contours_output;
	std::vector<cv::Vec4i> hierarchy;

	findContours( src_binary, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
	contours_output.reserve(contours.size());

	for(unsigned int i = 0; i < contours.size(); ++i )
	{
		double area = contourArea(contours[i]);
		if (area < minArea || area > maxArea) //if area is not "lagom", ignore it
			continue;

		cv::RotatedRect rotatedRect = minAreaRect( cv::Mat(contours[i]) );

		double ratio = (double)std::max( rotatedRect.size.height , rotatedRect.size.width ) /
					 (double)std::min( rotatedRect.size.height , rotatedRect.size.width );
		if (ratio > minRatio) //if it's a too narrow rectangle, ignore it
			continue;

		contours_output.push_back(contours[i]);
	}

	return contours_output;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<cv::Point> > Contour_Filter::merge_contours(std::vector<std::vector<cv::Point> > &contours)
{
	//merge overlapping contours
	cv::Mat temp_drawing = cv::Mat::zeros( cv::Size(640,480), CV_8UC1 );

	for(unsigned int i = 0; i< contours.size(); i++ )
	{
		   drawContours( temp_drawing, contours, i, 255, CV_FILLED, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
	}


	//increase the size of the contour a little bit in case it didn't detect the shadow of the object
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));
	cv::dilate(temp_drawing, temp_drawing, element);


	std::vector<std::vector<cv::Point> > merged_contours;
	std::vector<cv::Vec4i> merged_hierarchy;
	findContours( temp_drawing, merged_contours, merged_hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );

	return merged_contours;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Contour_Filter::depth_contour_filter(const sensor_msgs::ImageConstPtr& msg)
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

	cv::Mat src;
	src = cv_ptr->image; //get the source image from the pointer

	//cv::Mat src_filtered = remove_noise(src);
	//shadow_filter(src_filtered, shadow_img);
	cv::Mat shadow_bin = shadow_filter(src); //gets a binary image where the shadow contours of the objects are drawn

	/// Find contours
	cv::Mat shadow_bin_copy;
	shadow_bin.copyTo(shadow_bin_copy); //found_contours will change the input image, that's why we create a copy.
	std::vector<std::vector<cv::Point> > found_contours =  detect_good_contours(shadow_bin_copy);
	std::vector<std::vector<cv::Point> > merged_contours = merge_contours (found_contours);

	objects.clear();
	objects.reserve(merged_contours.size());

	//save the contours
	for (unsigned int i = 0; i < merged_contours.size(); ++i){
		DetectedObject newObj;
		newObj.rotatedRect = minAreaRect( cv::Mat(merged_contours[i]) );
		newObj.contours_poly = merged_contours[i];
		newObj.boundRect = boundingRect( cv::Mat(merged_contours[i]) );

		/// Get the mass centers using the moment of each contour
		cv::Moments mu = moments( merged_contours[i], false );
		newObj.mc = cv::Point( (int)mu.m10/mu.m00 , (int)mu.m01/mu.m00 );

		objects.push_back(newObj); //put the new object in the vector called objects
	}


        if (FLAG_SHOW_IMAGE)
	{
		std::cout<<"Found "<<objects.size()<<" objects"<<std::endl;
		for (unsigned int i = 0; i < objects.size(); ++i)
		{
			std::cout<<i+1<<" mc: "<<objects[i].mc<<" contour area: "<<contourArea(objects[i].contours_poly)<<std::endl;
		}
		std::cout<<"\n\n";
		cv::imshow("Original", src);
		cv::imshow("shadow binary", shadow_bin);

		cv::Mat drawing = cv::Mat::zeros( src.size(), CV_8UC3 );
		cv::Scalar color_red = cv::Scalar( 0, 0, 255 );
		cv::Scalar color_blue = cv::Scalar( 255, 0, 0);
		cv::Scalar color_white = cv::Scalar( 255, 255, 255);

		cv::Point2f rect_points[4];
		for(std::vector<DetectedObject>::iterator it = objects.begin(); it != objects.end(); ++it)
		{
			//draw the mass centers
			line(drawing, it->mc, it->mc, color_white, 10, 8, 0);

			//draw rectangle
			rectangle( drawing, it->boundRect.tl(), it->boundRect.br(), color_red, 2, 8, 0 );

			//draw rotatedRect
			it->rotatedRect.points( rect_points );
			for( int j = 0; j < 4; j++ )
				line( drawing, rect_points[j], rect_points[(j+1)%4], color_blue, 1, 8 );

			//draw contours
			std::vector<std::vector<cv::Point> > T = std::vector<std::vector<cv::Point> >(1,it->contours_poly);
			for (unsigned int i = 0; i < T.size(); i++)
			{
				drawContours( drawing, T,i, color_red, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
			}

		}
		imshow( "Found contours in depth image", drawing );

		cv::waitKey(3);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{


  ros::init(argc, argv, "contour_filter");
  Contour_Filter cf;
	if (argc != 1)
	{
		if (atoi(argv[1]) == 1)
		{
                        cf.showDebugOutput();
		}
	}

	std::cout<<"Contour filter is up and running!"<<std::endl;
	std::cout<<"Found object images are being sent to /color_filter/filtered_image"<<std::endl;
	ros::spin();

	return 0;
}


