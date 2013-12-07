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
		image_transport::Subscriber image_sub_color;

	public:

		static const int SIZE_OF_IMAGE_ARRAY = 5;
		cv::Mat im_array[SIZE_OF_IMAGE_ARRAY];	//circular array to store 5 images
		int array_ptr;			//circular array pointer
		int nr_of_stored_images; //to check if the image array is "full"

	Contour_Filter(): it_(nh_), array_ptr(0), nr_of_stored_images(0)
	{

		//image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &Contour_Filter::contour_filter, this);
		image_sub_ = it_.subscribe("/camera/depth/image_rect", 1, &Contour_Filter::depth_contour_filter, this);

		//obj_pub_ = nh_.advertise<contour_filter::Objects>("/recognition/detect", 1);
		image_sub_color = it_.subscribe("/camera/rgb/image_color", 1, &Contour_Filter::show_detected_objects, this);

		img_pub_ = it_.advertise("/color_filter/filtered_image", 1);
	}

	~Contour_Filter()
	{
		cv::destroyAllWindows();
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//TODO
	//Translate the coordinates between depth and color so that the rectangle are covering the whole object
	//use the contour to mask out the image instead of the ROI
	void show_detected_objects(const sensor_msgs::ImageConstPtr& msg)
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
			//Drawing the up-right rectangle as the ROI
			//cv::Rect ROI(it->boundRect.x, it->boundRect.y, it->boundRect.width , it->boundRect.height);
			//rectangle(mask, ROI.tl(), ROI.br(), color_white, CV_FILLED); //mask

			//Drawing the found contours of the objects as the ROI
			std::vector<cv::Point> contour_copy = it->contours_poly;

			moveContour(contour_copy, DEPTH_TO_COLOR_DX, DEPTH_TO_COLOR_DY); //move the contour of depth image to match the color image

			std::vector<std::vector<cv::Point> > T = std::vector<std::vector<cv::Point> >(1,contour_copy);
			for (unsigned int i = 0; i < T.size(); i++)
			{
				drawContours( mask, T,i, color_white, CV_FILLED, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
			}

			//increase the size of the contour
			cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));
			cv::dilate(mask, mask, element);

			// Cut out ROI and store it in crop_img
			src.copyTo(crop_img, mask);
		}
		//for some reason the source image get overwritten. Retrieve it back!
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		src = cv_ptr->image.clone();

		if(FLAG_SHOW_IMAGE)
			imshow("Found objects in color image", crop_img);

		//publish image
	    cv_bridge::CvImagePtr img_ptr = cv_ptr;
	    crop_img.copyTo(img_ptr->image);
	    img_pub_.publish(img_ptr->toImageMsg());


	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void moveContour(std::vector<cv::Point>& contour, int dx, int dy)
	{
	    for (size_t i=0; i<contour.size(); i++)
	    {
	        contour[i].x += dx;
	        contour[i].y += dy;
	    }
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

		int m_size = SHADOW_FILTER_MASK_SIZE;
		float threshold = SHADOW_FILTER_THRESHOLD;
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
		cv::Mat src;
		src = cv_ptr->image; //get the source image from the pointer

		//remove top part of image (set to black)
//		int y_remove = 100; float max_depth = 0.75;
//		for(int i=0; i<src.rows; i++)
//		   for(int j=0; j<src.cols; j++)
//		   {
//			   if ( isnan(src.at<float>(i,j)) || i <= y_remove || src.at<float>(i,j) >= max_depth )
//				   src.at<float>(i,j) = 0;
//		   }




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
		//cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11));
		cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
		cv::erode(shadow_bin, shadow_bin, element);
		cv::dilate(shadow_bin, shadow_bin, element);

		/// Find contours
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat shadow_bin_copy;
		shadow_bin.copyTo(shadow_bin_copy); //findContours will change the input image, that's why we create a copy.

		findContours( shadow_bin_copy, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );


		//clear the object vector, and creates a temporary vector storing all detectable things
		objects.clear();
		objects.reserve(contours.size());
		std::vector<DetectedObject> temp_objects;
		temp_objects.reserve(contours.size());
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
			temp_objects.push_back(newObj); //put the new object in the vector called objects
		}


		//merge shadow and object into one object

		if (temp_objects.size() == 2){
			DetectedObject shadow_obj;
			DetectedObject obj;
			//check which one could be the object and which one is its shadow
			if ( contourArea(temp_objects[0].contours_poly) > contourArea(temp_objects[1].contours_poly) ){
				shadow_obj = temp_objects[0];
				obj = temp_objects[1];
			}
			else{
				shadow_obj = temp_objects[1];
				obj = temp_objects[0];
			}
			//go through all points around the smallest contour, if a point is inside the bigger one we know it's its shadow
			for (unsigned int i = 0; i < obj.contours_poly.size(); i++){
				cv::Point2f point = obj.contours_poly[i];
				int is_inside = pointPolygonTest( shadow_obj.contours_poly, point, false );
				if (is_inside == 1)
				{
					//just in case that the shadows middle point is not located on the object, we change the middle point.
					//Then save the shadow as the object
					shadow_obj.mc = obj.mc;
					objects.push_back(shadow_obj);
					break;
				}
			}
		}
		else if(temp_objects.size() != 0)
		{
			objects = temp_objects;
		}



		if (FLAG_SHOW_IMAGE)
		{
			//cv::imshow("Original", src);
			//cv::imshow("src filtered", src_filtered);
			//cv::imshow("shadow filter", shadow_img);
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
					//drawContours( drawing, T,i, color_red, CV_FILLED, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

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



			imshow( "Found contours in depth image", drawing );

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
	puts("Found object images are being sent to /color_filter/filtered_image");
	ros::spin();

	return 0;
}

