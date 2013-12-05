#include "contour_filter.h"

class Contour_Filter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher obj_pub_;
	image_transport::Publisher img_pub_;

	public:

	static const int SIZE_OF_IMAGE_ARRAY = 5;
	cv::Mat im_array[SIZE_OF_IMAGE_ARRAY];	//circular array to store 5 images
	int array_ptr;			//circular array pointer
	int nr_of_stored_images; //to check if the image array is "full"

	Contour_Filter(): it_(nh_)
	{
		array_ptr = 0;
		nr_of_stored_images = 0;

		//image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &Contour_Filter::contour_filter, this);
		image_sub_ = it_.subscribe("/camera/depth/image_rect", 1, &Contour_Filter::depth_contour_filter, this);
		//img_pub_ = it_.advertise("/contour_filter/filtered_image", 1);
		//obj_pub_ = nh_.advertise<contour_filter::Objects>("/recognition/detect", 1);
	}

	~Contour_Filter()
	{
		cv::destroyAllWindows();
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
	//TODO
	//Testa Utso binary för att ta bort bakgrunden så mycket som möjligt.
	//watershed (dilate och erode två separata bilder för att utskilja konturerna mellan för,- och bakgrund)
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


		cv::Mat src_filtered = remove_noise(src);

		cv::Mat shadow_img;
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

		//Detect edges using sobel filter
//		cv::Mat sobel_img_raw, sobel_img_thresh, sobel_img;
//		int ddepth = -1; //same depth as source
//		int dx = 1;
//		int dy = 1;
//		int ksize = 3;
//		cv::Sobel(shadow_bin, sobel_img_raw, ddepth, dx, dy, ksize);
//		threshold(sobel_img_raw, sobel_img_thresh, 0, 255, CV_THRESH_BINARY);
//		sobel_img_thresh.convertTo(sobel_img,CV_8U);


		/// Find contours
		///*
		//src_bin = src > 0;
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat shadow_bin_temp, sobel_img_temp;
		shadow_bin.copyTo(shadow_bin_temp);
		//sobel_img.copyTo(sobel_img_temp);
		//findContours( sobel_img_temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

		findContours( shadow_bin_temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
		//findContours( shadow_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		std::vector< std::vector<cv::Point> > contours_obj;
		contours_obj.reserve(contours.size()); //reserve space
		for(unsigned int i = 0; i < contours.size(); i++ )
		{
			double area = contourArea(contours[i]);
			//std::cout<<"Area of contour nr "<<i<<": "<<area<<std::endl;
			if (area >= minArea && area<= maxArea) //save the closed approximated contour if it's area is "lagom"
			{
				contours_obj.push_back(contours[i]);
			}

		}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//		//remove objects that are to narrow to be a real object
//		for(std::vector<DetectedObject>::iterator it = temp_objects.begin(); it != temp_objects.end(); ++it)
//		{
//
//		  it->rotatedRect = minAreaRect( cv::Mat(it->contours_poly) );
//
//		  double ratio = (double)std::max( it->rotatedRect.size.height , it->rotatedRect.size.width ) /
//						 (double)std::min( it->rotatedRect.size.height , it->rotatedRect.size.width );
//
//		  if (ratio > minRatio) //if it's a too narrow rectangle, ignore it
//			continue;
//
//		  it->boundRect = boundingRect( cv::Mat(it->contours_poly) );
//
//		  /// Get the mass centers using the moment of each contour
//		  cv::Moments mu = moments( it->contours_poly, false );
//
//		  it->mc = cv::Point( (int)mu.m10/mu.m00 , (int)mu.m01/mu.m00 );
//		  objects.push_back(*it); //copy the object and put it in the vector called objects
//
//		}
//
//		//crop out the objects from the image
//
//
//		cv::Mat crop_img = cv::Mat::zeros( src_image.size(), src_image.type());
//		cv::Mat mask = cv::Mat::zeros( src_image.size(), src_image.type());
//		cv::Scalar color_white = cv::Scalar( 255, 255, 255);
//		for(std::vector<DetectedObject>::iterator it = objects.begin(); it != objects.end(); ++it)
//		{
//		  cv::Rect ROI(it->boundRect.x, it->boundRect.y, it->boundRect.width , it->boundRect.height);
//		  rectangle(mask, ROI.tl(), ROI.br(), color_white, CV_FILLED); //mask
//
//		  // Cut out ROI and store it in crop_img
//		  src_image.copyTo(crop_img, mask);
//		}
//		//for some reason the source image get overwritten. Retrieve it back!
//		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//		src_image = cv_ptr->image.clone();



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/// Draw polygonal contour + bonding rectangles
		cv::Mat drawing = cv::Mat::zeros( src.size(), CV_8UC3 );
		for(unsigned int i = 0; i< contours_obj.size(); i++ )
		{
			cv::Scalar color = cv::Scalar( 0, 0, 255 );
			drawContours( drawing, contours_obj, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
		}
		//*/



		//cv::imshow("src filtered", src_filtered);
		//cv::imshow("shadow filter", shadow_img);
		cv::imshow("shadow binary", shadow_bin);
		//cv::imshow("sobel",sobel_img);
		cv::imshow("contours", drawing);
		cv::waitKey(3);
	}




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
/*




		cv::Mat src_image, hsv_image, im_walls, im_floor, remove_walls, remove_floor, remove_background,filter_image;
		src_image = cv_ptr->image; //get the source image from the pointer

		//remove top part of image (set to black)
		int y_remove = 100;
		for(int i=0; i<y_remove; i++)
		   for(int j=0; j<src_image.cols; j++)
		   {
			   src_image.at<cv::Vec3b>(i,j)[0] = 0;
			   src_image.at<cv::Vec3b>(i,j)[1] = 0;
			   src_image.at<cv::Vec3b>(i,j)[2] = 0;
		   }


		cv::cvtColor(src_image, hsv_image,CV_BGR2HSV);
		cv::inRange(hsv_image,cv::Scalar(wall_H_MIN,wall_S_MIN,wall_V_MIN),cv::Scalar(wall_H_MAX,wall_S_MAX,wall_V_MAX),im_walls);
		cv::inRange(hsv_image,cv::Scalar(floor_H_MIN,floor_S_MIN,floor_V_MIN),cv::Scalar(floor_H_MAX,floor_S_MAX,floor_V_MAX),im_floor);
		bitwise_not(im_walls, remove_walls); //invert the pixels so that the walls are removed
		bitwise_not(im_floor, remove_floor); //invert the pixels so that the floor is removed
		bitwise_and(remove_walls, remove_floor, remove_background); //Grayscale image with the background removed
		cvtColor(remove_background, remove_background, CV_GRAY2BGR); //change image to a BGR image
		bitwise_and(src_image, remove_background, filter_image); //Remove the background

		//Publish filtered image
		cv_bridge::CvImagePtr img_ptr = cv_ptr;
		filter_image.copyTo(img_ptr->image);
		img_pub_.publish(img_ptr->toImageMsg());

		//Does some erosion and dilation to remove some of the pixels
		cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
		cv::erode(filter_image, filter_image, cv::Mat());
		cv::dilate(filter_image, filter_image, element);



//CONTOURS DETECTION!

		//http://docs.opencv.org/doc/tutorials/imgproc/shapedescriptors/bounding_rects_circles/bounding_rects_circles.html
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		/// Detect edges using Threshold
		cv::Mat filter_image_bin;
		threshold( filter_image, filter_image_bin, 0, 255, 0 ); //0 == THRESH_BINARY
		cvtColor(filter_image_bin, filter_image_bin, CV_RGB2GRAY); //one channel

		/// Find contours
		findContours( filter_image_bin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

		std::vector< std::vector<cv::Point> > contours_poly;
		std::vector<cv::Point> cur_contours_poly;
		contours_poly.reserve(contours.size()); //reserve space
		for(unsigned int i = 0; i < contours.size(); i++ )
		{
			approxPolyDP( cv::Mat(contours[i]), cur_contours_poly, 3, true );
			double area = contourArea(cur_contours_poly);
			//std::cout<<"Area of contour nr "<<i<<": "<<area<<std::endl;
			if (area >= minArea && area<= maxArea) //save the closed approximated contour if it's area is "lagom"
			{
				//std::cout<<"\nContour is lagom! nr "<<i<<": "<<area<<std::endl;
				contours_poly.push_back(cur_contours_poly);
			}
		}



//		/// Get the moments
//		std::vector<cv::Moments> mu(contours_poly.size() );
//		for( unsigned int i = 0; i < contours_poly.size(); i++ )
//		{
//			mu[i] = moments( contours_poly[i], false );
//		}
//
//		///  Get the mass centers:
//		std::vector<cv::Point> mc( contours_poly.size() );
//		for( int i = 0; i < contours_poly.size(); i++ )
//		{
//			mc[i] = cv::Point( (int)mu[i].m10/mu[i].m00 , (int)mu[i].m01/mu[i].m00 );
//			int x = mc[i].x;
//			int y = mc[i].y;
//			std::cout<<"mc["<<i<<"].x "<<mc[i].x<<std::endl;
//			std::cout<<"mc["<<i<<"].y "<<mc[i].y<<std::endl;
//			cv::Vec3b pixel = hsv_image.at<cv::Vec3b>(y,x);
//	         int h = pixel[0];
//	         int s = pixel[1];
//	         int v = pixel[2];
//	         std::cout << "h:" << h<< " s:" << s << " v:" << v << std::endl;
//
//		}
//		std::cout<<""<<std::endl;


		/// Approximate contours to polygons + get bounding rectangles
		cv::Rect rect;
		std::vector<cv::Rect> boundRect;
		boundRect.reserve( contours_poly.size() );
		for(unsigned int i = 0; i < contours_poly.size(); i++ )
		{
			rect = boundingRect( cv::Mat(contours_poly[i]) );
			double ratio = (double)std::max(rect.height,rect.width)  / (double)std::min(rect.height,rect.width);
			//std::cout<<ratio<<std::endl;
			if (ratio > 4.3) //if it's a too narrow rectangle, ignore it
			{
				continue;
			}
			boundRect.push_back(rect);

		}

		//could not get the damned openCV-merging-rectangles-function to work properly.
		//Had to create my own way of merging intersecting rectangles!
		bool MERGE_DONE = false;
		unsigned int i = 0;
		while(!MERGE_DONE && boundRect.size()>1)
		{
			bool have_merged = false;
			for (unsigned int j = i+1; j<boundRect.size(); j++)
			{
				cv::Rect rect_a = boundRect[i];

				cv::Rect rect_b = boundRect[j];
				cv::Rect intersect = rect_a & rect_b;
				if (intersect.height != 0 && intersect.width != 0) //rectangles intersect!
				{
					int x = std::min(rect_a.x , rect_b.x);
					int y = std::min(rect_a.y , rect_b.y);
					int width = std::max(rect_a.br().x , rect_b.br().x) - x;
					int height = std::max(rect_a.br().y , rect_b.br().y) - y;
					cv::Rect merged_rect(x,y,width, height);

					boundRect[i] = merged_rect; //write over the old rectangle with the bigger one
					boundRect.erase(boundRect.begin() + j); //delete the rectangle that is already merged

					have_merged = true;
					break;
				}
			}

			//if we have merged two rectangles, start all over again in case there exist
			//rectangles that will intersect with the new bigger added rectangle.
			if (have_merged)
			{
				i = 0;
				continue;
			}
			i++;
			if (i == boundRect.size()) MERGE_DONE = true;
		}

		bool flag_send_msg = false;
		if (boundRect.size() == 1)
		{
			//std::cout<<"boundRect[0]: ("<<boundRect[0].tl()<<", "<<boundRect[0].br()<<")"<<std::endl;
			//std::cout<<"prev_rect: ("<<prev_rect.tl()<<", "<<prev_rect.br()<<")"<<std::endl;
			//std::cout<<"(boundRect[0] & prev_rect): "<<(boundRect[0] & prev_rect).tl()<<", "<<(boundRect[0] & prev_rect).br()<<")"<<std::endl;
			if ((boundRect[0] & prev_rect).height == 0) //if its not overlapping with the previous rectangle, its a new object
			{
				ROI_id_counter++;
				prev_rect = boundRect[0];
				flag_send_msg = true;
			}
			else
			{
				prev_rect = boundRect[0];
			}
		}
		else
			prev_rect = cv::Rect();
		//Create ros-message
		//if (flag_send_msg)
		//{
		//	contour_filter::Objects obj_msg;
		//	obj_msg.ROI.reserve( contours_poly.size() );
		//	for (unsigned int i=0; i < boundRect.size(); i++)
		//	{
		//		cv::Rect rect = boundRect[i];
		//		contour_filter::Rect2D_ rect2D_msg;
		//		rect2D_msg.x = rect.x;
		//		rect2D_msg.y = rect.y;
		//		rect2D_msg.height = rect.height;
		//		rect2D_msg.width = rect.width;
		//		obj_msg.ROI.push_back(rect2D_msg);
		//		obj_msg.ROI_id = ROI_id_counter;
		//	}
		//	obj_pub_.publish(obj_msg);
		//}




		if (FLAG_SHOW_IMAGE)
		{
			cv::imshow("Original", src_image);

			/// Draw polygonal contour + bonding rectangles
			cv::Mat drawing = cv::Mat::zeros( filter_image_bin.size(), CV_8UC3 );

			for(unsigned int i = 0; i< contours_poly.size(); i++ )
			{
				cv::Scalar color = cv::Scalar( 0, 0, 255 );
				drawContours( drawing, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
			}
			for(unsigned int i = 0; i< boundRect.size(); i++ )
			{
				cv::Scalar color = cv::Scalar( 0, 0, 255 );
				rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
			}
			imshow( "Found objects", drawing );
			cv::waitKey(3);
		}

		if (FLAG_CHANGE_THRESHOLD)
		{
			//cv::imshow("Original", src_image);
			cv::imshow("HSV", hsv_image);
			cv::imshow("Only the walls", im_walls);
			cv::imshow("Only the floor", im_floor);
			//cv::imshow("Result", filter_image);
			cv::waitKey(3);
		}
*/




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

