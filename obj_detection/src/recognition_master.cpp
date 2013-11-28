#include "headers/recognition_master.hpp"
#include "color_filter/Rect2D_.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>      // CV GUI
#include <sensor_msgs/image_encodings.h>

namespace objRecognition
{

  void RecognitionMaster::rcvSlaveRecognition(const color_filter::Objects::ConstPtr &msg)
  {
    std::vector<color_filter::Rect2D_> detectedObjects = msg->ROI;
    int cDetectedObjs = (int)detectedObjects.size();
    ROS_INFO("Recognition master: received %i objects", cDetectedObjs);

    // Reject if no object was detected
    if (cDetectedObjs == 0)
      return;
    // Received obejct (probably): Activate depth image for getting distance
    subscribeDepthImg();

    // Take biggest object (based on rectangle), estimate position based on center point
    int bestRatio = 0, biggestObj = -1;
    for (int objId = 0; objId < cDetectedObjs; ++objId) {
      int objRatio = detectedObjects[objId].width * detectedObjects[objId].height;
      if (objRatio > bestRatio)
      {
        biggestObj = objId;
        bestRatio = objRatio;
      }
    }

    explorer::Object relMazePos = translateCvToMap(detectedObjects[biggestObj].y + detectedObjects[biggestObj].height / 2,
                                                   detectedObjects[biggestObj].x + detectedObjects[biggestObj].width / 2);

    // If position couldn't determined, reject frame
    if (relMazePos.x == -1)
      return;

    ROS_INFO("Used positon, y: %i, x: %i", detectedObjects[biggestObj].y + detectedObjects[biggestObj].height / 2,
             detectedObjects[biggestObj].x + detectedObjects[biggestObj].width / 2);
    // DEBUG print
    for (int yPos = -10; yPos <= 10; ++yPos) {
      for (int xPos = -10; xPos <= 10; ++xPos) {
        curDepthImg.at<float>(detectedObjects[biggestObj].y + detectedObjects[biggestObj].height / 2 + yPos,
                              detectedObjects[biggestObj].x + detectedObjects[biggestObj].width / 2 + xPos) = 255;
      }
    }
    cv::imshow("Depth image with marked point", curDepthImg);
    cvWaitKey(10);

    // Check whether obj id was also detected, otherwise reject information
    if (lastRecognizedId == OBJTYPE_NO_OBJECT)
      return;

    relMazePos.id = (int)lastRecognizedId;
    pub_recognition_result.publish(relMazePos);

//    // DEBUG (TODO: remove!!)
//    sub_obj_detection.shutdown();

  }

  /**
   * @brief RecognitionMaster::rcvObjType Store obj type information for processing later in rcvSlaveRecognition
   * @param msg
   */
  void RecognitionMaster::rcvObjType(const color_filter::Objects::ConstPtr& msg)
  {


  }

  void RecognitionMaster::rcvDepthImg(const sensor_msgs::ImageConstPtr& msg)
  {
    ++cRejectedFrames;
    // TODO: deactivate subscription if no depth pos was requested for 10 frames

    // Convert depth image to OpenCV Mat format
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            curDepthImg = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
            ROS_ERROR("[Recognition master] cv_bridge exception: %s", e.what());
            return;
    }
  }

  explorer::Object RecognitionMaster::translateCvToMap(int y, int x)
  {
    explorer::Object relMazePos;
    relMazePos.x = -1;

    if (cRejectedFrames == 0)
    {
      // No frame available: no position estimation possible
      return relMazePos;
    }

    float sumDistance = 0;
    int nAvg = 0;
    for (int i = 0; i <= 10; i++) {
            for (int j = 0; j <= 10; j++) {
                    if ((((y + j - 5) < curDepthImg.rows) & ((x + i - 5) < curDepthImg.cols))
                                    && (((y + j - 5) >= 0) & ((x + i - 5) >= 0))) {
                            float curDepth = curDepthImg.at<float>(y + j - 5, x + i - 5);
//                                ROS_INFO("Raw value: %f", curDepth);
                            if (!isnan(curDepth)) {
                                    sumDistance += curDepth;
                                    ++nAvg;
                            }
                    }
            }
    }
    if (nAvg != 0) {
            relMazePos.x = sumDistance / nAvg * 1000;
            // calculate y approximation
            relMazePos.y = (320 - x) * (relMazePos.x / 530);

    } else {
            relMazePos.x = -1;
    }
//    relMazePos.y = 0; // TODO: needs more calculations and a noise-free depth image (use point cloud later maybe)

    return relMazePos;
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_processing");
  ros::NodeHandle nh;

  ROS_INFO("Starting recognition master.");

  objRecognition::RecognitionMaster recognMst(nh);
//  processPipeline.start();

//  // Create a ROS subscriber for thle input point cloud
//  sub_pcl_primesense = nh.subscribe("/camera/depth_registered/points", 1, process_pcl_data);
//  pub_pcl_filtered = nh.advertise<sensor_msgs::PointCloud2>("/camera/filtered_points", 1);
//  ROS_INFO("Subscribed to depth pointcloud.");

  ros::spin();
  return 0;
}
