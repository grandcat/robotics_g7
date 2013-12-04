#include "headers/recognition_master.hpp"
#include "color_filter/Rect2D_.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>      // CV GUI
#include <sensor_msgs/image_encodings.h>

#define RGB_DEPTH_IMG_ALIGNMENT 15          // Alignment correction of depth image to RGB image (in pixel)

namespace objRecognition
{

/**
 * @brief RecognitionMaster::runRecognitionPipeline
 *  Start of the recongition pipeline. This will execute the necessary recognition slaves at a given frame
 *  rate and calculate results.
 * @param msg Primesense image
 */
void RecognitionMaster::runRecognitionPipeline(const sensor_msgs::ImageConstPtr& msg)
{
  ros::Duration(0.1).sleep();
  // TODO: disable recognition when turning 90 degree
  // Get object positions (if any)
  objRecog_Colorfilter.showDebugOutput();
  objRecog_Colorfilter.color_filter(msg);
  lastObjPositions = objRecog_Colorfilter.getDetectedObjects();
  unsigned int cDetectedObjs = lastObjPositions.size();
  ROS_INFO("Detected amount of objects: %u", cDetectedObjs);

  // Reject if no object was detected
  if (cDetectedObjs != 1)
  {
    ROS_INFO("Rejected basic obj detection, more than 2 objects or no objects.");
    return;
  }

  ROS_INFO("1. Obj:\n id:%i, cm y:%i x:%i", lastObjPositions[0].ROI_id,
      lastObjPositions[0].mc.y, lastObjPositions[0].mc.x);

  explorer::Object relMazePos = translateCvToMap(lastObjPositions[0].mc.y, lastObjPositions[0].mc.x);
  // If position couldn't determined, reject frame
  if (relMazePos.x == -1)
    return;
  ROS_INFO("Depth used positon, y: %i, x: %i", lastObjPositions[0].mc.y, lastObjPositions[0].mc.x);

  if (lastRecognizedId == OBJTYPE_NO_OBJECT)
    {
      ROS_INFO("Rejected basic obj detection: feature detection didn't work!");
      lastRecognizedId = OBJTYPE_UNKNOWN_OBJECT_DETECTED;
//      return;
    }

  relMazePos.id = (int)lastRecognizedId;
  ROS_INFO("Detected object: %s", TEXT_OBJECTS[lastRecognizedId].c_str());
  pub_recognition_result.publish(relMazePos);
  // TESTING (sleeping after each processing is good for system performance)
//  ros::Duration(5).sleep();
}


/**
 * @brief RecognitionMaster::rcvObjType Store obj type information for processing later in rcvSlaveRecognition
 * @param msg
 */
void RecognitionMaster::rcvObjType(const object_recognition::Recognized_objects::ConstPtr msg)
{
  std::vector<int> detectedObjTypes = msg->obj_type;
  lastRecognizedId = (enum EObjectTypes)detectedObjTypes[0];
}

void RecognitionMaster::rcvDepthImg(const sensor_msgs::ImageConstPtr& msg)
{
  ++cRejectedFrames;
  // TODO: deactivate subscription if no depth pos was requested for 10 frames
//  ros::Duration(4).sleep(); // <-- same thread as rgb image, therefore same rate
  ROS_INFO("[recognition master] received depth image %f", ros::Time::now().toSec());
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
  // Correct alignment of depth image to RGB image
  x += RGB_DEPTH_IMG_ALIGNMENT;
//  y += RGB_DEPTH_IMG_ALIGNMENT;

  explorer::Object relMazePos;
  relMazePos.x = -1;

  if (cRejectedFrames == 0)
    {
      // No frame available: no position estimation possible
      return relMazePos;
    }

  float sumDistance = 0;
  int nAvg = 0;

  for (int i = 0; i <= 10; i++)
    {
      for (int j = 0; j <= 10; j++)
        {
          if ((((y + j - 5) < curDepthImg.rows) & ((x + i - 5) < curDepthImg.cols))
              && (((y + j - 5) >= 0) & ((x + i - 5) >= 0)))
            {
              float curDepth = curDepthImg.at<float>(y + j - 5, x + i - 5);
              // DEBUG: mark point for visualization
              curDepthImg.at<float>(y + j - 5, x + i - 5) = 255;

//                                ROS_INFO("Raw value: %f", curDepth);
              if (!isnan(curDepth))
                {
                  sumDistance += curDepth;
                  ++nAvg;
                }
            }
        }
    }

  if (nAvg != 0)
    {
      relMazePos.x = sumDistance / nAvg;// * 1000;
      // calculate y approximation
      relMazePos.y = (320 - x) * (relMazePos.x / 530);

      // DEBUG: show visualization
      cv::imshow("Depth image with marked point", curDepthImg);
      cvWaitKey(10);
    }
  else
    {
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
  ROS_INFO("[Recognition master] Starting up...");
  objRecognition::RecognitionMaster recognMaster(nh);

  ros::spin();
  return 0;
}
