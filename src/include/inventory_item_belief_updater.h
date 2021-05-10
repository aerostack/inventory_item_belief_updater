#ifndef INVENTORY_ITEM_BELIEF_UPDATER_H
#define INVENTORY_ITEM_BELIEF_UPDATER_H

#include <string>
#include <queue>
#include <array>
#include <list>
#include <cmath>
#include <algorithm>
#include <iterator>
#include "cv.h"
// ROS
#include <ros/ros.h>

// Move base
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <aerostack_msgs/QrCodeLocalized.h>
#include <aerostack_msgs/ListOfQrCodeLocalized.h>
#include "std_srvs/Empty.h"
#include <belief_manager_msgs/AddBelief.h>
#include <belief_manager_msgs/QueryBelief.h>
#include <belief_manager_msgs/GenerateID.h>
#include <belief_manager_msgs/RemoveBelief.h>
#include <droneMsgsROS/QRInterpretation.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>

// Aerostack
#include <robot_process.h>
#include <stdio.h>
#include <math.h>
#include "zbar.h"
#include "std_msgs/Bool.h"

using namespace zbar;

class InventoryItemBeliefUpdater : public RobotProcess
{

  public:
    InventoryItemBeliefUpdater();
    ~InventoryItemBeliefUpdater();
      void ownSetUp();
      void ownRun();
      void ownStop();
      void ownStart();

  private:
	struct G_relation{
		int code;
		std::string box;
	};
	G_relation relations[20] = {{4, "Box_A"}, {9, "Box_B"},{5, "Box_C"}, {6, "Box_D"}, {30, "Box_E"},
{3, "Box_F"}, {2, "Box_G"},{1, "Box_H"}, {8, "Box_I"},{10, "Box_J"}, {32, "Box_K"},{29, "Box_L"}, {19, "Box_M"},
{26, "Box_N"}, {27, "Box_O"},{14, "Box_P"}, {17, "Box_Q"},{15, "Box_R"}, {12, "Box_S"}, {20, "Box_T"}};
		
	bool sent;
	int n_codes;
	int unseen;
	
	ros::NodeHandle node_handle;
	bool Visible (std::string code);
	//bool sendQRInterpretation(std::list<std::string>message, geometry_msgs::Point point , bool visible);

  	bool addBelief(std::string message, bool multivalued);
  	bool removeBelief(std::string message);
  	bool setupBeliefs(std::string message, geometry_msgs::Point point);
	float round(float var);
	std::string FindQrBoxName(int code);
  	int requestBeliefId();

  	ros::ServiceClient add_client;
  	ros::ServiceClient remove_client;
	ros::ServiceClient id_gen_client;
	ros::ServiceClient query_client;
  //Congfig variables
	std::string ing_box;
	bool bounding_box;
	std::list <std::list <std::string>> qr_list;
	std::list <std::list <geometry_msgs::Point>> points_list;
  	std::string qr_code_belief_id;
	std::string qr_position_topic_str;
	std::string camera_topic_str;
	std::string camera_bounding_topic_str;

	ros::Subscriber qr_code_localized_sub;
	ros::Subscriber camera_sub;
	ros::Publisher qr_camera_pub;
	aerostack_msgs::ListOfQrCodeLocalized codes_in_frame;
	cv::Rect BoundingBox;
	std::string qrBox;
	cv_bridge::CvImagePtr image_cv;
    	std::mutex mtx;

  	void PointCallback (const aerostack_msgs::ListOfQrCodeLocalized &msg);
	void CameraCallback (const sensor_msgs::ImageConstPtr& msg);

};
#endif
