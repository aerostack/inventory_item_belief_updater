/*!*******************************************************************************************
 *  \file       inventory_item_belief_updater.cpp
 *  \brief      distance measurement implementation file.
 *  \details    This file contains the DistanceMeasurement implementattion of the class.
 *  \authors    Javier Melero Deza
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All Rights Reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/
#include "../include/inventory_item_belief_updater.h"
#include <pluginlib/class_list_macros.h>


InventoryItemBeliefUpdater::InventoryItemBeliefUpdater(){
}

InventoryItemBeliefUpdater::~InventoryItemBeliefUpdater(){
}

void InventoryItemBeliefUpdater::ownSetUp(){
    ros::NodeHandle nh("~");

    nh.param<std::string>("camera_topic", camera_topic_str, "camera_front/image_raw");
    nh.param<std::string>("qr_position_topic", qr_position_topic_str, "qr_code_localized");
    nh.param<std::string>("qr_camera_topic", camera_bounding_topic_str, "bounding_image_raw");
    qr_code_localized_sub = node_handle.subscribe(qr_position_topic_str, 1, &InventoryItemBeliefUpdater::PointCallback, this);
    camera_sub = node_handle.subscribe(camera_topic_str, 1, &InventoryItemBeliefUpdater::CameraCallback, this);
    qr_camera_pub = node_handle.advertise<sensor_msgs::Image>(camera_bounding_topic_str, 1, true);

    add_client = node_handle.serviceClient<belief_manager_msgs::AddBelief>("add_belief");
    remove_client = node_handle.serviceClient<belief_manager_msgs::RemoveBelief>("remove_belief");
    id_gen_client = node_handle.serviceClient<belief_manager_msgs::GenerateID>("belief_manager_process/generate_id");
    query_client = node_handle.serviceClient<belief_manager_msgs::QueryBelief>("query_belief");


    sent = false;
    n_codes = 0;
    unseen = 0;

}

void InventoryItemBeliefUpdater::CameraCallback (const sensor_msgs::ImageConstPtr &msg){
  mtx.lock();
  try
  {
    image_cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    
  }
  mtx.unlock();
  // ToDo := If mutex implementation doesn't unlock upon destruction or destruction doesn't happen
  // when going through the catch clause, there will be a deadlock
}



void InventoryItemBeliefUpdater::PointCallback (const aerostack_msgs::ListOfQrCodeLocalized &msg){
    codes_in_frame = msg;
    bool bounding_box = false;
    std::list <std::string> qr_in_frame;
    std::list <geometry_msgs::Point> points_in_frame;
    std::list <cv::Rect> boundings_in_frame;
    
    for (int i = 0; i < codes_in_frame.list_of_qr_codes.size(); i++){
        bounding_box = false;
        
        qr_in_frame.push_back(codes_in_frame.list_of_qr_codes[i].code);
    }
    
      if (qr_list.size() < 8){
        qr_list.push_back(qr_in_frame);
    //    sendQRInterpretation(qr_in_frame, points_in_frame, true);
        for (int i = 0; i<codes_in_frame.list_of_qr_codes.size(); i++){
          if (codes_in_frame.list_of_qr_codes[i].code != ""){
            bounding_box = Visible(codes_in_frame.list_of_qr_codes[i].code);
          }
          
          if (bounding_box){ // Display received bounding box
              std::vector<cv::Point2f> points (4);
              points[0] = cv::Point2f(codes_in_frame.list_of_qr_codes[i].bounding_points[0].x, codes_in_frame.list_of_qr_codes[i].bounding_points[0].y);
              points[1] = cv::Point2f(codes_in_frame.list_of_qr_codes[i].bounding_points[1].x, codes_in_frame.list_of_qr_codes[i].bounding_points[1].y);
              points[2] = cv::Point2f(codes_in_frame.list_of_qr_codes[i].bounding_points[2].x, codes_in_frame.list_of_qr_codes[i].bounding_points[2].y);
              points[3] = cv::Point2f(codes_in_frame.list_of_qr_codes[i].bounding_points[3].x, codes_in_frame.list_of_qr_codes[i].bounding_points[3].y);

              cv::RotatedRect box = cv::minAreaRect(points);
              BoundingBox = box.boundingRect();
              setupBeliefs(codes_in_frame.list_of_qr_codes[i].code, codes_in_frame.list_of_qr_codes[i].point);
              cv::rectangle(image_cv->image, BoundingBox, cv::Scalar(255,255,0), 5);
              cv::putText(image_cv->image, qrBox, cv::Point(BoundingBox.tl().x, BoundingBox.tl().y - 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,0), 2);

          }
        } 
      }
          
      else {
        
        std::list<std::string> codes = qr_list.front();  
        qr_list.pop_front();
        qr_list.push_back(qr_in_frame);
        std::list<std::string>::iterator codes_it = std::next(codes.begin(), 0);
        for (int i = 0; i<codes.size(); i++){
          
          std::string code = *codes_it;
                  
          if (code != "" && !Visible(code)){
            qrBox = FindQrBoxName(std::stoi(code));
            belief_manager_msgs::QueryBelief srv;
            srv.request.query = "object(?x, " + qrBox + ")"; 
            query_client.call(srv);
            belief_manager_msgs::QueryBelief::Response response= srv.response;
            if(response.success==true){
              std::string str = response.substitutions;
              qr_code_belief_id = str.substr(3);
              removeBelief("visible(" +  qr_code_belief_id + ")");

            }            
          }
          std::advance(codes_it, 1);
        }
              
        for (int i = 0; i<codes_in_frame.list_of_qr_codes.size(); i++){
          if (codes_in_frame.list_of_qr_codes[i].code != ""){
            bounding_box = Visible(codes_in_frame.list_of_qr_codes[i].code);
          }
          if (bounding_box){ // Display received bounding box
              std::vector<cv::Point2f> points (4);
              points[0] = cv::Point2f(codes_in_frame.list_of_qr_codes[i].bounding_points[0].x, codes_in_frame.list_of_qr_codes[i].bounding_points[0].y);
              points[1] = cv::Point2f(codes_in_frame.list_of_qr_codes[i].bounding_points[1].x, codes_in_frame.list_of_qr_codes[i].bounding_points[1].y);
              points[2] = cv::Point2f(codes_in_frame.list_of_qr_codes[i].bounding_points[2].x, codes_in_frame.list_of_qr_codes[i].bounding_points[2].y);
              points[3] = cv::Point2f(codes_in_frame.list_of_qr_codes[i].bounding_points[3].x, codes_in_frame.list_of_qr_codes[i].bounding_points[3].y);

              cv::RotatedRect box = cv::minAreaRect(points);
              BoundingBox = box.boundingRect();
              setupBeliefs(codes_in_frame.list_of_qr_codes[i].code, codes_in_frame.list_of_qr_codes[i].point);
              cv::rectangle(image_cv->image, BoundingBox, cv::Scalar(255,255,0), 5);
              cv::putText(image_cv->image, qrBox, cv::Point(BoundingBox.tl().x, BoundingBox.tl().y - 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,0), 2);
              
          }
        }
      }
    qr_camera_pub.publish(image_cv->toImageMsg());
  }
      

int InventoryItemBeliefUpdater::requestBeliefId()
{
  int ret = 100;
  belief_manager_msgs::GenerateID::Request req;
  belief_manager_msgs::GenerateID::Response res;

  id_gen_client.call(req, res);

  if (res.ack)
  {
      ret = res.id;
  }  unseen = 0;
  return ret;
}

bool InventoryItemBeliefUpdater::addBelief(std::string message, bool multivalued)
{
  belief_manager_msgs::AddBelief::Request req;
  belief_manager_msgs::AddBelief::Response res;
  req.belief_expression = message;
  req.multivalued = multivalued;
  add_client.call(req, res);

  return res.success;
}

bool InventoryItemBeliefUpdater::removeBelief(std::string message)
{
  belief_manager_msgs::RemoveBelief::Request req;
  belief_manager_msgs::RemoveBelief::Response res;
  req.belief_expression = message;
  remove_client.call(req, res);
  return res.success;
}

std::string InventoryItemBeliefUpdater::FindQrBoxName(int code){
  std::string qrBox;
  for (int i = 0; i<sizeof(relations); i++){
    if (relations[i].code == code){
      qrBox = relations[i].box;
      break;
    }
  }
  return qrBox;
}

float InventoryItemBeliefUpdater::round(float var) 
{  
    float value = roundf(var * 100) / 100;
    return value; 
} 

bool InventoryItemBeliefUpdater::setupBeliefs(std::string message, geometry_msgs::Point point)
{
  qrBox = FindQrBoxName(std::stoi(message));
  belief_manager_msgs::QueryBelief srv;
  srv.request.query = "object(?x, " + qrBox + ")"; 
  query_client.call(srv);
  belief_manager_msgs::QueryBelief::Response response= srv.response;
  if(response.success==true){
    std::string str = response.substitutions;
    qr_code_belief_id = str.substr(3);

  }
  else {
    qr_code_belief_id = std::to_string(requestBeliefId());
    std::string msg1 = "object(" + qr_code_belief_id + ", " + qrBox + ")";
    std::stringstream out;
    out << std::setprecision(1) << std::fixed << point.x;
    std::string x = out.str();
    out.str(std::string());
    out << std::setprecision(1) << std::fixed << point.y;
    std::string y = out.str();
    out.str(std::string());
    out << std::setprecision(1) << std::fixed << point.z;
    std::string z = out.str();
    std::string msg2 = "position(" + qr_code_belief_id + ", (" + x + ", " + y + ", " + z + "))";
    if (!addBelief(msg1, true)){
      removeBelief(msg1);
      return false;
    }
    if (!addBelief(msg2, true)){
      removeBelief(msg1);
      removeBelief(msg2);
      return false;
    }    
  }
  srv.request.query = "visible(" + qr_code_belief_id + ")"; 
  query_client.call(srv);
  response= srv.response;
  if(response.success==false){
    std::string msg3 = "visible(" + qr_code_belief_id + ")";
    if (!addBelief(msg3, true)){
      removeBelief(msg3);
      return false;
    }
  }
  return true;
}
bool InventoryItemBeliefUpdater::Visible (std::string code){
  bool ret = true;
  int c = 0;
  std::list<std::list<std::string>>::iterator qr_in_frames_iterator = std::next(qr_list.begin(), 0);
  for (int i = 0; i< qr_list.size(); i++){
    
    std::list<std::string> qr_in_frames = *qr_in_frames_iterator;
    
    if (!(std::find(std::begin(qr_in_frames), std::end(qr_in_frames), code) != std::end(qr_in_frames))){                   
      c++;
    }
      if (c > 6){
        ret = false;
        break;
      }
      std::advance(qr_in_frames_iterator, 1);
  }


  return ret;
}


void InventoryItemBeliefUpdater::ownStart(){
}

void InventoryItemBeliefUpdater::ownRun(){
}

void InventoryItemBeliefUpdater::ownStop(){
      add_client.shutdown();
  remove_client.shutdown();

}


