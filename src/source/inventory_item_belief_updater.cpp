/*!*******************************************************************************************
 *  \file       inventory_item_belief_updater.cpp
 *  \brief      belief updater implementation file.
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

    nh.param<std::string>("qr_position_topic", qr_position_topic_str, "qr_code_localized");
    nh.param<std::string>("item_annotator_topic", item_annotator_topic_str, "item_annotator");
    qr_code_localized_sub = node_handle.subscribe(qr_position_topic_str, 1, &InventoryItemBeliefUpdater::PointCallback, this);
    item_annotator_pub = node_handle.advertise<aerostack_msgs::ListOfInventoryItemAnnotation>(item_annotator_topic_str, 30, true);
    add_client = node_handle.serviceClient<belief_manager_msgs::AddBelief>("add_belief");
    remove_client = node_handle.serviceClient<belief_manager_msgs::RemoveBelief>("remove_belief");
    id_gen_client = node_handle.serviceClient<belief_manager_msgs::GenerateID>("belief_manager_process/generate_id");
    query_client = node_handle.serviceClient<belief_manager_msgs::QueryBelief>("query_belief");

    sent = false;

}

void InventoryItemBeliefUpdater::PointCallback (const aerostack_msgs::ListOfQrCodeLocalized &msg){
    aerostack_msgs::ListOfInventoryItemAnnotation annotationList;
    codes_in_frame = msg;
    bool bounding_box = false;
    std::list <std::string> qr_in_frame;
    std::list <geometry_msgs::Point> points_in_frame;
    std::list <cv::Rect> boundings_in_frame;
    
    for (int i = 0; i < codes_in_frame.list_of_qr_codes.size(); i++){
        bounding_box = false;
        
        qr_in_frame.push_back(codes_in_frame.list_of_qr_codes[i].code);
    }
    
    if (frames_list.size() < 15){
      frames_list.push_back(qr_in_frame);
      for (int i = 0; i<codes_in_frame.list_of_qr_codes.size(); i++){
        if (codes_in_frame.list_of_qr_codes[i].code != ""){
          bounding_box = Visible(codes_in_frame.list_of_qr_codes[i].code);
        }
          
        if (bounding_box){ // Display received bounding box
          aerostack_msgs::InventoryItemAnnotation annotation;
          annotation.bounding_points = codes_in_frame.list_of_qr_codes[i].bounding_points;
          setupBeliefs(codes_in_frame.list_of_qr_codes[i].code, codes_in_frame.list_of_qr_codes[i].point);
          annotation.object = qrBox;
          annotationList.list_of_annotations.push_back(annotation);
        }
      } 
    }
          
    else {
        
      std::list<std::string> codes = frames_list.front();  
      frames_list.pop_front();
      frames_list.push_back(qr_in_frame);
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
            aerostack_msgs::InventoryItemAnnotation annotation;
            annotation.bounding_points = codes_in_frame.list_of_qr_codes[i].bounding_points;             
            setupBeliefs(codes_in_frame.list_of_qr_codes[i].code, codes_in_frame.list_of_qr_codes[i].point);
            annotation.object = qrBox;
            annotationList.list_of_annotations.push_back(annotation);
              
        }
      }
    }
  item_annotator_pub.publish(annotationList);
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
  }  
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
  std::list<std::list<std::string>>::iterator qr_in_frames_iterator = std::next(frames_list.begin(), 0);
  for (int i = 0; i< frames_list.size(); i++){
    
    std::list<std::string> qr_in_frames = *qr_in_frames_iterator;
    
    if (!(std::find(std::begin(qr_in_frames), std::end(qr_in_frames), code) != std::end(qr_in_frames))){                   
      c++;
    }
      if (c >= 10){
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


