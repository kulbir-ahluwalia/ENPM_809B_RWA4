//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>



//AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}
{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);


    break_beam_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/break_beam_1_change", 10,
            &AriacOrderManager::break_beam_callback, this);

    camera_4_subscriber_ = order_manager_nh_.subscribe("/ariac/logical_camera_4", 10,
                                                &AriacOrderManager::LogicalCamera4Callback, this);

}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN(">>>>> OrderCallback");
    received_orders_.push_back(*order_msg);
}


void AriacOrderManager::break_beam_callback(const osrf_gear::Proximity::ConstPtr& msg) {
    double temp=ros::Time::now().toSec();
    if (flag_==1)
      {ROS_INFO_STREAM("Time Delay observed"<<temp-camera_4_detection_time_);}
    if (flag_==1     &&    temp-camera_4_detection_time_>4.50)
    {
    if(msg->object_detected) {
        ROS_INFO("Part Required Detected by Break beam triggered..");
        flag_break_beam_ = 1;
        break_beam_time = ros::Time::now();
    }
  }
}
void AriacOrderManager::LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    // if (init_) return;
    // ROS_INFO_STREAM_THROTTLE(10,
    //                          "Logical camera 4: '" << image_msg->models.size() << "' objects.");
    // if (image_msg->models.size() == 0)
    //     ROS_ERROR_STREAM("Logical Camera 4 does not see anything");

    // current_parts_4_ = *image_msg;
    // this->BuildProductFrames(3);
    if (init_==0)
      return;



    for(int i=0; i<image_msg->models.size();i++){
       if (image_msg->models[i].type==camera_4_product_){
        //ROS_INFO_STREAM("Part Required Detected");
        flag_=1;
        camera_4_detection_time_=ros::Time::now().toSec();
        ROS_INFO_STREAM("Camera Detected at "<<camera_4_detection_time_);
        geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
        StampedPose_in.header.frame_id = "/logical_camera_4_frame";
        StampedPose_in.pose = image_msg->models[i].pose;
        //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        camera4_op_pose_=StampedPose_out.pose;
        //ROS_INFO_STREAM("part is at   "<<camera4_op_pose_);
       }

     }
     if (image_msg->models.size()==0)
      {flag_=0;}
    init_=1;}



/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type) {
    //--Grab the last one from the list then remove it
    if (!product_frame_list_.empty()) {
        std::string frame = product_frame_list_[product_type].back();
        ROS_INFO_STREAM("Frame >>>> " << frame);
        product_frame_list_[product_type].pop_back();
        return frame;
    } else {
        ROS_ERROR_STREAM("No product frame found for " << product_type);
        ros::shutdown();
    }
}

bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    std::string product_type = product_type_pose.first;
    ROS_WARN_STREAM("Product type >>>> " << product_type);
    std::string product_frame = this->GetProductFrame(product_type);
    ROS_WARN_STREAM("Product frame >>>> " << product_frame);
    auto part_pose = camera_.GetPartPose("/world",product_frame);


    if(product_type == "pulley_part")
        part_pose.position.z += 0.08;
    //--task the robot to pick up this part
    bool failed_pick = arm1_.PickPart(part_pose);
    ROS_WARN_STREAM("Picking up state " << failed_pick);
    ros::Duration(0.5).sleep();

    while(!failed_pick){
        auto part_pose = camera_.GetPartPose("/world",product_frame);
        failed_pick = arm1_.PickPart(part_pose);
    }

    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y -= 0.2;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

    }
    else{
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y += 0.2;
        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
    }
    auto result = arm1_.DropPart(StampedPose_out.pose);

    bool result1;
    if (result==-1){
      ROS_INFO_STREAM("Retrying due to defected part");
      std::pair<std::string,geometry_msgs::Pose> retry_pair;
      retry_pair.first=product_type;
      retry_pair.second=drop_pose;
      //Make it go to home first
      arm1_.SendRobotHome();
      ros::Duration(0.5).sleep();
      arm1_.GripperToggle(false);
      result1=PickAndPlace(retry_pair, int(1)); }
    
    if (result==0)
      result1=false;
    else
      result1=true;
    return result1;
}


void AriacOrderManager::PickFromConveyer(std::string product_type)
{  camera_4_product_=product_type;   //Let camera detect product_type parts
   


   ROS_INFO_STREAM("Picking up from conveyer");
   arm1_.SendRobotConveyer();
   ros::Duration(0.5).sleep();
   

   auto temp_pose=camera4_op_pose_;
   temp_pose.position.z=temp_pose.position.z-0.1;

   geometry_msgs::Pose temporaryPose;
   temporaryPose.position.x = 1.193;
   temporaryPose.position.y = 0.407;
   temporaryPose.position.z = 1.175 ;
   
   auto temp_pose_1 = temporaryPose;
   temp_pose_1.position.z -= (0.120);
   if (product_type=="pulley_part")
    {
     temp_pose_1.position.z -= (0.120+0.08); 
    }                                       //Take robot to conveyer position
   arm1_.GoToTarget({ temporaryPose,temp_pose_1,});       
   

   while(1)
   {
    init_=1;
    //auto time1=ros::Time::now();
    auto up_pose=camera4_op_pose_;
    up_pose.position.z=up_pose.position.z+0.15;
    up_pose.position.y=up_pose.position.y+0.10;
   if (flag_==1){
    ROS_INFO_STREAM("Moving down...");
    camera4_op_pose_.position.y=camera4_op_pose_.position.y-0.25+0.008;
    arm1_.GripperToggle(true);
    camera4_op_pose_.position.z=camera4_op_pose_.position.z+0.02;
    arm1_.GoToTarget(camera4_op_pose_);
      if (arm1_.GetGripperStatus()==true)
          break;
      else{ros::spinOnce();
        arm1_.GoToTarget(temp_pose_1);}
      }
      
    ros::spinOnce();}
   
init_=0;
arm1_.SendRobotHome();
ros::Duration(0.25).sleep();

}

void AriacOrderManager::ExecuteOrder() {
    ROS_WARN(">>>>>> Executing order...");
    //scanned_objects_ = camera_.GetParts();

    //-- used to check if pick and place was successful
    while (1)
        {ros::spinOnce();
            if (received_orders_.size()!=0)
                break;
        }



    bool pick_n_place_success{false};

    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;

    ros::spinOnce();
    // ros::Duration(1.0).sleep();
   product_frame_list_ = camera_.get_product_frame_list();
   for (const auto &order:received_orders_){
         //auto order_id = order.order_id;
         auto shipments = order.shipments;
         for (const auto &shipment: shipments){
             auto shipment_type = shipment.shipment_type;
              for (const auto &product: shipment.products){
                ROS_INFO_STREAM("Product to place is  "    <<product.type);
                std::pair<std::string,geometry_msgs::Pose> product_type_pose;
                product_type_pose.first=product.type;
                product_type_pose.second=product.pose;
                if (product_frame_list_.find(product.type)!=product_frame_list_.end())  //Found in list
                  PickAndPlace(product_type_pose, int(1));
                else{
                  ROS_INFO_STREAM("Please pick up from conveyer"    <<product.type);
                  PickFromConveyer(product_type_pose.first);
                  bool result,result1;
                  geometry_msgs::Pose drop_pose = product_type_pose.second;

			    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

			    // if(agv_id==1){
			        StampedPose_in.header.frame_id = "/kit_tray_1";
			        StampedPose_in.pose = drop_pose;
			        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
			        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
			        StampedPose_out.pose.position.z += 0.1;
			        StampedPose_out.pose.position.y -= 0.2;
			        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

			    // }
                  while(1) {


                  result = arm1_.DropPart(StampedPose_out.pose);
                  ROS_INFO_STREAM("result from DropPart: "<< result);

			    // bool result1;
			    if (result==-1){
			      ROS_INFO_STREAM("Retrying due to defected part");
			      std::pair<std::string,geometry_msgs::Pose> retry_pair;
			      retry_pair.first=product.type;
			      retry_pair.second=product.pose;
			      //Make it go to home first
			      arm1_.SendRobotHome();
			      ros::Duration(0.5).sleep();
			      arm1_.GripperToggle(false);
			      PickFromConveyer(retry_pair.first); 
			      //result1 = arm1_.GetGripperStatus();
			  }
			  if (result==0) break;}
			    
			    
			    // return result1;


                    }

                

                }
                //Write function for pick and place from conveyer
                ROS_INFO_STREAM("Shipment completed Please send back AGV");
                SubmitAGV(1);
                ros::spinOnce();
                //SubmitAGV(1);
                }
              ROS_INFO_STREAM("Congragulations Order Completed");
              ros::spinOnce();
              }
             

}


void AriacOrderManager::SubmitAGV(int num) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    // srv.request.kit_type = "order_0_kit_0";
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else
        ROS_INFO("Service succeeded.");
}
