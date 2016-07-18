/**
author Sandra Devin

Module allowing to perform unitary action test

**/

#include <gtp_ros_msg/requestAction.h>
#include <pr2motion/Arm_Right_MoveAction.h>
#include <pr2motion/Arm_Left_MoveAction.h>
#include <pr2motion/Gripper_Right_OperateAction.h>
#include <pr2motion/Gripper_Left_OperateAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2motion/InitAction.h>
#include <pr2motion/connect_port.h>
#include <pr2motion/Head_Move_TargetAction.h>
#include <action_tester/ExecuteAction.h>
#include <action_tester/ExecuteTask.h>
#include <action_tester/ControlGripper.h>
#include <action_tester/ExecuteSubTraj.h>
#include "toaster_msgs/PutInHand.h"
#include "toaster_msgs/RemoveFromHand.h"
#include "toaster_msgs/ObjectListStamped.h"

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

ros::NodeHandle* node;
std::string robotName;
double waitActionServer;
std::map<int, action_tester::ExecuteAction::Request> gtpTasks;

actionlib::SimpleActionClient<gtp_ros_msg::requestAction>* acGTP;
actionlib::SimpleActionClient<pr2motion::InitAction>* PR2motion_init;
actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction>* PR2motion_arm_right;
actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveAction>* PR2motion_arm_left;
actionlib::SimpleActionClient<pr2motion::Gripper_Right_OperateAction>* PR2motion_gripper_right;
actionlib::SimpleActionClient<pr2motion::Gripper_Left_OperateAction>* PR2motion_gripper_left;
actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction>* head_action_client;
actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>* gripper_right;
actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>* gripper_left;

/*
Find a plan with gtp and return the corresponding id (-1 if no solution found)
*/
int planGTP(action_tester::ExecuteAction::Request  &req){

    //Update GTP world state
    gtp_ros_msg::requestGoal goal;
    goal.req.requestType = "update";
    acGTP->sendGoal(goal);

    //wait for the action to return
    bool finishedBeforeTimeout = acGTP->waitForResult(ros::Duration(waitActionServer));

    if (!finishedBeforeTimeout){
      ROS_INFO("[action_executor] GTP Update did not finish before the time out.");
      return -1;
    }

    //Planning request
    goal.req.requestType = "planning";
    goal.req.actionName = req.actionName;
    std::vector<gtp_ros_msg::Ag> agents;
    std::vector<gtp_ros_msg::Obj> objects;
    std::vector<gtp_ros_msg::Points> points;
    std::vector<gtp_ros_msg::Data> datas;

    gtp_ros_msg::Ag agent;
    agent.actionKey = "mainAgent";
    agent.agentName = robotName;
    agents.push_back(agent);

    if(req.object.size()){
        gtp_ros_msg::Obj object;
        object.actionKey = "mainObject";
        object.objectName = req.object;
        objects.push_back(object);
    }
    if(req.support.size()){
        gtp_ros_msg::Obj support;
        support.actionKey = "supportObject";
        support.objectName = req.support;
        objects.push_back(support);
    }
    if(req.container.size()){
        gtp_ros_msg::Obj container;
        container.actionKey = "containerObject";
        container.objectName = req.container;
        objects.push_back(container);
    }
    if(req.targetAgent.size()){
        gtp_ros_msg::Ag agent;
        agent.actionKey = "targetAgent";
        agent.agentName = req.targetAgent;
        agents.push_back(agent);
    }
    if(req.confName.size()){
        gtp_ros_msg::Data data;
        data.dataKey = "confName";
        data.dataValue = req.confName;
        datas.push_back(data);
    }
    if(req.arm.size()){
        gtp_ros_msg::Data data;
        data.dataKey = "hand";
        data.dataValue = req.arm;
        datas.push_back(data);
    }

    goal.req.involvedAgents = agents;
    goal.req.involvedObjects = objects;
    goal.req.data = datas;
    goal.req.points = points;
    goal.req.predecessorId.actionId = req.prevId;
    goal.req.predecessorId.alternativeId = 0;

   acGTP->sendGoal(goal);
   finishedBeforeTimeout = acGTP->waitForResult(ros::Duration(waitActionServer));

   if (finishedBeforeTimeout)
   {
     if(acGTP->getResult()->ans.success){
         gtpTasks[acGTP->getResult()->ans.identifier.actionId] = req;
         return acGTP->getResult()->ans.identifier.actionId;
     }else{
         ROS_INFO("[action_executor] GTP no plan found.");
         return -1;
     }
   }
   else{
       ROS_INFO("[action_executor] GTP Planning did not finish before the time out.");
       return -1;
   }

     return -1;
}


/*
Function which open a gripper
*/
void openGripper(int armId){

    /*bool finishedBeforeTimeout;
    if(armId == 0){//right arm
       pr2motion::Gripper_Right_OperateGoal gripper_goal;
       gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_OPEN;
       PR2motion_gripper_right->sendGoal(gripper_goal);
       finishedBeforeTimeout = PR2motion_gripper_right->waitForResult(ros::Duration(waitActionServer));
       if(!finishedBeforeTimeout){
         ROS_INFO("[action_executor] PR2motion Action did not finish before the time out.");
       }
    }else{
       pr2motion::Gripper_Left_OperateGoal gripper_goal;
       gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_OPEN;
       PR2motion_gripper_left->sendGoal(gripper_goal);
       finishedBeforeTimeout = PR2motion_gripper_left->waitForResult(ros::Duration(waitActionServer));
       if(!finishedBeforeTimeout){
         ROS_INFO("[action_executor] PR2motion Action did not finish before the time out.");
       }
    }*/
    
    bool finishedBeforeTimeout;
   pr2_controllers_msgs::Pr2GripperCommandGoal open_cmd;
   open_cmd.command.position = 0.08;
   open_cmd.command.max_effort = -1.0;
   if(armId == 0){//right arm
       gripper_right->sendGoal(open_cmd);
       finishedBeforeTimeout = gripper_right->waitForResult(ros::Duration(waitActionServer));
    }else{
   gripper_left->sendGoal(open_cmd);
       finishedBeforeTimeout = gripper_left->waitForResult(ros::Duration(waitActionServer));
    }
    if(!finishedBeforeTimeout){
      ROS_INFO("[action_executor] gripper Action did not finish before the time out.");
    }

}

/*
Function which close a gripper
*/
void closeGripper(int armId){

    /*bool finishedBeforeTimeout;
    if(armId == 0){//right arm
       pr2motion::Gripper_Right_OperateGoal gripper_goal;
       gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_CLOSE;
       PR2motion_gripper_right->sendGoal(gripper_goal);
       finishedBeforeTimeout = PR2motion_gripper_right->waitForResult(ros::Duration(waitActionServer));
       if(!finishedBeforeTimeout){
         ROS_INFO("[action_executor] PR2motion Action did not finish before the time out.");
       }
    }else{
       pr2motion::Gripper_Left_OperateGoal gripper_goal;
       gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_CLOSE;
       PR2motion_gripper_left->sendGoal(gripper_goal);
       finishedBeforeTimeout = PR2motion_gripper_left->waitForResult(ros::Duration(waitActionServer));
       if(!finishedBeforeTimeout){
         ROS_INFO("[action_executor] PR2motion Action did not finish before the time out.");
       }
    }*/
    
    bool finishedBeforeTimeout;
    pr2_controllers_msgs::Pr2GripperCommandGoal open_cmd;
   open_cmd.command.position = 0.0;
   open_cmd.command.max_effort = -1.0;
   if(armId == 0){//right arm
       gripper_right->sendGoal(open_cmd);
       finishedBeforeTimeout = gripper_right->waitForResult(ros::Duration(waitActionServer));
    }else{
   gripper_left->sendGoal(open_cmd);
       finishedBeforeTimeout = gripper_left->waitForResult(ros::Duration(waitActionServer));
    }
    if(!finishedBeforeTimeout){
      ROS_INFO("[action_executor] gripper Action did not finish before the time out.");
    }


}


/*
Function which puts an object in the hand of the robot
    @object: the object to put
    @hand: the hand to attach to (right or left)
*/
void PutInHand(std::string object, std::string hand){

   ros::ServiceClient client = node->serviceClient<toaster_msgs::PutInHand>("pdg/put_in_hand");

    //put the object in the hand of the robot
    std::string robotHand;
    std::string handTopic = "/robot/hands/";
    handTopic = handTopic + hand;
    node->getParam(handTopic, robotHand);
    std::string robotToasterName;
    node->getParam("robot/toasterName", robotToasterName);
    toaster_msgs::PutInHand srv;
   srv.request.objectId = object;
   srv.request.agentId = robotToasterName;
   srv.request.jointName = robotHand;
   if (!client.call(srv)){
     ROS_ERROR("[action_executor] Failed to call service pdg/put_in_hand");
    }

}

/*
Function which remove an object from the hand of the robot
    @object: the object to remove
*/
void RemoveFromHand(std::string object){

   ros::ServiceClient client = node->serviceClient<toaster_msgs::RemoveFromHand>("pdg/remove_from_hand");

    //remove the object from the hand of the robot
    toaster_msgs::RemoveFromHand srv;
   srv.request.objectId = object;
   if (!client.call(srv)){
     ROS_ERROR("[action_executor] Failed to call service pdg/remove_from_hand");
    }

}

/*
Function which execute a trajectory
*/
void executeTrajectory(int actionId, int actionSubId, int armId){

   gtp_ros_msg::requestGoal goal;
   goal.req.requestType = "load";
   goal.req.loadAction.actionId = actionId;
   goal.req.loadAction.alternativeId = 0;
   goal.req.loadSubTraj = actionSubId;

   acGTP->sendGoal(goal);
   bool finishedBeforeTimeout = acGTP->waitForResult(ros::Duration(waitActionServer));

   if (finishedBeforeTimeout){
     if(armId == 0){//right arm
        pr2motion::Arm_Right_MoveGoal arm_goal_right;
        arm_goal_right.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
        arm_goal_right.path_mode.value=pr2motion::pr2motion_PATH_MODE::pr2motion_PATH_PORT;
        PR2motion_arm_right->sendGoal(arm_goal_right);
        finishedBeforeTimeout = PR2motion_arm_right->waitForResult(ros::Duration(waitActionServer));
        if(!finishedBeforeTimeout){
            ROS_INFO("[action_executor] pr2motion action did not finish before the time out.");
            return;
        }
     }else{
        pr2motion::Arm_Left_MoveGoal arm_goal_left;
        arm_goal_left.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
        arm_goal_left.path_mode.value=pr2motion::pr2motion_PATH_MODE::pr2motion_PATH_PORT;
        PR2motion_arm_left->sendGoal(arm_goal_left);
        finishedBeforeTimeout = PR2motion_arm_left->waitForResult(ros::Duration(waitActionServer));
        if(!finishedBeforeTimeout){
            ROS_INFO("[action_executor] pr2motion action did not finish before the time out.");
            return;
        }

     }
   }
   else{
    ROS_INFO("[action_executor] GTP load did not finish before the time out.");
   }

}

/*
Executes a gtp task
*/
void execGtpTask(action_tester::ExecuteAction::Request  &req, int id){

    gtp_ros_msg::requestGoal goal;
    goal.req.requestType = "details";
    goal.req.loadAction.actionId = id;
    goal.req.loadAction.alternativeId = 0;

    acGTP->sendGoal(goal);
    bool finishedBeforeTimeout = acGTP->waitForResult(ros::Duration(waitActionServer));

    if (finishedBeforeTimeout){
       std::vector<gtp_ros_msg::SubTraj> subTrajs = acGTP->getResult()->ans.subTrajs;
       if(req.actionName == "pick"){//the robot should have the gripper open to execute the trajectory
          openGripper(subTrajs[0].armId);
       }
       for(std::vector<gtp_ros_msg::SubTraj>::iterator it = subTrajs.begin(); it != subTrajs.end(); it++){
           if(it->agent == robotName){
              if(it->subTrajName == "grasp"){
                  closeGripper(it->armId);
                  std::string hand;
                  if(it->armId == 0){
                      hand = "right";
                  }else{
                      hand = "left";
                  }
                  PutInHand(req.object, hand);
              }else if(it->subTrajName == "release"){
                  openGripper(it->armId);
                  RemoveFromHand(req.object);
              }else{//this is a trajectory
                  executeTrajectory(id, it->subTrajId, it->armId);
              }
           }
       }
    }else{
       ROS_INFO("[action_executor] GTP get details did not finish before the time out.");
    }

}

/*
Look at the object of the action
*/
void lookAt(action_tester::ExecuteAction::Request  &req){

    std::string object;
    //first we choose the object
    if(req.actionName == "pick"){
        object = req.object;
    }else if(req.actionName == "place"){
        object = req.support;
    }else if(req.actionName == "drop"){
        object = req.container;
    }else{//no object to look
        return;
    }

    //we get the coordonates of the object
    toaster_msgs::ObjectListStamped objectList;
    double x,y,z;
   try{
       objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
       for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
         if(it->meEntity.id == object){
            x = it->meEntity.pose.position.x;
            y = it->meEntity.pose.position.y;
            z = it->meEntity.pose.position.z;
            break;
         }
       }
   }
    catch(const std::exception & e){
        ROS_WARN("[action_executor] Failed to read %s pose from toaster", object.c_str());
    }

    //we look at the object
    pr2motion::Head_Move_TargetGoal goal;
    goal.head_mode.value = 0;
    goal.head_target_frame = "map";
    goal.head_target_x = x;
    goal.head_target_y = y;
    goal.head_target_z = z;
    head_action_client->sendGoal(goal);

    bool finishedBeforeTimeout = head_action_client->waitForResult(ros::Duration(waitActionServer));

    if (!finishedBeforeTimeout){
        ROS_INFO("[action_executor] pr2motion head action did not finish before the time out.");
    }
}


/*
Service call to execute an action
*/
bool execAction(action_tester::ExecuteAction::Request  &req, action_tester::ExecuteAction::Response &res){

    lookAt(req);
    int gtpId = planGTP(req);

    if(gtpId != -1){
        execGtpTask(req, gtpId);
    }

    ROS_INFO("[action_tester] Action executed: id = %d", gtpId);

    return true;
}

/*
Service call to execute an action
*/
bool execSubTraj(action_tester::ExecuteSubTraj::Request  &req, action_tester::ExecuteSubTraj::Response &res){

    executeTrajectory(req.actionId, req.actionSubId, req.armId);

    ROS_INFO("[action_tester] Subtraj executed: id = %d %d", req.actionId, req.actionSubId);

    return true;
}

/*
Service call to execute an action
*/
bool controlGripper(action_tester::ControlGripper::Request  &req, action_tester::ControlGripper::Response &res){

    if(req.open){
        openGripper(req.armId);
    }else{
        closeGripper(req.armId);
    }

    return true;
}

/*
Service call to execute an action
*/
bool planService(action_tester::ExecuteAction::Request  &req, action_tester::ExecuteAction::Response &res){

    lookAt(req);
    int gtpId = planGTP(req);

    ROS_INFO("[action_tester] Action planned: id = %d", gtpId);

    return true;
}




/*
Service call to execute a gtp task
*/
bool execTask(action_tester::ExecuteTask::Request  &req, action_tester::ExecuteTask::Response &res){

    lookAt(gtpTasks[req.id]);
    execGtpTask(gtpTasks[req.id], req.id);

    return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_tester");
  ros::NodeHandle _node;
  node = &_node;

  ROS_INFO("[action_tester] Init action_tester");
 
  //Services declarations
  ros::ServiceServer service_action = _node.advertiseService("action_tester/execute_action", execAction);
  ros::ServiceServer service_task = _node.advertiseService("action_tester/execute_gtp_task", execTask);
  ros::ServiceServer service_subtraj = _node.advertiseService("action_tester/execute_subtraj", execSubTraj);
  ros::ServiceServer service_gripper = _node.advertiseService("action_tester/control_gripper", controlGripper);
  ros::ServiceServer service_plan = _node.advertiseService("action_tester/plan", planService);

  node->getParam("/robot/name", robotName);
  node->getParam("/waitActionServer", waitActionServer);

  ROS_INFO("[action_tester] Waiting for GTP action server");
  acGTP = new actionlib::SimpleActionClient<gtp_ros_msg::requestAction>("gtp_ros_server", true);
  acGTP->waitForServer();
  ROS_INFO("[action_tester] Waiting for pr2motion action server");
  PR2motion_init = new actionlib::SimpleActionClient<pr2motion::InitAction>("pr2motion/Init", true);
  PR2motion_init->waitForServer();
  PR2motion_arm_right = new actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction>("pr2motion/Arm_Right_Move",true);
  PR2motion_arm_right->waitForServer();
  PR2motion_arm_left = new actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveAction>("pr2motion/Arm_Left_Move",true);
  PR2motion_arm_left->waitForServer();
  PR2motion_gripper_right = new actionlib::SimpleActionClient<pr2motion::Gripper_Right_OperateAction>("pr2motion/Gripper_Right_Operate",true);
  PR2motion_gripper_right->waitForServer();
  PR2motion_gripper_left = new actionlib::SimpleActionClient<pr2motion::Gripper_Left_OperateAction>("pr2motion/Gripper_Left_Operate",true);
  PR2motion_gripper_left->waitForServer();
  head_action_client = new actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction>("pr2motion/Head_Move_Target",true);
  head_action_client->waitForServer();
  
  ROS_INFO("[action_tester] Waiting for gripper action server");
  gripper_right = new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>("r_gripper_sensor_controller/gripper_action", true);
  gripper_right->waitForServer();
  gripper_left = new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>("l_gripper_sensor_controller/gripper_action", true);
  gripper_left->waitForServer();
 

  ROS_INFO("[action_tester] Init pr2motion");
  ros::ServiceClient connect = node->serviceClient<pr2motion::connect_port>("pr2motion/connect_port");
  pr2motion::InitGoal goal_init;
  PR2motion_init->sendGoal(goal_init);

  pr2motion::connect_port srv;
  srv.request.local = "joint_state";
  srv.request.remote = "joint_states";
  if (!connect.call(srv)){
     ROS_ERROR("[action_executor] Failed to call service pr2motion/connect_port");
  }
  srv.request.local = "head_controller_state";
  srv.request.remote = "/head_traj_controller/state";
  if (!connect.call(srv)){
      ROS_ERROR("[action_executor] Failed to call service pr2motion/connect_port");
  }
  srv.request.local = "traj";
  srv.request.remote = "gtp_trajectory";
  if (!connect.call(srv)){
      ROS_ERROR("[action_executor] Failed to call service pr2motion/connect_port");
  }


  ROS_INFO("[action_tester] action_tester ready");

  ros::spin();

  return 0;
}
