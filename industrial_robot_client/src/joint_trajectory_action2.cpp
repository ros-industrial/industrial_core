#include <industrial_robot_client/joint_trajectory_action2.h>
#include <industrial_robot_client/utils.h>
#include <industrial_utils/param_utils.h>
#include <industrial_utils/utils.h>

namespace industrial_robot_client
{
namespace joint_trajectory_action2
{

const double JointTrajectoryAction2::WATCHD0G_PERIOD_ = 1.0;
const double JointTrajectoryAction2::DEFAULT_GOAL_THRESHOLD_ = 0.01;

JointTrajectoryAction2::JointTrajectoryAction2() :
    action_server_(node_, "joint_trajectory_action", false)
{




//    action_server_(node_, "joint_trajectory_action2", boost::bind(&JointTrajectoryAction2::goalCB, _1,&action_server_),
//                   boost::bind(&JointTrajectoryAction2::cancelCB,_1,&action_server_), false);

  ros::NodeHandle pn("~");

  pn.param("constraints/goal_threshold", goal_threshold_, DEFAULT_GOAL_THRESHOLD_);

  std::map<int,RobotGroup> robot_groups;

  std::string value;
  ros::param::search("topics_list", value);

  ROS_INFO("%s", value.c_str());

  XmlRpc::XmlRpcValue topics_list_rpc;
  ros::param::get(value,topics_list_rpc);


  std::vector<XmlRpc::XmlRpcValue> topics_list;

  for (int i=0; i < topics_list_rpc.size();i++)
  {
      XmlRpc::XmlRpcValue state_value;
      state_value = topics_list_rpc[i];
      topics_list.push_back(state_value);
  }

  std::vector<XmlRpc::XmlRpcValue> groups_list;
  //TODO: check the consistency of the group numbers
  for (int i=0; i< topics_list[0]["state"].size();i++)
  {
      XmlRpc::XmlRpcValue group_value;
      group_value = topics_list[0]["state"][i];
      groups_list.push_back(group_value);
      std::cout << i << std::endl;
  }


  for (int i=0; i < groups_list.size();i++)
  {
      RobotGroup rg;
      std::vector<std::string> rg_joint_names;

      XmlRpc::XmlRpcValue joints;

      joints = groups_list[i]["group"][0]["joints"];
      for (int jt=0; jt<joints.size(); jt++)
                 rg_joint_names.push_back(static_cast<std::string>(joints[jt]));

      XmlRpc::XmlRpcValue group_number;


      group_number = groups_list[i]["group"][0]["group_number"];
      int group_number_int = static_cast<int>(group_number);

      ROS_ERROR("group_number: %d", static_cast<int>(group_number));

      XmlRpc::XmlRpcValue name;
      std::string name_string;

      name = groups_list[i]["group"][0]["name"];
      name_string = static_cast<std::string>(name);

      ROS_ERROR("name: %s", static_cast<std::string>(name).c_str());

      XmlRpc::XmlRpcValue ns;
      std::string ns_string;

      ns = groups_list[i]["group"][0]["ns"];

      ns_string = static_cast<std::string>(ns);

      ROS_ERROR("ns: %s", static_cast<std::string>(ns).c_str());

      rg.set_group_id(group_number_int);
      rg.set_joint_names(rg_joint_names);
      rg.set_name(name_string);
      rg.set_ns(ns_string);

      std::string joint_path_action_name = ns_string + "/" + name_string;
      ROS_ERROR("%s", joint_path_action_name.c_str());
      robot_groups[group_number] = rg;

      actionServer_ = new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(node_, joint_path_action_name+ "/joint_trajectory_action" , false);
      actionServer_->registerGoalCallback(boost::bind(&JointTrajectoryAction2::goalCB,    this, _1, group_number_int));
      actionServer_->registerCancelCallback(boost::bind(&JointTrajectoryAction2::cancelCB,    this, _1,group_number_int));
//      server.registerGoalCallback(boost::bind(&JointTrajectoryAction2::goalCB,    this, _1));//      server.registerGoalCallback(boost::bind(&JointTrajectoryAction2::goalCB,    this, _1));
      //      server.registerCancelCallback(boost::bind(&JointTrajectoryAction2::cancelCB,    this, _1));
//      server.registerCancelCallback(boost::bind(&JointTrajectoryAction2::cancelCB,    this, _1));

      pub_trajectory_command_ = node_.advertise<industrial_msgs::DynamicJointTrajectory>(joint_path_action_name+"/joint_path_command", 1);
      sub_trajectory_state_ = node_.subscribe(joint_path_action_name+"/feedback_states", 1, &JointTrajectoryAction2::controllerStateCB, this);
      sub_robot_status_ = node_.subscribe("robot_status", 1, &JointTrajectoryAction2::robotStatusCB, this);

      pub_trajectories_[group_number_int] = pub_trajectory_command_;
      sub_trajectories_[group_number_int] = (sub_trajectory_state_);
      sub_status_[group_number_int] = (sub_robot_status_);

      this->act_servers_[group_number_int] = actionServer_;

      this->act_servers_[group_number_int]->start();

  }

  this->robot_groups_ = robot_groups;

  //TODO:create here for each group a subscriber to /namespace/name/feedback_states
  //TODO:create here for each group a subscriber to /namespace/name/robot_status
  //TODO:create here for each group a advertiser to /namespace/name/joint_path_command

    //TODO: if more than one group generate a joint_path_command_ex


  watchdog_timer_ = node_.createTimer(ros::Duration(WATCHD0G_PERIOD_), &JointTrajectoryAction2::watchdog, this);
  action_server_.start();
}

JointTrajectoryAction2::~JointTrajectoryAction2()
{
}

void JointTrajectoryAction2::robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
{
  last_robot_status_ = msg; //caching robot status for later use.
}

void JointTrajectoryAction2::watchdog(const ros::TimerEvent &e)
{
  // Some debug logging
  if (!last_trajectory_state_)
  {
    ROS_DEBUG("Waiting for subscription to joint trajectory state");
  }
  if (!trajectory_state_recvd_)
  {
    ROS_DEBUG("Trajectory state not received since last watchdog");
  }

  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_)
  {
    if (!trajectory_state_recvd_)
    {
      // last_trajectory_state_ is null if the subscriber never makes a connection
      if (!last_trajectory_state_)
      {
        ROS_WARN("Aborting goal because we have never heard a controller state message.");
      }
      else
      {
        ROS_WARN_STREAM(
            "Aborting goal because we haven't heard from the controller in " << WATCHD0G_PERIOD_ << " seconds");
      }

      abortGoal();

    }
  }

  // Reset the trajectory state received flag
  trajectory_state_recvd_ = false;
}

void JointTrajectoryAction2::goalCB(JointTractoryActionServer::GoalHandle & gh, int group_number)
{
  ROS_INFO("Received new goal");
  if (!gh.getGoal()->trajectory.points.empty())
  {
    if (industrial_utils::isSimilar(this->robot_groups_[group_number].get_joint_names(), gh.getGoal()->trajectory.joint_names))
    {

      // Cancels the currently active goal.
      if (has_active_goal_)
      {
        ROS_WARN("Received new goal, canceling current goal");
        abortGoal();
      }

      // Sends the trajectory along to the controller
      if (withinGoalConstraints(last_trajectory_state_, gh.getGoal()->trajectory))
      {

        ROS_INFO_STREAM("Already within goal constraints, setting goal succeeded");
        gh.setAccepted();
        gh.setSucceeded();
        has_active_goal_ = false;

      }
      else
      {
        gh.setAccepted();
        active_goal_ = gh;
        has_active_goal_ = true;

        ROS_INFO("Publishing trajectory");

        current_traj_ = active_goal_.getGoal()->trajectory;
        industrial_msgs::DynamicJointTrajectory dyn_traj;

        for(int i = 0; i < current_traj_.joint_names.size(); i++)
        {
            dyn_traj.points[i].positions = current_traj_.points[i].positions;
            dyn_traj.points[i].positions = current_traj_.points[i].velocities;
            dyn_traj.points[i].positions = current_traj_.points[i].accelerations;
            dyn_traj.points[i].effort = current_traj_.points[i].effort;
            dyn_traj.points[i].time_from_start = current_traj_.points[i].time_from_start;

        }

        dyn_traj.header = current_traj_.header;
        dyn_traj.joint_names = current_traj_.joint_names;
        dyn_traj.num_groups = 1;

        this->pub_trajectories_[group_number].publish(dyn_traj);
      }
    }
    else
    {
      ROS_ERROR("Joint trajectory action failing on invalid joints");
      control_msgs::FollowJointTrajectoryResult rslt;
      rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(rslt, "Joint names do not match");
    }
  }
  else
  {
    ROS_ERROR("Joint trajectory action failed on empty trajectory");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Empty trajectory");
  }

  // Adding some informational log messages to indicate unsupported goal constraints
  if (gh.getGoal()->goal_time_tolerance.toSec() > 0.0)
  {
    ROS_WARN_STREAM("Ignoring goal time tolerance in action goal, may be supported in the future");
  }
  if (!gh.getGoal()->goal_tolerance.empty())
  {
    ROS_WARN_STREAM(
        "Ignoring goal tolerance in action, using paramater tolerance of " << goal_threshold_ << " instead");
  }
  if (!gh.getGoal()->path_tolerance.empty())
  {
    ROS_WARN_STREAM("Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
  }
}

void JointTrajectoryAction2::cancelCB(JointTractoryActionServer::GoalHandle & gh,int group_number)
{
  ROS_DEBUG("Received action cancel request");
  if (active_goal_ == gh)
  {
    // Stops the controller.
    trajectory_msgs::JointTrajectory empty;
    empty.joint_names = joint_names_;
    pub_trajectory_command_.publish(empty);

    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }
  else
  {
    ROS_WARN("Active goal and goal cancel do not match, ignoring cancel request");
  }
}

void JointTrajectoryAction2::controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
{
  //ROS_DEBUG("Checking controller state feedback");
  last_trajectory_state_ = msg;
  trajectory_state_recvd_ = true;

  if (!has_active_goal_)
  {
    //ROS_DEBUG("No active goal, ignoring feedback");
    return;
  }
  if (current_traj_.points.empty())
  {
    ROS_DEBUG("Current trajectory is empty, ignoring feedback");
    return;
  }

  if (!industrial_utils::isSimilar(joint_names_, msg->joint_names))
  {
    ROS_ERROR("Joint names from the controller don't match our joint names.");
    return;
  }

  // Checking for goal constraints
  // Checks that we have ended inside the goal constraints and has motion stopped

  ROS_DEBUG("Checking goal constraints");
  if (withinGoalConstraints(last_trajectory_state_, current_traj_))
  {
    if (last_robot_status_)
    {
      // Additional check for motion stoppage since the controller goal may still
      // be moving.  The current robot driver calls a motion stop if it receives
      // a new trajectory while it is still moving.  If the driver is not publishing
      // the motion state (i.e. old driver), this will still work, but it warns you.
      if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
      {
        ROS_INFO("Inside goal constraints, stopped moving, return success for action");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
      {
        ROS_INFO("Inside goal constraints, return success for action");
        ROS_WARN("Robot status in motion unknown, the robot driver node and controller code should be updated");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else
      {
        ROS_DEBUG("Within goal constraints but robot is still moving");
      }
    }
    else
    {
      ROS_INFO("Inside goal constraints, return success for action");
      ROS_WARN("Robot status is not being published the robot driver node and controller code should be updated");
      active_goal_.setSucceeded();
      has_active_goal_ = false;
    }
  }
}

void JointTrajectoryAction2::abortGoal()
{
  // Stops the controller.
  trajectory_msgs::JointTrajectory empty;
  pub_trajectory_command_.publish(empty);

  // Marks the current goal as aborted.
  active_goal_.setAborted();
  has_active_goal_ = false;
}

bool JointTrajectoryAction2::withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
                                                  const trajectory_msgs::JointTrajectory & traj)
{
  bool rtn = false;
  if (traj.points.empty())
  {
    ROS_WARN("Empty joint trajectory passed to check goal constraints, return false");
    rtn = false;
  }
  else
  {
    int last_point = traj.points.size() - 1;

    if (industrial_robot_client::utils::isWithinRange(last_trajectory_state_->joint_names,
                                                      last_trajectory_state_->actual.positions, traj.joint_names,
                                                      traj.points[last_point].positions, goal_threshold_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
    }
  }
  return rtn;
}

} //joint_trajectory_action
} //industrial_robot_client


