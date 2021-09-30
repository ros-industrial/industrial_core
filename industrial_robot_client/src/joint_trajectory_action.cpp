/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <industrial_robot_client/joint_trajectory_action.h>
#include <industrial_robot_client/utils.h>
#include <industrial_utils/param_utils.h>
#include <industrial_utils/utils.h>

namespace industrial_robot_client
{
namespace joint_trajectory_action
{

const double JointTrajectoryAction::WATCHDOG_PERIOD_ = 1.0;
const double JointTrajectoryAction::DEFAULT_GOAL_THRESHOLD_ = 0.01;

JointTrajectoryAction::JointTrajectoryAction() :
    action_server_(node_, "joint_trajectory_action", boost::bind(&JointTrajectoryAction::goalCB, this, _1),
                   boost::bind(&JointTrajectoryAction::cancelCB, this, _1), false), has_active_goal_(false),
                   controller_alive_(false), has_moved_once_(false), name_("joint_trajectory_action")
{
  ros::NodeHandle pn("~");

  pn.param("constraints/goal_threshold", goal_threshold_, DEFAULT_GOAL_THRESHOLD_);

  // Two parameters for bw-compatibility with the 'old' behaviour.
  pn.param("ignore_motion_server_error", ignore_motion_server_error_, ignore_motion_server_error_);
  pn.param("consider_status_unknowns_ok", consider_status_unknowns_ok_, consider_status_unknowns_ok_);
  std::string log_msg = std::string("Ignoring motion server errors: ") + (ignore_motion_server_error_ ? "true" : "false");
  if (ignore_motion_server_error_)
    ROS_WARN_STREAM_NAMED(name_, log_msg);
  else
    ROS_INFO_STREAM_NAMED(name_, log_msg);
  log_msg = std::string("Treating RobotStatus fields with UNKNOWNs as OK: ") + (consider_status_unknowns_ok_ ? "true" : "false");
  if (consider_status_unknowns_ok_)
    ROS_WARN_STREAM_NAMED(name_, log_msg);
  else
    ROS_INFO_STREAM_NAMED(name_, log_msg);

  if (!industrial_utils::param::getJointNames("controller_joint_names", "robot_description", joint_names_))
    ROS_ERROR_NAMED(name_, "Failed to initialize joint_names.");

  // The controller joint names parameter includes empty joint names for those joints not supported
  // by the controller.  These are removed since the trajectory action should ignore these.
  std::remove(joint_names_.begin(), joint_names_.end(), std::string());
  ROS_INFO_STREAM_NAMED(name_, "Filtered joint names to " << joint_names_.size() << " joints");

  pub_trajectory_command_ = node_.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);
  sub_trajectory_state_ = node_.subscribe("feedback_states", 1, &JointTrajectoryAction::controllerStateCB, this);
  sub_robot_status_ = node_.subscribe("robot_status", 1, &JointTrajectoryAction::robotStatusCB, this);

  watchdog_timer_ = node_.createTimer(ros::Duration(WATCHDOG_PERIOD_), &JointTrajectoryAction::watchdog, this, true);
  action_server_.start();
}

JointTrajectoryAction::~JointTrajectoryAction()
{
}

void JointTrajectoryAction::robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
{
  last_robot_status_ = msg; //caching robot status for later use.
  has_moved_once_ = has_moved_once_ ? true : (last_robot_status_->in_motion.val == industrial_msgs::TriState::TRUE);
}

void JointTrajectoryAction::watchdog(const ros::TimerEvent &e)
{
  // Some debug logging
  if (!last_trajectory_state_)
  {
    ROS_DEBUG_NAMED(name_, "Waiting for subscription to joint trajectory state");
  }

  ROS_WARN_NAMED(name_, "Trajectory state not received for %f seconds", WATCHDOG_PERIOD_);
  controller_alive_ = false;


  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_)
  {
    // last_trajectory_state_ is null if the subscriber never makes a connection
    if (!last_trajectory_state_)
    {
      ROS_WARN_NAMED(name_, "Aborting goal because we have never heard a controller state message.");
    }
    else
    {
      ROS_WARN_STREAM_NAMED(name_,
          "Aborting goal because we haven't heard from the controller in " << WATCHDOG_PERIOD_ << " seconds");
    }

    abortGoal();
  }
}

bool isMotionServerOK(industrial_msgs::RobotStatusConstPtr& msg, bool unknown_is_ok = false)
{
  // unless it's OK for values to be UNKNOWN, the state relay must report
  //  - motion_possible == true
  //  - in_error == false
  //  - e_stopped == false
  //  - no error code
  return utils::tri_state::isOn(msg->motion_possible, unknown_is_ok)
    && msg->error_code == 0
    && utils::tri_state::isOff(msg->in_error, unknown_is_ok)
    && utils::tri_state::isOff(msg->e_stopped, unknown_is_ok);
}

std::string describeRobotStatusMsg(industrial_msgs::RobotStatusConstPtr& msg, bool unknown_is_on = false)
{
  std::stringstream ss;

  // mention e-stop specifically
  if (utils::tri_state::isOn(msg->e_stopped, unknown_is_on))
  {
    ss.clear();
    ss << "robot controller reported e-stop";
  }
  // some (generic ?) other error
  else if (msg->error_code != 0 || utils::tri_state::isOn(msg->in_error, unknown_is_on))
  {
    ss.clear();
    ss << "robot controller reported (active) error";

    // it could be state server does not report specific error codes
    if (msg->error_code != 0)
    {
      ss << " (OEM code: " << msg->error_code << ")";
    }
    else
    {
      ss << " (0 or no OEM error code communicated)";
    }
  }
  // we use this last, as it could be we currently don't yet know *why*
  // the state server decides motion_possible == false. So we first
  // check the specific problems above, then fall back to this generic
  // "it doesn't work, don't know why" statement
  else if (utils::tri_state::isOff(msg->motion_possible, unknown_is_on))
  {
    ss.clear();
    ss << "controller reported motion not possible (no further information)";
  }

  return ss.str();
}

void JointTrajectoryAction::goalCB(JointTractoryActionServer::GoalHandle gh)
{
  ROS_INFO_STREAM_NAMED(name_, "Received new goal");

  // reject all goals as long as we haven't heard from the remote controller
  if (!controller_alive_)
  {
    ROS_ERROR_NAMED(name_, "Joint trajectory action rejected: waiting for (initial) feedback from controller");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Waiting for (initial) feedback from controller");

    // no point in continuing: already rejected
    return;
  }

  // check robot can actually execute trajectory, if not, refuse goal.
  // no point in accepting the goal only to cancel it immediately later
  if (!isMotionServerOK(last_robot_status_, consider_status_unknowns_ok_) && !ignore_motion_server_error_)
  {
    // translate status into user readable description
    const std::string reject_msg = {"Rejecting goal: "
      + describeRobotStatusMsg(last_robot_status_, consider_status_unknowns_ok_) };
    ROS_ERROR_STREAM_NAMED(name_, reject_msg);
    control_msgs::FollowJointTrajectoryResult rslt;
    // the goal is actually probably OK, but we have to choose one of the existing
    // constants, and this one comes closest
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    rslt.error_string = reject_msg;
    gh.setRejected(rslt, reject_msg);

    // no point in continuing: already rejected
    return;
  }

  if (!gh.getGoal()->trajectory.points.empty())
  {
    if (industrial_utils::isSimilar(joint_names_, gh.getGoal()->trajectory.joint_names))
    {

      // Cancels the currently active goal.
      if (has_active_goal_)
      {
        ROS_WARN_NAMED(name_, "Received new goal, canceling current goal");
        abortGoal();
      }

      gh.setAccepted();
      active_goal_ = gh;
      has_active_goal_ = true;
      time_to_check_ = ros::Time::now() +
          ros::Duration(active_goal_.getGoal()->trajectory.points.back().time_from_start.toSec() / 2.0);
      has_moved_once_ = false;

      ROS_INFO_STREAM_NAMED(name_, "Publishing trajectory");

      current_traj_ = active_goal_.getGoal()->trajectory;
      pub_trajectory_command_.publish(current_traj_);

    }
    else
    {
      ROS_ERROR_NAMED(name_, "Joint trajectory action failing on invalid joints");
      control_msgs::FollowJointTrajectoryResult rslt;
      rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(rslt, "Joint names do not match");
    }
  }
  else
  {
    ROS_ERROR_NAMED(name_, "Joint trajectory action failed on empty trajectory");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Empty trajectory");
  }

  // Adding some informational log messages to indicate unsupported goal constraints
  if (gh.getGoal()->goal_time_tolerance.toSec() > 0.0)
  {
    ROS_WARN_STREAM_NAMED(name_, "Ignoring goal time tolerance in action goal, may be supported in the future");
  }
  if (!gh.getGoal()->goal_tolerance.empty())
  {
    ROS_WARN_STREAM_NAMED(name_,
        "Ignoring goal tolerance in action, using paramater tolerance of " << goal_threshold_ << " instead");
  }
  if (!gh.getGoal()->path_tolerance.empty())
  {
    ROS_WARN_STREAM_NAMED(name_, "Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
  }
}

void JointTrajectoryAction::cancelCB(JointTractoryActionServer::GoalHandle gh)
{
  ROS_DEBUG_NAMED(name_, "Received action cancel request");
  if (active_goal_ == gh)
  {
    // Stops the controller.
    stopRelay();

    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }
  else
  {
    ROS_WARN_NAMED(name_, "Active goal and goal cancel do not match, ignoring cancel request");
  }
}

void JointTrajectoryAction::controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
{
  ROS_DEBUG_STREAM_NAMED(name_, "Checking controller state feedback");

  last_trajectory_state_ = msg;
  controller_alive_ = true;

  watchdog_timer_.stop();
  watchdog_timer_.start();

  if (!has_active_goal_)
  {
    //ROS_DEBUG_NAMED(name_, "No active goal, ignoring feedback");
    return;
  }
  if (current_traj_.points.empty())
  {
    ROS_INFO_NAMED(name_, "Current trajectory is empty, ignoring feedback");
    return;
  }

  if (!industrial_utils::isSimilar(joint_names_, msg->joint_names))
  {
    ROS_ERROR_NAMED(name_, "Joint names from the controller don't match our joint names.");
    return;
  }

  // see if we need to abort the goal due to on-controller errors.
  // NOTE: we do this *before* checking has_moved_once_, as otherwise we would not
  // notice problems on the controller side unless the robot has already moved,
  // which it may be unable to do.
  if(!isMotionServerOK(last_robot_status_, consider_status_unknowns_ok_) && !ignore_motion_server_error_)
  {
    const std::string abort_msg = {"Aborting goal: "
      + describeRobotStatusMsg(last_robot_status_, consider_status_unknowns_ok_) };

    // Stop the relay
    stopRelay();

    // return abort to action client
    control_msgs::FollowJointTrajectoryResult result;
    // would like to use a better error constant, but we have to choose one of the existing
    // constants, and this one comes closest
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = abort_msg;
    active_goal_.setAborted(result, abort_msg);
    has_active_goal_ = false;
    ROS_ERROR_STREAM_NAMED(name_, abort_msg);
    return;
  }

  if (!has_moved_once_ && (ros::Time::now() < time_to_check_))
  {
    ROS_DEBUG_NAMED(name_, "Waiting to check for goal completion until halfway through trajectory");
    return;
  }

  // Checking for goal constraints
  // Checks that we have ended inside the goal constraints and has motion stopped

  ROS_DEBUG_STREAM_NAMED(name_, "Checking goal constraints");
  if (withinGoalConstraints(last_trajectory_state_, current_traj_))
  {
    if (last_robot_status_)
    {
      // Additional check for motion stoppage since the controller goal may still
      // be moving.  The current robot driver calls a motion stop if it receives
      // a new trajectory while it is still moving.  If the driver is not publishing
      // the motion state (i.e. old driver), this will still work, but it warns you.
      if (utils::tri_state::isOff(last_robot_status_->in_motion, /*unknown_is_off=*/false))
      {
        ROS_INFO_NAMED("joint_trajectory_action.controllerStateCB", "Inside goal constraints - stopped moving - return success for action");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else if (utils::tri_state::isUnknown(last_robot_status_->in_motion))
      {
        ROS_INFO_NAMED(name_, "Inside goal constraints, return success for action");
        ROS_WARN_NAMED(name_, "Robot status in motion unknown, the robot driver node and controller code should be updated");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else
      {
        ROS_DEBUG_NAMED(name_, "Within goal constraints but robot is still moving");
      }
    }
    else
    {
      ROS_INFO_NAMED(name_, "Inside goal constraints, return success for action");
      ROS_WARN_NAMED(name_, "Robot status is not being published the robot driver node and controller code should be updated");
      active_goal_.setSucceeded();
      has_active_goal_ = false;
    }
  }
}

void JointTrajectoryAction::stopRelay()
{
  // Stops the controller.
  trajectory_msgs::JointTrajectory empty;
  pub_trajectory_command_.publish(empty);
}

void JointTrajectoryAction::abortGoal()
{
  stopRelay();
  // Marks the current goal as aborted.
  active_goal_.setAborted();
  has_active_goal_ = false;
}

bool JointTrajectoryAction::withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
                                                  const trajectory_msgs::JointTrajectory & traj)
{
  bool rtn = false;
  if (traj.points.empty())
  {
    ROS_WARN_NAMED(name_, "Empty joint trajectory passed to check goal constraints, return false");
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
