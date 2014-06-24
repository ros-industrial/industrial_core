#ifndef FLATHEADERS
#include "simple_message/joint_traj_pt_full_ex.h"
#include "simple_message/joint_traj_pt_full.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "joint_traj_pt_full_ex.h"
#include "joint_traj_pt_full.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::joint_data;
using namespace industrial::shared_types;
using namespace industrial::joint_traj_pt_full;

namespace industrial
{
namespace joint_traj_pt_full_ex
{

JointTrajPtFullEx::JointTrajPtFullEx(void)
{
  this->init();
}
JointTrajPtFullEx::~JointTrajPtFullEx(void)
{

}

void JointTrajPtFullEx::init()
{
  this->num_groups_ = MAX_NUM_GROUPS;
  this->sequence_ = 0;

    for (int i=0;i<MAX_NUM_GROUPS;i++)
    {
        JointTrajPtFull joint_traj_pt_full;

        joint_traj_pt_full.init();

        this->joint_trajectory_points_.push_back(joint_traj_pt_full);
    }

}

void JointTrajPtFullEx::init(industrial::shared_types::shared_int num_groups,
          industrial::shared_types::shared_int sequence,
           std::vector<industrial::joint_traj_pt_full::JointTrajPtFull> joint_trajectory_points)
{
  this->setNumGroups(num_groups);
  this->setSequence(sequence);
  this->setMultiJointTrajPtData(joint_trajectory_points_);
}

void JointTrajPtFullEx::copyFrom(JointTrajPtFullEx &src)
{
  this->setNumGroups(src.num_groups_);
  this->setSequence(src.sequence_);
  this->setMultiJointTrajPtData(src.joint_trajectory_points_);
}

bool JointTrajPtFullEx::operator==(JointTrajPtFullEx &rhs)
{
 //TODO: expand the capabilities of this check
  return this->num_groups_ == rhs.num_groups_;
}

bool JointTrajPtFullEx::load(industrial::byte_array::ByteArray *buffer)
{

  LOG_ERROR("Loading message");

  LOG_COMM("Executing joint trajectory point load");

  if (!buffer->load(this->num_groups_))
  {
    LOG_ERROR("Failed to load joint traj pt. robot_id");
    return false;
  }

  if (!buffer->load(this->sequence_))
  {
    LOG_ERROR("Failed to load joint traj. pt. sequence number");
    return false;
  }

  for(int i=0; i<joint_trajectory_points_.size(); i++)
  {
      JointTrajPtFull traj_full = joint_trajectory_points_[i];

        ROS_ERROR("Loading point %d",i);

      ROS_ERROR("GRROUP ID %d",traj_full.getRobotID() );
      if (!buffer->load(traj_full.getRobotID()))
      {
        LOG_ERROR("Failed to load joint traj pt. robot_id");
        ROS_ERROR("Failed to load joint traj pt. robot_id");
        return false;
      }


      if (!buffer->load(traj_full.getValidFields()))
      {

        LOG_ERROR("Failed to load joint traj. pt. valid fields");
        ROS_ERROR("Failed to load joint traj pt. vfiels");
        return false;
      }

      industrial::shared_types::shared_real this_time;
      traj_full.getTime(this_time);
      ROS_ERROR("TIME %f", this_time);
      if (!buffer->load(this_time))
      {
        LOG_ERROR("Failed to load joint traj. pt. time");
        ROS_ERROR("Failed to load joint traj pt. time");
        return false;
      }

      industrial::joint_data::JointData positions;
      traj_full.getPositions(positions);

      industrial::shared_types::shared_real pos;

      for (int j=0;j < positions.getMaxNumJoints();j++)
      {
          pos = positions.getJoint(j);
          ROS_ERROR("pos:%f",pos);
          if (!buffer->load(pos))
          {
            LOG_ERROR("Failed to load joint traj. pt. positions");
            ROS_ERROR("Failed to load joint traj pt. positions");
            return false;
          }

      }


      industrial::joint_data::JointData velocities;
      traj_full.getVelocities(velocities);
      industrial::shared_types::shared_real vel;

      for (int j=0;j < velocities.getMaxNumJoints();j++)
      {
          vel = velocities.getJoint(j);
          ROS_ERROR("vel:%f",vel);
          if (!buffer->load(vel))
          {
            LOG_ERROR("Failed to load joint traj. pt. positions");
            ROS_ERROR("Failed to load joint traj pt. positions");
            return false;
          }

      }

      industrial::joint_data::JointData accelerations;
      traj_full.getAccelerations(accelerations);
      industrial::shared_types::shared_real acc;

    for (int j=0;j < accelerations.getMaxNumJoints();j++)
    {
        acc = accelerations.getJoint(j);
        ROS_ERROR("acc:%f",acc);
        if (!buffer->load(acc))
        {
          LOG_ERROR("Failed to load joint traj. pt. positions");
          ROS_ERROR("Failed to load joint traj pt. positions");
          return false;
        }

    }

      LOG_COMM("Trajectory point successfully loaded");
      ROS_ERROR("point succesfully loaded");

  }


  LOG_COMM("Trajectory point successfully loaded");
  ROS_ERROR("succesfully loaded");
  return true;
}

bool JointTrajPtFullEx::unload(industrial::byte_array::ByteArray *buffer)
{

  LOG_COMM("Executing joint traj. pt. unload");

  if (!buffer->unload(this->sequence_))
  {
    LOG_ERROR("Failed to unload joint traj. pt. sequence number");
    return false;
  }

  if (!buffer->unload(this->num_groups_))
  {
    LOG_ERROR("Faild to unload joint traj. pt. num_groups");
    return false;
  }


  LOG_COMM("Joint traj. pt successfully unloaded");
  return true;
}

}
}


