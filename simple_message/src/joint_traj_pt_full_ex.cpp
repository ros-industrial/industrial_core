#ifndef FLATHEADERS
#include "simple_message/joint_traj_pt_full_ex.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "joint_traj_pt_full_ex.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::joint_data;
using namespace industrial::shared_types;

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
  this->num_groups_ = 0;
  this->sequence_ = 0;

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

  LOG_COMM("Trajectory point successfully loaded");
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


