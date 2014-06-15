#ifndef FLATHEADERS
#include "simple_message/joint_feedback_ex.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "joint_feedback_ex.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::joint_data;
using namespace industrial::shared_types;
using namespace industrial::joint_feedback_message;
using namespace industrial::joint_feedback;

namespace industrial
{
namespace joint_feedback_ex
{

JointFeedbackEx::JointFeedbackEx(void)
{
  this->init();
}
JointFeedbackEx::~JointFeedbackEx(void)
{

}

void JointFeedbackEx::init()
{
  this->groups_number_ = 0;

}

void JointFeedbackEx::init(industrial::shared_types::shared_int groups_number,
          std::vector<joint_feedback_message::JointFeedbackMessage> joints_feedback_points)
{
  this->setGroupsNumber(groups_number);
  this->joint_feedback_messages_ = joints_feedback_points;
}

void JointFeedbackEx::copyFrom(JointFeedbackEx &src)
{
  this->setGroupsNumber(src.getGroupsNumber());
  this->joint_feedback_messages_ = src.joint_feedback_messages_;
}

bool JointFeedbackEx::operator==(JointFeedbackEx &rhs)
{
  return this->groups_number_ == rhs.groups_number_;
}

bool JointFeedbackEx::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing joint feedback load");

  ROS_ERROR("Executiing load");
  if (!buffer->load(this->groups_number_))
  {
    LOG_ERROR("Failed to load joint feedback groups_number");
    return false;
  }

  LOG_COMM("Joint feedback successfully loaded");
  return true;
}

bool JointFeedbackEx::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing joint feedback unload");

  ROS_ERROR("Executing unload");
  ROS_ERROR("%d",buffer->getBufferSize());

  if (!buffer->unloadFront(this->groups_number_))
  {
    LOG_ERROR("Failed to unload joint feedback groups_number");
    return false;
  }

  for(int i=0;i<this->groups_number_;i++)
  {
  JointFeedbackMessage tmp_msg;
  JointFeedback j_feedback;



  if (!buffer->unload(j_feedback))
  {
    LOG_ERROR("Failed to unload joint feedback groups_number");
    return false;
  }

  tmp_msg.init(j_feedback);

  this->joint_feedback_messages_.push_back(tmp_msg);
}
  ROS_ERROR("Executing unload");
  ROS_ERROR("%d",buffer->getBufferSize());

  LOG_COMM("Joint feedback successfully unloaded");
  return true;
}

}
}


