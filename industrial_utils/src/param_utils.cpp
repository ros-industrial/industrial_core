/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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

#include <sstream>

#include "industrial_utils/param_utils.h"
#include "ros/ros.h"
#include "urdf/model.h"

namespace industrial_utils
{
namespace param
{
bool getListParam(const std::string param_name, std::vector<std::string> & list_param)
{
  bool rtn = false;
  XmlRpc::XmlRpcValue rpc_list;

  list_param.clear(); //clear out return value

  rtn = ros::param::get(param_name, rpc_list);

  if (rtn)
  {
    rtn = (rpc_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    if (rtn)
    {

      for (int i = 0; i < rpc_list.size(); ++i)
      {
        rtn = (rpc_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        if (rtn)
        {
          ROS_INFO_STREAM("Adding " << rpc_list[i] << " to list parameter");
          list_param.push_back(static_cast<std::string>(rpc_list[i]));
        }
        else
        {
          ROS_ERROR_STREAM("List item for: " << param_name << " not of string type");
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM("Parameter: " << param_name << " not of list type");
    }
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get parameter: " << param_name);
  }

  return rtn;

}

bool getJointNames(const std::string param_name, std::vector<std::string> & joint_names)
{
  if (ros::param::has(param_name) && getListParam(param_name, joint_names))
    return true;

  const int NUM_JOINTS = 6;  //Most robots have 6 joints

  joint_names.clear();
  for (int i=0; i<NUM_JOINTS; ++i)
  {
    std::stringstream tmp;
    tmp << "joint_" << i+1;
    joint_names.push_back(tmp.str());
  }

    return false;
}

bool getJointVelocityLimits(const std::string urdf_param_name, std::map<std::string, double> &velocity_limits)
{
  urdf::Model model;
  std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator iter;

  if (!ros::param::has(urdf_param_name) || !model.initParam(urdf_param_name))
    return false;
    
  velocity_limits.clear();
  for (iter=model.joints_.begin(); iter!=model.joints_.end(); ++iter)
  {
    std::string joint_name(iter->first);
    double limit = iter->second->limits->velocity;
    velocity_limits.insert(std::pair<std::string,double>(joint_name,limit));
  }
  
  return true;
}

} //industrial_utils::param
} //industrial_utils
