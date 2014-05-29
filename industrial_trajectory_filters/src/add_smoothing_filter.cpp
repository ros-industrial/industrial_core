/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Southwest Research Institute
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Southwest Research Institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Chris Lewis */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.h>

#include <industrial_trajectory_filters/smoothing_trajectory_filter.h>
#include <ros/ros.h>  // required for NodeHandle
#include <ros/console.h>

namespace industrial_trajectory_filters
{

class AddSmoothingFilter : public planning_request_adapter::PlanningRequestAdapter
{
public:

  static const std::string FILTER_PARAMETER_NAME_; // base name for filter parameters

  AddSmoothingFilter() : planning_request_adapter::PlanningRequestAdapter(), nh_("~")
  {
      int num_coef;

    // set of default filter coefficients in case paramter, or its associated file, or its syntax is wrong
      filter_coef_.push_back(0.25);
      filter_coef_.push_back(0.5);
      filter_coef_.push_back(1.0);
      filter_coef_.push_back(0.5);
      filter_coef_.push_back(0.25);

    // see if a new set of filter parameters is on the parameter server, if so, install them
    if (!nh_.getParam(FILTER_PARAMETER_NAME_, filter_base_name_)){
      ROS_INFO_STREAM("Param '" << FILTER_PARAMETER_NAME_ << "' was not set. Using default filter values " );
    }
    else{ // base filter name exists
      std::string filter_order_param = filter_base_name_ + "/num_coeficients";
      if(!nh_.getParam(filter_order_param, num_coef)){
	ROS_INFO_STREAM("Param '" << filter_order_param << "' was not set. Using default filter values " );
      }
      else{ // num_coef exists for this base name
	if(num_coef %2 == 0){
	  ROS_ERROR("Smoothing filters must have odd number of coeficients");
	}
	else{
	  std::vector<double> temp_coef;
	  for(int i=0; i<num_coef; i++){
	    char post[100];
	    sprintf(post, "/coeficient_%d", i);
	    std::string coef_name = filter_base_name_ + post;
	    double coef;
	    if(!nh_.getParam(coef_name, coef)){ // this order of coef exist for this base name 
	      ROS_INFO_STREAM("Param '" << coef_name << "' was not set. Using default filter values " );
	      break;
	    }
	    temp_coef.push_back(coef);
	  }
	  filter_coef_.clear();
	  for(int i=0; i<num_coef; i++){ // install the filter coefficients
	    filter_coef_.push_back(temp_coef[i]);
	  }
	} // odd number of coef?
      }
    }
    smoothing_filter_ = new industrial_trajectory_filters::SmoothingTrajectoryFilter(); //construct the filter
    if(!smoothing_filter_->Init(filter_coef_))
      ROS_ERROR("Initialization error on smoothing filter. Requires an odd number of coeficients");
  };

  virtual std::string getDescription() const { return "Add Smoothing Trajectory Filter"; }

  virtual bool adaptAndPlan(const PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest &req,
                            planning_interface::MotionPlanResponse &res,
                            std::vector<std::size_t> &added_path_index) const
  {
    // do anything prior to calling the planner here
    // ....
    // IN this case nothing was done

    // call the nested planner or adapter
    bool result = planner(planning_scene, req, res); 

    // do anything after calling the nested planner or adapters here
    if (result && res.trajectory_)
    {
      ROS_DEBUG("Running '%s'", getDescription().c_str()); // inform the user 
      if (!smoothing_filter_->applyFilter(*res.trajectory_)) // do the work of smoothing using a smoothing object
        ROS_WARN("Smoothing filter of the solution path failed.");
    }

    return result;
  }
  

private:
  ros::NodeHandle nh_;
  industrial_trajectory_filters::SmoothingTrajectoryFilter *smoothing_filter_;
  std::string filter_base_name_;
  std::vector<double> filter_coef_;

};

const std::string  AddSmoothingFilter::FILTER_PARAMETER_NAME_ = "/move_group/smoothing_filter_name";

}

CLASS_LOADER_REGISTER_CLASS(industrial_trajectory_filters::AddSmoothingFilter,
                            planning_request_adapter::PlanningRequestAdapter);
