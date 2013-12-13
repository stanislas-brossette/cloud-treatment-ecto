#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>

#include <ecto/ecto.hpp>

#include <rosbuild_msgs/RecalibrateEnv.h>

using ecto::tendrils;
namespace cloud_treatment
{
	struct RecalibrateMsgCell
	{

		static void declare_params(tendrils& params)
		{
      std::size_t choice_index;
			params.declare<std::size_t> ("choice_index", "Item of the list to publish", choice_index);
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
      inputs.declare<std::vector<Eigen::Matrix3f > > ("frames", "list of frames");
      inputs.declare<std::vector<Eigen::Vector4f > > ("origins", "list of origins");
      inputs.declare<std_msgs::Header > ("header", "Header");
      outputs.declare<boost::shared_ptr<geometry_msgs::PoseStamped const> > (
                                                                      "pose_stamped_msg", 
                                                                      "pose message to publish");
      outputs.declare<boost::shared_ptr<std_msgs::String const> > ("surf_name",
                                                                  "Name of the surface");
		}

		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
      header_ = inputs["header"];
      origins_ = inputs["origins"];
      frames_ = inputs["frames"];
      pose_stamped_msg_ = outputs["pose_stamped_msg"];
      surf_name_ = outputs["surf_name"];
      choice_index_ = params["choice_index"];
		}

    Eigen::Matrix3f reOrganizeFrame(const Eigen::Matrix3f& frame)
    {
      Eigen::Matrix3f output;

      if(frame.col(0).dot(Eigen::Vector3f(-1.0, 0.0, 0.0)) < 0)
        output.col(0) = -frame.col(0);
      else
        output.col(0) = frame.col(0);

      if(frame.col(1).dot(Eigen::Vector3f(0.0, -1.0, 0.0)) < 0)
        output.col(1) = -frame.col(1);
      else
        output.col(1) = frame.col(1);

      output.col(2) = output.col(0).cross(output.col(1));

      return output;
    }

		int process(const tendrils& inputs, const tendrils& outputs)
		{
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.header = *header_; 
      poseStamped.pose.position.x = origins_->at(*choice_index_).x();
      poseStamped.pose.position.y = origins_->at(*choice_index_).y();
      poseStamped.pose.position.z = origins_->at(*choice_index_).z();
      Eigen::Matrix3f frame = reOrganizeFrame(frames_->at(*choice_index_));
      Eigen::Quaternionf quaternion (frame);
      poseStamped.pose.orientation.x = quaternion.x(); 
      poseStamped.pose.orientation.y = quaternion.y(); 
      poseStamped.pose.orientation.z = quaternion.z(); 
      poseStamped.pose.orientation.w = quaternion.w(); 
      *pose_stamped_msg_ = 
               boost::make_shared<geometry_msgs::PoseStamped const>(poseStamped); 
      std_msgs::String surf_name;
      if(*choice_index_ == 0)
        surf_name.data = "Floor";
      else
        surf_name.data = "Step" + boost::lexical_cast<std::string>(*choice_index_);
      *surf_name_ = boost::make_shared<std_msgs::String const> (surf_name);

			return ecto::OK;
		}
    ecto::spore< boost::shared_ptr<geometry_msgs::PoseStamped const> > pose_stamped_msg_;
    ecto::spore< std::size_t > choice_index_;
    ecto::spore< std::string > topic_name_;
    ecto::spore< std::vector< Eigen::Vector4f > > origins_;
    ecto::spore< std::vector< Eigen::Matrix3f > > frames_; 
    ecto::spore< std_msgs::Header > header_;  
    ecto::spore<boost::shared_ptr<std_msgs::String const> > surf_name_;
	};
}

ECTO_CELL(cloud_treatment, cloud_treatment::RecalibrateMsgCell,
		  "RecalibrateMsgCell", "Publishes a ros message with origin and frame that can be used for recalibration");

