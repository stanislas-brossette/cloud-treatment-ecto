#include <boost/make_shared.hpp>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

#include <ecto/ecto.hpp>

using ecto::tendrils;
namespace cloud_treatment
{
	struct EditHeaderCell
	{

		static void declare_params(tendrils& params)
		{
      params.declare<std::string> ("frame_id", "Frame_ID");
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
      inputs.declare<boost::shared_ptr<sensor_msgs::PointCloud2 const> > (
                                                           "pointcloud2msg", 
                                                           "pointCloud2 message");
      outputs.declare<boost::shared_ptr<sensor_msgs::PointCloud2 const> > (
                                                           "pointcloud2msg", 
                                                           "pointCloud2 message");
		}

		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
      input_ = inputs["pointcloud2msg"];
      output_ = outputs["pointcloud2msg"];
      frame_id_ = params["frame_id"];
		}

		int process(const tendrils& inputs, const tendrils& outputs)
		{
      sensor_msgs::PointCloud2 msgBuffer = *(*input_);
      msgBuffer.header.frame_id = *frame_id_;
      *output_ = boost::make_shared<sensor_msgs::PointCloud2 const>(msgBuffer);
			return ecto::OK;
		}
    ecto::spore<boost::shared_ptr<sensor_msgs::PointCloud2 const> > input_;
    ecto::spore<boost::shared_ptr<sensor_msgs::PointCloud2 const> > output_;
    ecto::spore<std::string> frame_id_;
	};
}

ECTO_CELL(cloud_treatment, cloud_treatment::EditHeaderCell,
		  "EditHeaderCell", "Edits the frame_id of the header of of pointcloud2 msg");

