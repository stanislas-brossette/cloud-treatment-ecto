#include <boost/make_shared.hpp>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

#include <ecto/ecto.hpp>

#include "typedefs.h"

using ecto::tendrils;
namespace cloud_treatment
{
	struct ExtractHeaderCell
	{

		static void declare_params(tendrils& params)
		{
		}

		static void declare_io(const tendrils& params, 
                           tendrils& inputs, tendrils& outputs)
		{
      inputs.declare<PointCloud2_Ptr_t> ("pointcloud2msg", 
                                         "pointCloud2 message");
      outputs.declare<std_msgs::Header > ("header", 
                                          "extracted header message");
		}

		void configure(const tendrils& params, 
                   const tendrils& inputs, const tendrils& outputs)
		{
      input_ = inputs["pointcloud2msg"];
      output_ = outputs["header"];
		}

		int process(const tendrils& inputs, const tendrils& outputs)
		{
      *output_ = (*input_)->header; 
			return ecto::OK;
		}
    ecto::spore<PointCloud2_Ptr_t > input_;
    ecto::spore<std_msgs::Header > output_;
	};
}

ECTO_CELL(cloud_treatment, cloud_treatment::ExtractHeaderCell,
		  "ExtractHeaderCell", "Extracts a Header from a poincloud2 message"
     "\nTODO:should be templated" );

