#include <pcl/filters/voxel_grid.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

namespace cloud_treatment
{
	struct VoxelGridCell
	{
		static void declare_params(tendrils& params)
		{
			double leafSize = 0;
			double leafX = 0;
			double leafY = 0;
			double leafZ = 0;


			params.declare<float> ("leafSize",
									"Minimum value for the filter.", leafSize);
			params.declare<float> ("leafX",
									"Minimum value for the filter.", leafX);
			params.declare<float> ("leafY",
									"Maximum value for the filter.", leafY);
			params.declare<float> ("leafZ",
									"Maximum value for the filter.", leafZ);
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			outputs.declare<ecto::pcl::PointCloud> ("output", "Filtered Cloud.");
		}

		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
			leafSize_ = params["leafSize"];
//			if(*leafSize_==0)
//			{
				leafX_ = params["leafX"];
				leafY_ = params["leafY"];
				leafZ_ = params["leafZ"];
//			}
//			else
//			{
//				*leafX_ = *leafSize_;
//				*leafY_ = *leafSize_;
//				*leafZ_ = *leafSize_;
//			}

			output_ = outputs["output"];
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const pcl::PointCloud< Point > >& inputCloud)
		{
			typename pcl::PointCloud<Point>::Ptr cloud_filtered (new typename pcl::PointCloud<Point>);
			pcl::VoxelGrid<Point> sor;
			sor.setInputCloud (inputCloud);
			sor.setLeafSize (*leafX_, *leafY_, *leafZ_);
			sor.filter (*cloud_filtered);

			*output_ = ecto::pcl::xyz_cloud_variant_t(cloud_filtered);
			return ecto::OK;
		}

		ecto::spore<float> leafSize_;
		ecto::spore<float> leafX_;
		ecto::spore<float> leafY_;
		ecto::spore<float> leafZ_;
		ecto::spore<ecto::pcl::PointCloud> output_;
	};
}



ECTO_CELL(cloud_treatment, ecto::pcl::PclCell<cloud_treatment::VoxelGridCell>,
		  "VoxelGridCell", "VoxelGrid filter");


