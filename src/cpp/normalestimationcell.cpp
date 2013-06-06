#include <pcl/features/integral_image_normal.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

namespace cloud_treatment
{
	struct NormalEstimationCell
	{
		static void declare_params(ecto::tendrils& params)
		{
			float max_depth_change_factor = 0.02f;
			float normal_smoothing_size = 20.0f;
			bool depth_dependent_smoothing = false;

			params.declare<float> ("max_depth_change_factor",
								 "the depth change threshold for computing object borders based on depth changes",
								  max_depth_change_factor);
			params.declare<float> ("normal_smoothing_size",
								 "normal_smoothing_size factor which influences the size of the area used to smooth normals",
								  normal_smoothing_size);
			params.declare<bool> ("depth_dependent_smoothing",
								 "Set whether to use depth depending smoothing or not",
								  depth_dependent_smoothing);
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			outputs.declare<ecto::pcl::FeatureCloud> ("normals", "Cloud of normals.");
			outputs.declare<float*> ("distance_map", "Returns a pointer to the distance map which was computed internally");
		}

		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
			max_depth_change_factor_ = params["max_depth_change_factor"];
			normal_smoothing_size_ = params["normal_smoothing_size"];
			depth_dependent_smoothing_ = params["depth_dependent_smoothing"];
			normals_ = outputs["normals"];
			distance_map_ = outputs["distance_map"];
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
		{
			pcl::IntegralImageNormalEstimation<Point, pcl::Normal> ne;

			ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
			ne.setMaxDepthChangeFactor (*max_depth_change_factor_);
			ne.setNormalSmoothingSize (*normal_smoothing_size_);
			ne.setDepthDependentSmoothing (*depth_dependent_smoothing_);

			pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (
						new pcl::PointCloud<pcl::Normal>);
			ne.setInputCloud (input);
			ne.compute (*normal_cloud);

			*distance_map_ = ne.getDistanceMap();
			*normals_ = ecto::pcl::feature_cloud_variant_t(normal_cloud);

			return ecto::OK;
		}

		ecto::spore<float> max_depth_change_factor_;
		ecto::spore<float> normal_smoothing_size_;
		ecto::spore<bool> depth_dependent_smoothing_;
		ecto::spore<ecto::pcl::FeatureCloud> normals_;
		ecto::spore<float*> distance_map_;
	};
}

ECTO_CELL(cloud_treatment, ecto::pcl::PclCell<cloud_treatment::NormalEstimationCell>,
		  "NormalEstimationCell",
		  "Integral image normal estimation");

