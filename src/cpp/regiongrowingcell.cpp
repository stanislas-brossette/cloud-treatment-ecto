#include <boost/make_shared.hpp>

#include <pcl/segmentation/region_growing.h>
#include <pcl/io/pcd_io.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

namespace cloud_treatment
{
	struct RegionGrowingCell
	{
		static void declare_params(ecto::tendrils& params)
		{
			pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> default_;

			int min_cluster_size = default_.getMinClusterSize();
			int max_cluster_size = default_.getMaxClusterSize();
			bool smooth_mode_flag = default_.getSmoothModeFlag();
			bool curvature_test_flag = default_.getCurvatureTestFlag();
			bool residual_test_flag = default_.getResidualTestFlag();
			float smoothness_threshold = default_.getSmoothnessThreshold();
			float residual_threshold = default_.getResidualThreshold();
			float curvature_threshold = default_.getCurvatureThreshold();
			int number_of_neighbours = default_.getNumberOfNeighbours();


			params.declare<int> ("min_cluster_size",
								 "Stores the minimum number of points that a cluster needs to contain in order to be considered valid",
								 min_cluster_size);
			params.declare<int> ("max_cluster_size",
								 "Stores the maximum number of points that a cluster needs to contain in order to be considered valid",
								 max_cluster_size);
			params.declare<bool> ("smooth_mode_flag",
								 "Flag that signalizes if the smoothness constraint will be used",
								  smooth_mode_flag);
			params.declare<bool> ("curvature_test_flag",
								 "If set to true then curvature test will be done during segmentation",
								  curvature_test_flag);
			params.declare<bool> ("residual_test_flag",
								 "If set to true then residual test will be done during segmentation",
								  residual_test_flag);
			params.declare<float> ("smoothness_threshold",
								 "Threshold used for testing the smoothness between points",
								  smoothness_threshold);
			params.declare<float> ("residual_threshold",
								 "Threshold used in residual test",
								  residual_threshold);
			params.declare<float> ("curvature_threshold",
								 "Threshold used in residual test",
								  curvature_threshold);
			params.declare<int> ("number_of_neighbours",
								 "Number of neighbours to find",
								  number_of_neighbours);
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			outputs.declare<ecto::pcl::Clusters > ("clusters",
												   "Clusters found by the algorithm");
		}

		void configure(const tendrils& params, const tendrils& inputs,
					   const tendrils& outputs)
		{
			temp_normals_ = boost::make_shared<pcl::PointCloud< pcl::Normal> > ();
			min_cluster_size_ = params["min_cluster_size"];
			max_cluster_size_ = params["max_cluster_size"];
			smooth_mode_flag_ = params["smooth_mode_flag"];
			curvature_test_flag_ = params["curvature_test_flag"];
			residual_test_flag_ = params["residual_test_flag"];
			smoothness_threshold_ = params["smoothness_threshold"];
			residual_threshold_ = params["residual_threshold"];
			curvature_threshold_ = params["curvature_threshold"];
			number_of_neighbours_ = params["number_of_neighbours"];

			clusters_ = outputs["clusters"];
			clusters_->resize(0);
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const pcl::PointCloud<Point> >& input,
					boost::shared_ptr<const pcl::PointCloud<pcl::Normal > >& normals)
		{
			pcl::RegionGrowing<Point, pcl::Normal> reg;

			//Set the algorithm's parameters
			reg.setMinClusterSize (*min_cluster_size_);
			reg.setMaxClusterSize (*max_cluster_size_);
			reg.setSmoothModeFlag (*smooth_mode_flag_);
			reg.setCurvatureTestFlag (*curvature_test_flag_);
			reg.setResidualTestFlag (*residual_test_flag_);
			reg.setSmoothnessThreshold (static_cast<float>(
											*smoothness_threshold_ / 180.0 * M_PI));
			reg.setResidualThreshold (*residual_threshold_);
			reg.setCurvatureThreshold (*curvature_threshold_);
			reg.setNumberOfNeighbours (*number_of_neighbours_);

			////TO FIX
			//A copy is necessary to match the signature of
			//the pcl::RegionGrowing::setInputNormals method
			pcl::copyPointCloud(*normals, *temp_normals_);
			////

			reg.setInputNormals(temp_normals_);
			reg.setInputCloud(input);

			reg.extract (*clusters_);
			std::cout << "clusters_.size() = " << clusters_->size() << std::endl;

			return ecto::OK;
		}
		// Store params/inputs/outputs in spores
		ecto::spore<int> min_cluster_size_;
		ecto::spore<int> max_cluster_size_;
		ecto::spore<bool> smooth_mode_flag_;
		ecto::spore<bool> curvature_test_flag_;
		ecto::spore<bool> residual_test_flag_;
		ecto::spore<float> smoothness_threshold_;
		ecto::spore<float> residual_threshold_;
		ecto::spore<float> curvature_threshold_;
		ecto::spore<int> number_of_neighbours_;
		ecto::spore<ecto::pcl::Clusters> clusters_;

		boost::shared_ptr<pcl::PointCloud< pcl::Normal> > temp_normals_;
	};
}



ECTO_CELL(cloud_treatment, ecto::pcl::PclCellWithNormals<cloud_treatment::RegionGrowingCell>,
		  "RegionGrowingCell",
		  "Region growing algorithm");

