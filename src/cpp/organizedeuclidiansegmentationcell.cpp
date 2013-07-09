#include <boost/make_shared.hpp>

#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/io/pcd_io.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

namespace cloud_treatment
{
	struct OrganizedEuclidianSegmentationCell
	{
		static void declare_params(ecto::tendrils& params)
		{
			pcl::EuclideanClusterComparator<
					pcl::PointXYZ, pcl::Normal, pcl::Label> default_;
			unsigned cluster_min_inliers = 1000;
			double angular_threshold = default_.getAngularThreshold();
			double distance_threshold = default_.getDistanceThreshold();
			bool depth_dependent = false;

			params.declare<unsigned> ("cluster_min_inliers",
								 "The minimum number of inliers required for each cluster",
								 cluster_min_inliers);
			params.declare<double> ("angular_threshold",
								 "the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane",
								 angular_threshold);
			params.declare<double> ("distance_threshold",
								 "The tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane",
								 distance_threshold);
			params.declare<bool> ("depth_dependent",
								 "use planar refinement",
								 depth_dependent);
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			inputs.declare<std::vector<pcl::PointIndices> > ("label_indices", "label_indices obtained from multi plane segmentation").required(false);
			inputs.declare<pcl::PointCloud<pcl::Label>::Ptr > ("labels", "labels obtained from multi plane segmentation").required(false);

			outputs.declare<ecto::pcl::Clusters > ("clusters",
												   "Clusters found by the algorithm");
		}

		void configure(const tendrils& params, const tendrils& inputs,
					   const tendrils& outputs)
		{
			cluster_min_inliers_ = params["cluster_min_inliers"];
			angular_threshold_ = params["angular_threshold"];
			distance_threshold_ = params["distance_threshold"];
			depth_dependent_ = params["depth_dependent"];

			clusters_ = outputs["clusters"];
			clusters_->resize(0);
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const pcl::PointCloud<Point> >& input,
					boost::shared_ptr<const pcl::PointCloud<pcl::Normal > >& normals)
		{
			pcl::EuclideanClusterComparator<Point, pcl::Normal, pcl::Label>
					euclidean_cluster_comparator;
			std::vector<pcl::PointIndices> label_indices =
					inputs.get<std::vector<pcl::PointIndices> >("label_indices");
			pcl::PointCloud<pcl::Label>::Ptr labels =
					inputs.get<pcl::PointCloud<pcl::Label>::Ptr>("labels");
			std::vector<bool> plane_labels;
			plane_labels.resize (label_indices.size (), false);
			for (size_t i = 0; i < label_indices.size (); i++)
			{
				if (label_indices[i].indices.size () > 10000)
				{
					plane_labels[i] = true;
				}
			}

			euclidean_cluster_comparator.setInputCloud (input);
			euclidean_cluster_comparator.setLabels (labels);
			euclidean_cluster_comparator.setExcludeLabels (plane_labels);
			euclidean_cluster_comparator.setDistanceThreshold (*distance_threshold_, false);

			pcl::PointCloud<pcl::Label> euclidean_labels;
			std::vector<pcl::PointIndices> euclidean_label_indices;
			pcl::OrganizedConnectedComponentSegmentation<Point,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator&);
			euclidean_segmentation.setInputCloud (input);
			euclidean_segmentation.segment (
						euclidean_labels, euclidean_label_indices);

			*clusters_ = euclidean_label_indices;

			PCL_INFO ("Got %d euclidean clusters!\n", clusters_->size ());


			return ecto::OK;
		}
		// Store params/inputs/outputs in spores
		ecto::spore<unsigned> cluster_min_inliers_;
		ecto::spore<double> angular_threshold_;
		ecto::spore<double> distance_threshold_;
		ecto::spore<bool> depth_dependent_;

		ecto::spore<ecto::pcl::Clusters> clusters_;
	};
}



ECTO_CELL(cloud_treatment, ecto::pcl::PclCellWithNormals<cloud_treatment::OrganizedEuclidianSegmentationCell>,
		  "OrganizedEuclidianSegmentationCell",
		  "Organized Euclidian Segmentation Cell");

