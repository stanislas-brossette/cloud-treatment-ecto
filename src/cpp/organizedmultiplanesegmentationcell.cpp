#include <boost/make_shared.hpp>
#include <boost/variant.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

#include "typedefs.h"

namespace cloud_treatment
{
	struct OrganizedMultiPlaneSegmentationCell
	{
		static void declare_params(ecto::tendrils& params)
		{
			pcl::OrganizedMultiPlaneSegmentation<
					pcl::PointXYZ, pcl::Normal, pcl::Label> default_;
			unsigned plane_min_inliers = default_.getMinInliers();
			double plane_angular_threshold = default_.getAngularThreshold();
			double plane_distance_threshold = default_.getDistanceThreshold();
			double maximum_curvature = default_.getMaximumCurvature();
			bool use_planar_refinement = false;

			////TODO////
			//bool 	project_points_
			//Whether or not points should be projected to the plane,
			//or left in the original 3D space.
			//PlaneComparatorPtr 	compare_
			//A comparator for comparing neighboring pixels' plane equations.
			//PlaneRefinementComparatorPtr 	refinement_compare_
			//A comparator for use on the refinement step.
			////////////

			params.declare<unsigned> ("plane_min_inliers",
								 "The minimum number of inliers required for each plane",
								 plane_min_inliers);
			params.declare<double> ("plane_angular_threshold",
								 "The tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane",
								 plane_angular_threshold);
			params.declare<double> ("plane_distance_threshold",
								 "The tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane",
								 plane_distance_threshold);
			params.declare<double> ("maximum_curvature",
								 "The tolerance for maximum curvature after fitting a plane",
								 maximum_curvature);
			params.declare<bool> ("use_planar_refinement",
								 "use planar refinement",
								 use_planar_refinement);
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			outputs.declare< planarRegions_t > ("regions", "Segmented plane regions");
			outputs.declare< std::vector<pcl::ModelCoefficients> > ("model_coefficients", "model_coefficients of the segmented plane regions, only valid if use_planar_refinement=TRUE");
			outputs.declare< std::vector<pcl::PointIndices> > ("inlier_indices", "inlier_indices of the segmented plane regions, only valid if use_planar_refinement=TRUE");
			outputs.declare< pcl::PointCloud<pcl::Label>::Ptr > ("labels", "labels of the segmented plane regions, only valid if use_planar_refinement=TRUE");
			outputs.declare< std::vector<pcl::PointIndices> > ("label_indices", "label_indices of the segmented plane regions, only valid if use_planar_refinement=TRUE");
			outputs.declare< std::vector<pcl::PointIndices> > ("boundary_indices", "model_coefficients of the segmented plane regions, only valid if use_planar_refinement=TRUE");
		}

		void configure(const tendrils& params, const tendrils& inputs,
					   const tendrils& outputs)
		{
			plane_min_inliers_ = params["plane_min_inliers"];
			plane_angular_threshold_ = params["plane_angular_threshold"];
			plane_distance_threshold_ = params["plane_distance_threshold"];
			maximum_curvature_ = params["maximum_curvature"];
			use_planar_refinement_ = params["use_planar_refinement"];

			regions_ = outputs["regions"];
			model_coefficients_ = outputs["model_coefficients"];
			inlier_indices_ = outputs["inlier_indices"];
			labels_ = outputs["labels"];
			label_indices_ = outputs["label_indices"];
			boundary_indices_ = outputs["boundary_indices"];
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const pcl::PointCloud<Point> >& input,
					boost::shared_ptr<const pcl::PointCloud<pcl::Normal > >& normals)
		{
			pcl::OrganizedMultiPlaneSegmentation<Point, pcl::Normal, pcl::Label> mps;

			//Set the algorithm's parameters
			mps.setMinInliers (*plane_min_inliers_);
			mps.setAngularThreshold (pcl::deg2rad (*plane_angular_threshold_)); //3 degrees
			mps.setDistanceThreshold (*plane_distance_threshold_); //2cm
			mps.setMaximumCurvature (*maximum_curvature_);

			std::vector<pcl::PlanarRegion<Point>,
					Eigen::aligned_allocator<pcl::PlanarRegion<Point> > > regions;
			std::vector<pcl::ModelCoefficients> model_coefficients;
			std::vector<pcl::PointIndices> inlier_indices;
			pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
			std::vector<pcl::PointIndices> label_indices;
			std::vector<pcl::PointIndices> boundary_indices;
			mps.setInputNormals (normals);
			mps.setInputCloud (input);

			if (*use_planar_refinement_)
			{
				std::cout << "Using planar refinement\n";
				mps.segmentAndRefine (regions, model_coefficients, inlier_indices,
									  labels, label_indices, boundary_indices);
			}
			else
			{
				std::cout << "Not using planar refinement\n";
				mps.segment (regions);//, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
			}

			*regions_ = regions;
			*model_coefficients_ = model_coefficients;
			*inlier_indices_ = inlier_indices;
			*labels_ = labels;
			*label_indices_ = label_indices;
			*boundary_indices_ = boundary_indices;

			return ecto::OK;
		}
		// Store params/inputs/outputs in spores
		ecto::spore<unsigned> plane_min_inliers_;
		ecto::spore<double> plane_angular_threshold_;
		ecto::spore<double> plane_distance_threshold_;
		ecto::spore<double> maximum_curvature_;
		ecto::spore<bool> use_planar_refinement_;

		ecto::spore<planarRegions_t> regions_;
		ecto::spore<std::vector<pcl::ModelCoefficients> > model_coefficients_;
		ecto::spore<std::vector<pcl::PointIndices> > inlier_indices_;
		ecto::spore<pcl::PointCloud<pcl::Label>::Ptr > labels_;
		ecto::spore<std::vector<pcl::PointIndices> > label_indices_;
		ecto::spore<std::vector<pcl::PointIndices> > boundary_indices_;
	};
}



ECTO_CELL(cloud_treatment, ecto::pcl::PclCellWithNormals<cloud_treatment::OrganizedMultiPlaneSegmentationCell>,
		  "OrganizedMultiPlaneSegmentationCell",
		  "Organized multi plane segmentation");
