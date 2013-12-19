#include <boost/make_shared.hpp>
#include <Eigen/Core>

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

namespace cloud_treatment
{
	struct StepCenteringCell
	{
		static void declare_params(tendrils& params)
		{
		}

		static void declare_io(const tendrils& params, tendrils& inputs, 
                           tendrils& outputs)
		{
			inputs.declare<ecto::pcl::Clusters> ("clusters", "Extracted clusters");
      inputs.declare<std::vector< Eigen::Matrix3f > > ("frames", 
                                   "List of frames of the clusters");
      outputs.declare<std::vector< Eigen::Vector4f > > ("centers", 
                                   "Centers of the clusters");
		}

		void configure(const tendrils& params, 
                   const tendrils& inputs, const tendrils& outputs)
		{
      clusters_ = inputs["clusters"];
      frames_ = inputs["frames"];
      centers_ = outputs["centers"];
      centers_->resize(static_cast<std::size_t>(clusters_->size()));
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					      boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
		{
      centers_->clear();
      centers_->resize(static_cast<std::size_t>(clusters_->size()));
      ::pcl::ExtractIndices<Point> filter;
      filter.setInputCloud(input);
      Eigen::Vector3f firstAxis;
      Eigen::Vector3f secondAxis;
      Eigen::Matrix3f frame;

      for(std::size_t i = 0; i < clusters_->size(); ++i)
      {
        if((*clusters_)[i].indices.size() == 0)
          continue;

        frame = (*frames_)[i];
        firstAxis = frame.col(0);
        secondAxis = frame.col(1);
        boost::shared_ptr< ::pcl::PointCloud<Point> > cloud;
        cloud = boost::make_shared< ::pcl::PointCloud<Point> > ();
        // extract indices into a cloud
        filter.setIndices( ::pcl::PointIndicesPtr(
        new ::pcl::PointIndices ((*clusters_)[i])) );
        filter.filter(*cloud);
        Eigen::Vector3f point(cloud->points[0].x, 
                              cloud->points[0].y, cloud->points[0].z);
        Eigen::Vector3f minPoint(cloud->points[0].x, 
                                 cloud->points[0].y, cloud->points[0].z);
        double firstMin = point.dot(firstAxis);
        double firstMax = point.dot(firstAxis);
        double secondMin = point.dot(secondAxis);
        double secondMax = point.dot(secondAxis);
        for(std::size_t p = 0; p < cloud->points.size(); ++p)
        {
          point = Eigen::Vector3f(cloud->points[p].x, 
                                  cloud->points[p].y, cloud->points[p].z);
          if(point.dot(firstAxis) > firstMax)
          {
            firstMax = point.dot(firstAxis);
          }
          if(point.dot(firstAxis) < firstMin)
          {
            firstMin = point.dot(firstAxis);
            minPoint.x() = point.x();
          }
          if(point.dot(secondAxis) > secondMax)
          {
            secondMax = point.dot(secondAxis);
          }
          if(point.dot(secondAxis) < secondMin)
          {
            secondMin = point.dot(secondAxis);
            minPoint.y() = point.y();
          }
        }  
        Eigen::Vector3f Center = minPoint + 
                             ((firstMax-firstMin)/2)*firstAxis + 
                             ((secondMax-secondMin)/2)*secondAxis;
        centers_->at(i) = Eigen::Vector4f(Center.x(), 
                                          Center.y(), 
                                          Center.z(), 
                                          0.0); 
      }
      
			return ecto::OK;
		}

    ecto::spore< ecto::pcl::Clusters> clusters_;
    ecto::spore< std::vector< Eigen::Matrix3f > > frames_; 
    ecto::spore< std::vector< Eigen::Vector4f > > centers_;
	};
}



ECTO_CELL(cloud_treatment, 
          ecto::pcl::PclCell<cloud_treatment::StepCenteringCell>,
		      "StepCenteringCell", "Computes the center of the steps");
