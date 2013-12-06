#include <boost/make_shared.hpp>
#include <Eigen/Core>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include "typedefs.h" 

namespace cloud_treatment
{
	struct PrincipalComponentExtractionCell
	{
		static void declare_params(tendrils& params)
		{
      double length_rectangles = 0;
      double width_rectangles = 0;
      params.declare<double>("length_rectangles", "length of the rectangles", length_rectangles);
      params.declare<double>("width_rectangles", "width of the rectangles", width_rectangles);
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			inputs.declare<ecto::pcl::Clusters> ("clusters", "Extracted clusters");
      outputs.declare<std::vector< Eigen::Matrix3f > > ("eigenvectors", 
                                                       "List of eigen vectors of the clusters");
      outputs.declare<std::vector< Eigen::Vector4f > > ("centroids", "Centroids of the clusters");
      outputs.declare<std::vector< std::vector<Eigen::Vector3f> > > ("rectangles", 
                                                        "Rectangles representing steps in clusters");
		}

		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
      length_rectangles_ = params["length_rectangles"];
      width_rectangles_ = params["width_rectangles"];
      clusters_ = inputs["clusters"];
      eigenVectors_ = outputs["eigenvectors"];
      eigenVectors_->resize(static_cast<std::size_t>(clusters_->size()));
      centroids_ = outputs["centroids"];
      centroids_->resize(static_cast<std::size_t>(clusters_->size()));
      rectangles_ = outputs["rectangles"];
      rectangles_->resize(static_cast<std::size_t>(clusters_->size()));
      for(std::size_t i = 0; i < clusters_->size(); ++i)
      {
        rectangles_->at(i).resize(4);
      }
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
		{
      eigenVectors_->clear();
      centroids_->clear();
      ::pcl::ExtractIndices<Point> filter;
      filter.setInputCloud(input);

      rectangles_->resize(static_cast<std::size_t>(clusters_->size()));
      for(std::size_t i = 0; i < clusters_->size(); ++i)
      {
        rectangles_->at(i).resize(4);
      }

      for(std::size_t i = 0; i < clusters_->size(); ++i)
      {
        boost::shared_ptr< ::pcl::PointCloud<Point> > cloud;
        cloud = boost::make_shared< ::pcl::PointCloud<Point> > ();
        // extract indices into a cloud
        filter.setIndices( ::pcl::PointIndicesPtr(
        new ::pcl::PointIndices ((*clusters_)[i])) );
        filter.filter(*cloud);

        // extract the eigen vectors
        ::pcl::PointCloud< ::pcl::PointXYZ> proj;
        pcl::PCA <Point > pca;
        pca.setInputCloud(cloud);
        eigenVectors_->push_back(pca.getEigenVectors());
        centroids_->push_back(pca.getMean());

        //generate the rectangles
        Eigen::Vector3f center = Eigen::Vector3f(pca.getMean().x(),
                                                 pca.getMean().y(),
                                                 pca.getMean().z());
        Eigen::Vector3f longAxis = Eigen::Vector3f(pca.getEigenVectors()(0,0),
                                                   pca.getEigenVectors()(1,0),
                                                   pca.getEigenVectors()(2,0));
        Eigen::Vector3f shortAxis = Eigen::Vector3f(pca.getEigenVectors()(0,1),
                                                    pca.getEigenVectors()(1,1),
                                                    pca.getEigenVectors()(2,1));
        longAxis.normalize();
        longAxis = (*length_rectangles_)*longAxis;        
        shortAxis.normalize();
        shortAxis = (*width_rectangles_)*shortAxis;        
        
        rectangles_->at(i)[0] = center - longAxis/2 - shortAxis/2;
        rectangles_->at(i)[1] = center + longAxis/2 - shortAxis/2;
        rectangles_->at(i)[2] = center + longAxis/2 + shortAxis/2;
        rectangles_->at(i)[3] = center - longAxis/2 + shortAxis/2;

      }
      
			return ecto::OK;
		}

    ecto::spore< double > length_rectangles_;
    ecto::spore< double > width_rectangles_;
    ecto::spore< std::vector< Eigen::Vector4f > > centroids_;
    ecto::spore< std::vector< Eigen::Matrix3f > > eigenVectors_; 
    ecto::spore< std::vector< std::vector<Eigen::Vector3f> > > rectangles_;
    ecto::spore< ecto::pcl::Clusters> clusters_;
	};
}



ECTO_CELL(cloud_treatment, ecto::pcl::PclCell<cloud_treatment::PrincipalComponentExtractionCell>,
		  "PrincipalComponentExtractionCell", "Cell that extracts the principal components of clouds");
