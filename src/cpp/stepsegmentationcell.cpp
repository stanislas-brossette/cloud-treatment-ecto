#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include "typedefs.h" 

namespace cloud_treatment
{
	struct StepSegmentationCell
	{
		static void declare_params(tendrils& params)
		{
      int number_steps = 7;
      int number_visible_steps = 4;
			double positive_threshold = 0.01;
			double negative_threshold = 0.01;
			double z_step_1 = 0;
			double z_step_2 = 0;
			double z_step_3 = 0;
			double z_step_4 = 0;
			double z_step_5 = 0;
			double z_step_6 = 0;
			double z_step_7 = 0;
			double optim_precision = 0.01;
      double optim_number_of_iter = 10;
      
			params.declare<int> ("number_steps",	"Number of steps", number_steps);
			params.declare<int> ("number_visible_steps",	"Number of visible steps", number_visible_steps);
			params.declare<double> ("positive_threshold",
					"positive threshold for the segmentation of the ladder steps.", positive_threshold);
			params.declare<double> ("negative_threshold",
					"negative threshold for the segmentation of the ladder steps.", negative_threshold);
			params.declare<double> ("z_step_1",	"Altitude of the 1st step", z_step_1);
			params.declare<double> ("z_step_2",	"Altitude of the 2nd step", z_step_2);
			params.declare<double> ("z_step_3",	"Altitude of the 3rd step", z_step_3);
			params.declare<double> ("z_step_4",	"Altitude of the 4th step", z_step_4);
			params.declare<double> ("z_step_5",	"Altitude of the 5th step", z_step_5);
			params.declare<double> ("z_step_6",	"Altitude of the 6th step", z_step_6);
			params.declare<double> ("z_step_7",	"Altitude of the 7th step", z_step_7);
			params.declare<double> ("optim_precision",
                              "Precision of the steps of the optim", optim_precision);
			params.declare<double> ("optim_number_of_iter", 
                              "number of iter of the optim", optim_number_of_iter);
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			outputs.declare<ecto::pcl::Clusters> ("clusters", "Extracted clusters");
      outputs.declare<point_list_t > (
                          "centroids", "Centroids of the extracted clusters");
		}

		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
      number_steps_ = params["number_steps"];
      number_visible_steps_ = params["number_visible_steps"];
			positive_threshold_ = params["positive_threshold"];
			negative_threshold_ = params["negative_threshold"];
			z_step_1_ = params["z_step_1"];
			z_step_2_ = params["z_step_2"];
			z_step_3_ = params["z_step_3"];
			z_step_4_ = params["z_step_4"];
			z_step_5_ = params["z_step_5"];
			z_step_6_ = params["z_step_6"];
			z_step_7_ = params["z_step_7"];
			optim_precision_ = params["optim_precision"];
			optim_number_of_iter_ = params["optim_number_of_iter"];

			clusters_ = outputs["clusters"];
      clusters_->resize(static_cast<std::size_t>(*number_steps_));
			centroids_ = outputs["centroids"];
      centroids_->resize(static_cast<std::size_t>(*number_steps_));
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
		{
      clusters_->clear(); 
      clusters_->resize(static_cast<std::size_t>(*number_steps_));
      centroids_->clear(); 
      centroids_->resize(static_cast<std::size_t>(*number_steps_));
      std::vector<double> z_steps;
      z_steps.push_back(*z_step_1_ );
      z_steps.push_back(*z_step_2_ );
      z_steps.push_back(*z_step_3_ );
      z_steps.push_back(*z_step_4_ );
      z_steps.push_back(*z_step_5_ );
      z_steps.push_back(*z_step_6_ );
      z_steps.push_back(*z_step_7_ );

      optim_heights_.resize(*optim_number_of_iter_);
      for(int i = 0; i < *optim_number_of_iter_; ++i)
      {
        optim_heights_[i] = 
          (i - std::floor(*optim_number_of_iter_/2))*(*optim_precision_);
      }

      std::vector< std::vector<int> > numberInliers(
                         z_steps.size(), std::vector<int>(optim_heights_.size(), 0));   
      
      for (std::size_t i = 0; i < input->size(); ++i)
      {
        for (std::size_t s = 0; s < z_steps.size(); ++s)
        {
          //"optimization" part
          for(std::size_t n = 0; n < optim_heights_.size(); ++n)
          {
            if(input->points[i].z > z_steps[s]+optim_heights_[n] &&
                input->points[i].z < z_steps[s]+optim_heights_[n]+*optim_precision_)
            {
              numberInliers[s][n] = numberInliers[s][n] + 1;
            }
          }

          //segmentation part
          if (input->points[i].z < z_steps[s] + *positive_threshold_ &&
              input->points[i].z > z_steps[s] - *negative_threshold_)
          {
            clusters_->at(s).indices.push_back(static_cast<int>(i));
            centroids_->at(s).x += input->points[i].x;
            centroids_->at(s).y += input->points[i].y;
            centroids_->at(s).z += input->points[i].z;
            break;
          }
        }
      }
      for (std::size_t s = 0; s < z_steps.size(); s++)
      {
        centroids_->at(s).x = centroids_->at(s).x/clusters_->at(s).indices.size();
        centroids_->at(s).y = centroids_->at(s).y/clusters_->at(s).indices.size();
        centroids_->at(s).z = centroids_->at(s).z/clusters_->at(s).indices.size();
      }

      for(std::size_t s = 0; s < numberInliers.size(); ++s)
      {
        std::cout << "Step " << s << ": ";
        std::size_t maxIndex = 0;
        for(std::size_t i = 0; i < numberInliers[s].size(); ++i)
        {
          maxIndex = (numberInliers[s][i] > numberInliers[s][maxIndex]) ? i : maxIndex;
        }
        std::cout << "maxIndex=" << maxIndex << " value=" <<z_steps[s]+optim_heights_[maxIndex]; 
        std::cout << std::endl;
      }

			return ecto::OK;
		}

    std::vector< double> optim_heights_;
    ecto::spore<int> number_steps_;
    ecto::spore<int> number_visible_steps_;
		ecto::spore<double> positive_threshold_;
		ecto::spore<double> negative_threshold_;
		ecto::spore<double> z_step_1_;
		ecto::spore<double> z_step_2_;
		ecto::spore<double> z_step_3_;
		ecto::spore<double> z_step_4_;
		ecto::spore<double> z_step_5_;
		ecto::spore<double> z_step_6_;
		ecto::spore<double> z_step_7_;
		ecto::spore<double> optim_precision_;
		ecto::spore<double> optim_number_of_iter_;
		ecto::spore<ecto::pcl::Clusters> clusters_;
    ecto::spore<point_list_t > centroids_;
	};
}



ECTO_CELL(cloud_treatment, ecto::pcl::PclCell<cloud_treatment::StepSegmentationCell>,
		  "StepSegmentationCell", "Step Segmentation Cell");

