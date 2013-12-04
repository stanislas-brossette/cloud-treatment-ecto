#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

namespace cloud_treatment
{
	struct StepSegmentationCell
	{
		static void declare_params(tendrils& params)
		{
      int number_steps = 7;
      int number_visible_steps = 4;
			double threshold = 0.01;
			double z_step_1 = 0;
			double z_step_2 = 0;
			double z_step_3 = 0;
			double z_step_4 = 0;
			double z_step_5 = 0;
			double z_step_6 = 0;
			double z_step_7 = 0;
      
			params.declare<int> ("number_steps",	"Number of steps", number_steps);
			params.declare<int> ("number_visible_steps",	"Number of visible steps", number_visible_steps);
			params.declare<double> ("threshold",
					"threshold for the segmentation of the ladder steps.", threshold);
			params.declare<double> ("z_step_1",	"Altitude of the 1st step.", z_step_1);
			params.declare<double> ("z_step_2",	"Altitude of the 2nd step.", z_step_2);
			params.declare<double> ("z_step_3",	"Altitude of the 3rd step.", z_step_3);
			params.declare<double> ("z_step_4",	"Altitude of the 4th step.", z_step_4);
			params.declare<double> ("z_step_5",	"Altitude of the 5th step.", z_step_5);
			params.declare<double> ("z_step_6",	"Altitude of the 6th step.", z_step_6);
			params.declare<double> ("z_step_7",	"Altitude of the 7th step.", z_step_7);
		}

		static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
		{
			outputs.declare<ecto::pcl::Clusters> ("clusters", "Extracted clusters");
		}

		void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
		{
      number_steps_ = params["number_steps"];
      number_visible_steps_ = params["number_visible_steps"];
			threshold_ = params["threshold"];
			z_step_1_ = params["z_step_1"];
			z_step_2_ = params["z_step_2"];
			z_step_3_ = params["z_step_3"];
			z_step_4_ = params["z_step_4"];
			z_step_5_ = params["z_step_5"];
			z_step_6_ = params["z_step_6"];
			z_step_7_ = params["z_step_7"];

			clusters_ = outputs["clusters"];
      clusters_->resize(static_cast<std::size_t>(*number_steps_));
		}

		template <typename Point>
		int process(const tendrils& inputs, const tendrils& outputs,
					boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
		{
      clusters_->clear(); 
      clusters_->resize(static_cast<std::size_t>(*number_steps_));
      std::cout << "Processing Step Seg" << std::endl;
      std::vector<double> z_steps;
      z_steps.push_back(*z_step_1_ );
      z_steps.push_back(*z_step_2_ );
      z_steps.push_back(*z_step_3_ );
      z_steps.push_back(*z_step_4_ );
      z_steps.push_back(*z_step_5_ );
      z_steps.push_back(*z_step_6_ );
      z_steps.push_back(*z_step_7_ );
      for (std::size_t i = 0; i < input->size(); ++i)
      {
        for (std::size_t s = 0; s < z_steps.size(); s++)
        {
          if (input->points[i].z < z_steps[s] + *threshold_ &&
              input->points[i].z > z_steps[s] - *threshold_)
          {
            clusters_->at(s).indices.push_back(static_cast<int>(i));
            break;
          }
        }
      }
			return ecto::OK;
		}

    ecto::spore<int> number_steps_;
    ecto::spore<int> number_visible_steps_;
		ecto::spore<double> threshold_;
		ecto::spore<double> z_step_1_;
		ecto::spore<double> z_step_2_;
		ecto::spore<double> z_step_3_;
		ecto::spore<double> z_step_4_;
		ecto::spore<double> z_step_5_;
		ecto::spore<double> z_step_6_;
		ecto::spore<double> z_step_7_;
		ecto::spore<ecto::pcl::Clusters> clusters_;
	};
}



ECTO_CELL(cloud_treatment, ecto::pcl::PclCell<cloud_treatment::StepSegmentationCell>,
		  "StepSegmentationCell", "Step Segmentation Cell");
