#!/usr/bin/env python

import ecto, ecto_pcl
from ecto.opts import scheduler_options, run_plasm
import sys
import time
import os
import argparse

import cloud_treatment_ecto.cloud_treatment as cloud_treatment

parser = argparse.ArgumentParser(description='Treatment that segments the'
                'ladder steps from the rest on the scene')
scheduler_options(parser)

parser.add_argument('-p', '--pcdfile'
			, default='ladder_test_5.pcd', help='The pcdfile to input')

options = parser.parse_args()
pcdfilename = options.pcdfile
plasm = ecto.Plasm()

reader = cloud_treatment.PCDReaderCell(	"Reader_ecto",
					filename=pcdfilename)

passthrough3d = cloud_treatment.PassThrough3DCell(
					"passthrough3D",
          x_min=-1.0,
          x_max=2.0,
          y_min=-1.9,
          y_max=-0.5,
          z_min=-1.0,
          z_max=5.0)

stepsegmenter = cloud_treatment.StepSegmentationCell("Step_Seg",
                z_step_1=0.0,
                z_step_2=0.30,
                z_step_3=0.61,
                z_step_4=0.91,
                z_step_5=1.21,
                z_step_6=1.52,
                z_step_7=1.83,
                positive_threshold=0.01,
                negative_threshold=0.0,
                optim_precision=0.01,
                optim_number_of_iter=30
                )

principalcomponent = cloud_treatment.PrincipalComponentExtractionCell(
                "Principal_component",
                length_rectangles = 0.8128,
                width_rectangles = 0.127
                )


colorize = ecto_pcl.ColorizeClusters("colorize")

viewer = cloud_treatment.CloudViewerCell("Viewer_ecto",
					window_name="PCD Viewer")

graph = [reader["output"] >> passthrough3d["input"],
         passthrough3d["output"] >> stepsegmenter["input"],
         principalcomponent["centroids"] >> viewer["VIPoints"],
         principalcomponent["rectangles"] >> viewer["rectangles"],
         stepsegmenter["clusters"] >> colorize["clusters"],
         passthrough3d["output"] >> colorize["input"], 
         stepsegmenter["clusters"] >> principalcomponent["clusters"],
         passthrough3d["output"] >> principalcomponent["input"], 
         colorize[:] >> viewer["input"]
	]
#graph = [reader["output"] >> passthrough3d["input"],
#         passthrough3d["output"] >> viewer["input"]
#	]

plasm = ecto.Plasm()
plasm.connect(graph)

run_plasm(options, plasm, locals=vars())
#plasm.execute()
print "plasm_ecto executed"

