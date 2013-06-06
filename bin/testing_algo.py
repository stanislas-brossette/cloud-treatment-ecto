#!/usr/bin/env python

import ecto, ecto_pcl
from ecto.opts import scheduler_options, run_plasm
import sys
import time
import os
import argparse

import cloud_treatment_ecto.cloud_treatment as cloud_treatment

parser = argparse.ArgumentParser(description='My awesome program thing.')
scheduler_options(parser)

parser.add_argument('-p', '--pcdfile'
			, default='crapahut.pcd', help='The pcdfile to input')

options = parser.parse_args()
pcdfilename = options.pcdfile
plasm = ecto.Plasm()

reader = cloud_treatment.PCDReaderCell(
					"Reader_ecto",
					filename=pcdfilename)

viewer = cloud_treatment.CloudViewerCell(
					"Viewer_ecto",
					window_name="PCD Viewer")

passthrough = cloud_treatment.PassThroughCell(
					"passthrough",
					filter_field_name='z',
					filter_limit_max=4)

normals = cloud_treatment.NormalEstimationCell(
					"normals")

multiplanesegmentation = cloud_treatment.OrganizedMultiPlaneSegmentationCell(
					"multiplanesegmentation",
					plane_min_inliers=1000,
					plane_angular_threshold=3.0,
					plane_distance_threshold=0.02,
					use_planar_refinement=1
					)

colorize = ecto_pcl.ColorizeClusters("colorize")

graph = [reader['output'] >> passthrough['input'],
	passthrough[:] >> normals[:],
	passthrough[:] >> multiplanesegmentation ['input'],
	normals["normals"] >> multiplanesegmentation["normals"]
	]

plasm = ecto.Plasm()
plasm.connect(graph)

run_plasm(options, plasm, locals=vars())
#plasm_ecto.execute()
print "plasm_ecto executed"
