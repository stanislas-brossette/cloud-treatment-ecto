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

voxelGrid = cloud_treatment.VoxelGridCell(
					"voxelGrid",
					leafSize=0.02,
					leafX=0.02,
					leafY=0.02,
					leafZ=0.02)

normalSegmentation = cloud_treatment.NormalSegmentationCell(
		    "normalSeg",
		    angle_threshold=0.9,
		    curvature_threshold=0
                    )

colorize = ecto_pcl.ColorizeClusters("colorize")

#graph = [capture[:] >> cloud_generator[:],
	#cloud_generator[:] >> passthrough['input'],
	#passthrough[:] >> normals[:],
	#passthrough[:] >> multiplanesegmentation ['input'],
	#normals["normals"] >> multiplanesegmentation["normals"],
	#cloud_generator[:] >> viewer['input'],
	#multiplanesegmentation['regions'] >> viewer['regions']
	#]

graph = [reader[:] >> voxelGrid['input'],
	voxelGrid[:] >> normalSegmentation['input'],
	normalSegmentation[:] >> viewer['input'],
	#voxelGrid[:] >> viewer['input'],
	]

plasm = ecto.Plasm()
plasm.connect(graph)

run_plasm(options, plasm, locals=vars())
#plasm_ecto.execute()
print "plasm_ecto executed"
