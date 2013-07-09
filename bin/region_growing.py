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
					window_name="PCD Viewer")\

passthrough = cloud_treatment.PassThroughCell(
					"passthrough",
					filter_field_name='z',
					filter_limit_max=4)

voxel_grid = ecto_pcl.VoxelGrid(
					"voxel_grid",
					leaf_size=0.01)

normals_ecto = ecto_pcl.NormalEstimation(
					"normals",
					k_search=0,
					radius_search=0.02)

graph = [reader['output'] >> passthrough['input'],
	passthrough['output'] >> voxel_grid['input']
	]

region_growing = cloud_treatment.RegionGrowingCell(
					"RegionGrowingCell",
					min_cluster_size=500,
					max_cluster_size=70000,
					number_of_neighbours=30,
					smoothness_threshold=6,
					curvature_threshold=2)

colorize = ecto_pcl.ColorizeClusters(
					"colorize")

graph += [
	voxel_grid[:] >> normals_ecto[:],
	voxel_grid[:] >> region_growing["input"],
	normals_ecto[:] >> region_growing["normals"],
	region_growing[:] >> colorize["clusters"],
	voxel_grid[:] >> colorize["input"],
	colorize[:] >> viewer["input"]
	]

plasm = ecto.Plasm()
plasm.connect(graph)

run_plasm(options, plasm, locals=vars())
#plasm_ecto.execute()
print "plasm_ecto executed"
