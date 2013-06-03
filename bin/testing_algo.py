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
options = parser.parse_args()

plasm = ecto.Plasm()
pcdfile = 'cloud.pcd'


#if len(sys.argv) > 1:
#    pcdfile = sys.argv[1]

reader = cloud_treatment.PCDReaderCell(	"Reader_ecto",
					filename=pcdfile)
viewer = cloud_treatment.CloudViewerCell("Viewer_ecto",
					window_name="PCD Viewer")
xyz_switch_cell = cloud_treatment.XYZSwitchCell("XYZ_Switch_Cell")
passthrough1 = cloud_treatment.PassThroughCell(	"passthrough1",
					filter_field_name='z',
					filter_limit_max=4)

passthrough2 = cloud_treatment.PassThroughCell(	"passthrough2",
					filter_field_name='z',
					filter_limit_max=4)

voxel_grid = ecto_pcl.VoxelGrid(	"voxel_grid",
					leaf_size=0.005)

#moving_least_quares = ecto_pcl.MovingLeastSquares("MovingLeastSquare")
#normals = ecto_pcl.NormalEstimation("normals", k_search=0, radius_search=0.02)

graph = [reader['output'] >> passthrough1['input'],
	passthrough1['output'] >> passthrough2['input'],
	passthrough2['output'] >> voxel_grid['input'],
	voxel_grid['output'] >> viewer['input']
	]

plasm = ecto.Plasm()
plasm.connect(graph)

run_plasm(options, plasm, locals=vars())
#plasm_ecto.execute()
print "plasm_ecto executed"
