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

def kinect_highres(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.SXGA_RES,
		   depth_resolution=ResolutionMode.VGA_RES,
		   rgb_fps=15, depth_fps=30,
		   device_number=device_n,
		   registration=True,
		   synchronize=False,
		   device=Device.KINECT
		   )

def kinect_vga(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.VGA_RES,
		   depth_resolution=ResolutionMode.VGA_RES,
		   rgb_fps=30, depth_fps=30,
		   device_number=device_n,
		   registration=True,
		   synchronize=False,
		   device=Device.KINECT
		   )

def xtion_highres(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.SXGA_RES,
		   depth_resolution=ResolutionMode.VGA_RES,
		   rgb_fps=30, depth_fps=30,
		   device_number=device_n,
		   registration=True,
		   synchronize=True,
		   device=Device.ASUS_XTION_PRO_LIVE
		   )

def xtion_vga(device_n):
    from ecto_openni import Capture, ResolutionMode, Device
    return Capture('ni device', rgb_resolution=ResolutionMode.VGA_RES,
		   depth_resolution=ResolutionMode.VGA_RES,
		   rgb_fps=30, depth_fps=30,
		   device_number=device_n,
		   registration=True,
		   synchronize=True,
		   device=Device.ASUS_XTION_PRO_LIVE
		   )


device = 0
#capture = xtion_highres(device)
#capture = xtion_vga(device)
capture = kinect_vga(device)
#capture = kinect_highres(device)

#capture.process()

cloud_generator = ecto_pcl.NiConverter('cloud_generator')

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
					maximum_curvature=0.01,
					plane_min_inliers=1000,
					plane_angular_threshold=3.0,
					plane_distance_threshold=0.02,
					use_planar_refinement=1
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

graph = [reader[:] >> passthrough['input'],
	passthrough[:] >> normals[:],
	passthrough[:] >> multiplanesegmentation ['input'],
	normals["normals"] >> multiplanesegmentation["normals"],
	reader[:] >> viewer['input'],
	multiplanesegmentation['regions'] >> viewer['regions']
	]

plasm = ecto.Plasm()
plasm.connect(graph)

run_plasm(options, plasm, locals=vars())
#plasm_ecto.execute()
print "plasm_ecto executed"
