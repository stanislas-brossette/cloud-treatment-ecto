#!/usr/bin/env python

import ecto, ecto_pcl, ecto_openni
from ecto_image_pipeline import conversion
from ecto.opts import run_plasm, scheduler_options
from ecto_opencv import imgproc, calib, highgui
from ecto_openni import OpenNICapture, DEPTH_RGB

def kinect_highres(device_n):
    return ecto_openni.Capture('ni device', rgb_resolution=ecto_openni.ResolutionMode.SXGA_RES,
		   depth_resolution=ecto_openni.ResolutionMode.VGA_RES,
		   rgb_fps=15, depth_fps=30,
		   device_number=device_n,
		   registration=True,
		   synchronize=False,
		   device=ecto_openni.Device.KINECT
		   )

def kinect_vga(device_n):
    return ecto_openni.Capture('ni device', rgb_resolution=ecto_openni.ResolutionMode.VGA_RES,
		   depth_resolution=ecto_openni.ResolutionMode.VGA_RES,
		   rgb_fps=30, depth_fps=30,
		   device_number=device_n,
		   registration=True,
		   synchronize=False,
		   device=ecto_openni.Device.KINECT
		   )

device = 0
#capture = kinect_vga(device)
#capture = kinect_highres(device)
capture = OpenNICapture(stream_mode=DEPTH_RGB, registration=True, sync=False)

mat2pcl = conversion.MatToPointCloudXYZRGBOrganized('mat2pcl')
pcl2ecto = ecto_pcl.PointCloudT2PointCloud('pcl2ecto')

verter = highgui.NiConverter('verter')
fps = highgui.FPSDrawer('fps')

plasm = ecto.Plasm()
graph = [capture[:] >> verter[:],
	verter['image'] >> fps[:],
	fps[:] >> highgui.imshow('image display', name='image')[:],
	verter['depth'] >> highgui.imshow('depth display', name='depth')[:]
	]

cloud_generator = ecto_pcl.NiConverter('cloud_generator')
viewer = ecto_pcl.CloudViewer("viewer", window_name="Clouds!")

graph += [verter['image'] >> mat2pcl['image'],
	verter['depth'] >> mat2pcl['points'],
	mat2pcl[:] >> pcl2ecto[:],
	pcl2ecto[:] >> viewer[:]
	]

plasm = ecto.Plasm()
plasm.connect(graph)

if __name__ == "__main__":
    from ecto.opts import doit
    doit(plasm, description='Execute test kinect.')
