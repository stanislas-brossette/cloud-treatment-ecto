#!/usr/bin/env python

import ecto, ecto_pcl, ecto_openni
from ecto_image_pipeline import conversion
from ecto.opts import run_plasm, scheduler_options
from ecto_openni import OpenNICapture, DEPTH_RGB

capture = OpenNICapture(stream_mode=DEPTH_RGB, registration=True, sync=False)

mat2pcl = conversion.MatToPointCloudXYZRGBOrganized('mat2pcl')
pcl2ecto = ecto_pcl.PointCloudT2PointCloud('pcl2ecto')

viewer = ecto_pcl.CloudViewer("viewer", window_name="Clouds!")

graph = [capture['image'] >> mat2pcl['image'],
	capture['depth'] >> mat2pcl['points'],
	mat2pcl[:] >> pcl2ecto[:],
	pcl2ecto[:] >> viewer[:]
	]

plasm = ecto.Plasm()
plasm.connect(graph)

if __name__ == "__main__":
    from ecto.opts import doit
    doit(plasm, description='Execute test kinect.')
