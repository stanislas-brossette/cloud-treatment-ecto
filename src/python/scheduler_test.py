#!/usr/bin/env python

import ecto, ecto_pcl, ecto_ros, ecto_pcl_ros
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto.opts import scheduler_options, run_plasm
import sys
import time
import os
import argparse

import cloud_treatment_ecto.cloud_treatment as cloud_treatment

ecto_ros.init(sys.argv, "locate_ladder")

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

cloud_sub = ecto_sensor_msgs.Subscriber_PointCloud2("cloud_sub",
    topic_name='/worldmodel_main/pointcloud_vis')
msg2cloud = ecto_pcl_ros.Message2PointCloud("msg2cloud", format=ecto_pcl.XYZRGB)
cloud2msg_main = ecto_pcl_ros.PointCloud2Message("cloud2msg_main")
cloud2msg_seg = ecto_pcl_ros.PointCloud2Message("cloud2msg_seg")
cloud_pub_main = ecto_sensor_msgs.Publisher_PointCloud2(
                   "cloud_pub_main",topic_name='/ecto/main_cloud')
cloud_pub_seg = ecto_sensor_msgs.Publisher_PointCloud2(
                   "cloud_pub_seg",topic_name='/ecto/seg_cloud')

passthrough3d = cloud_treatment.PassThrough3DCell(
					"passthrough3D",
          x_min=-1.0,
          x_max=2.0,
          y_min=-1.9,
          y_max=-0.5,
          z_min=-1.0,
          z_max=5.0)

stepsegmenter = cloud_treatment.StepSegmentationCell("Step_Seg",
                z_step_1=-0.06,
                z_step_2=0.23,
                z_step_3=0.52,
                z_step_4=0.82,
                z_step_5=1.21,
                z_step_6=1.52,
                z_step_7=1.83,
                positive_threshold=0.01,
                negative_threshold=0.0
                )

principalcomponent = cloud_treatment.PrincipalComponentExtractionCell(
                "Principal_component",
                length_rectangles = 0.8128,
                width_rectangles = 0.127
                )


colorize = ecto_pcl.ColorizeClusters("colorize")

viewer = cloud_treatment.CloudViewerCell("Viewer_ecto",
					window_name="PCD Viewer")

graph = [cloud_sub["output"] >> msg2cloud[:],
         msg2cloud[:] >> passthrough3d["input"],
         passthrough3d["output"] >> stepsegmenter["input"],
         #principalcomponent["centroids"] >> viewer["VIPoints"],
         #principalcomponent["rectangles"] >> viewer["rectangles"],
         stepsegmenter["clusters"] >> colorize["clusters"],
         passthrough3d["output"] >> colorize["input"], 
         stepsegmenter["clusters"] >> principalcomponent["clusters"],
         passthrough3d["output"] >> principalcomponent["input"],  
         #colorize[:] >> viewer["input"]
         passthrough3d["output"] >> cloud2msg_main[:],
         cloud2msg_main[:] >> cloud_pub_main[:],
         colorize[:] >> cloud2msg_seg[:],
         cloud2msg_seg[:] >> cloud_pub_seg[:]
	]
#graph = [cloud_sub["output"] >> msg2cloud[:],
#         msg2cloud[:] >> cloud2msg[:],
#         cloud2msg[:] >> cloud_pub[:]
#	]

plasm = ecto.Plasm()
plasm.connect(graph)

#run_plasm(options, plasm, locals=vars())
plasm.execute(niter=1)

print "plasm_ecto executed"

