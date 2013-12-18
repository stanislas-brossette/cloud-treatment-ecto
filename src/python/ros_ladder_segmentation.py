#!/usr/bin/env python

import ecto, ecto_pcl, ecto_ros, ecto_pcl_ros
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
import ecto_ros.ecto_geometry_msgs as ecto_geometry_msgs
import ecto_ros.ecto_std_msgs as ecto_std_msgs
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

### ROS SUBSCRIBERS ###
cloud_sub = ecto_sensor_msgs.Subscriber_PointCloud2("cloud_sub",
                topic_name='/worldmodel_main/pointcloud_vis')

### ROS PUBLISHERS ###
cloud_pub_main = ecto_sensor_msgs.Publisher_PointCloud2(
                "cloud_pub_main",topic_name='/ecto/main_cloud')
cloud_pub_seg = ecto_sensor_msgs.Publisher_PointCloud2(
                "cloud_pub_seg",topic_name='/ecto/seg_cloud')
poseStamped_pub = ecto_geometry_msgs.Publisher_PoseStamped("poseStamped_pub",
                topic_name="/ecto/poseStamped")
poseStamped_pubTEST = ecto_geometry_msgs.Publisher_PoseStamped("poseStamped_pub",
                topic_name="/ecto/poseStampedTEST")
surfName_pub = ecto_std_msgs.Publisher_String("surfName_pub",
                topic_name="/ecto/surfName")
rectpub0=ecto_geometry_msgs.Publisher_PolygonStamped("rectpub0",topic_name='/ecto/polygon0')
rectpub1=ecto_geometry_msgs.Publisher_PolygonStamped("rectpub1",topic_name='/ecto/polygon1')
rectpub2=ecto_geometry_msgs.Publisher_PolygonStamped("rectpub2",topic_name='/ecto/polygon2')
rectpub3=ecto_geometry_msgs.Publisher_PolygonStamped("rectpub3",topic_name='/ecto/polygon3')
rectpub4=ecto_geometry_msgs.Publisher_PolygonStamped("rectpub4",topic_name='/ecto/polygon4')
rectpub5=ecto_geometry_msgs.Publisher_PolygonStamped("rectpub5",topic_name='/ecto/polygon5')

### ROS MSG EMITORS ###
msg2cloud = ecto_pcl_ros.Message2PointCloud("msg2cloud", format=ecto_pcl.XYZRGB)
cloud2msg_main = ecto_pcl_ros.PointCloud2Message("cloud2msg_main")
cloud2msg_seg = ecto_pcl_ros.PointCloud2Message("cloud2msg_seg")
recalibrate2msg = cloud_treatment.RecalibrateMsgCell("recalMsgEnv", choice_index=3)
recalibrate2msgTEST = cloud_treatment.RecalibrateMsgCell("recalMsgEnv", choice_index=3)
rectangle2msg = cloud_treatment.RectanglesPubCell("rectangle2msg")
extract_header = cloud_treatment.ExtractHeaderCell('extractHeader')
edit_header = cloud_treatment.EditHeaderCell('editHeader', frame_id="/map")


### CLOUD TREATMENTS ###
passthrough3d = cloud_treatment.PassThrough3DCell(
                "passthrough3D",
                x_min=-1.0,
                x_max=1.0,
                y_min=-5.0,
                y_max=-3.0,
                z_min=-1.0,
                z_max=2.5)
stepsegmenter = cloud_treatment.StepSegmentationCell("Step_Seg",
                z_step_1=0.0,
                z_step_2=0.30,
                z_step_3=0.60,
                z_step_4=0.90,
                z_step_5=1.20,
                z_step_6=1.50,
                z_step_7=1.80,
                positive_threshold=0.01,
                negative_threshold=0.0
                )
principalcomponent = cloud_treatment.PrincipalComponentExtractionCell(
                "Principal_component",
                length_rectangles = 0.8128,
                width_rectangles = 0.127
                )
stepcentering = cloud_treatment.StepCenteringCell("step_centering")
colorize = ecto_pcl.ColorizeClusters("colorize")
viewer = cloud_treatment.CloudViewerCell("Viewer_ecto",
                window_name="PCD Viewer")

### GRAPH ###
## SUBSCRIPTION 
graph =  [cloud_sub["output"] >> edit_header[:],
         edit_header[:] >> msg2cloud[:],
         edit_header[:] >> extract_header[:]
         ]
## CLOUD TREATMENT
graph += [msg2cloud[:] >> passthrough3d["input"],
         passthrough3d["output"] >> stepsegmenter["input"],
         stepsegmenter["clusters"] >> colorize["clusters"],
         passthrough3d["output"] >> colorize["input"], 
         stepsegmenter["clusters"] >> principalcomponent["clusters"],
         passthrough3d["output"] >> principalcomponent["input"],
         stepsegmenter["clusters"] >> stepcentering["clusters"],
         principalcomponent["eigenvectors"] >> stepcentering["frames"],
         passthrough3d["output"] >> stepcentering["input"]
         ]

## OUTPUT
graph += [principalcomponent["rectangles"] >> rectangle2msg["rectangles"],
         extract_header[:] >> rectangle2msg["header"],
         principalcomponent["eigenvectors"] >> recalibrate2msg["frames"],
         principalcomponent["eigenvectors"] >> recalibrate2msgTEST["frames"],
         principalcomponent["centroids"] >> recalibrate2msg["origins"],
         stepcentering["centers"] >> recalibrate2msgTEST["origins"],
         extract_header[:] >> recalibrate2msg["header"],
         extract_header[:] >> recalibrate2msgTEST["header"],
         passthrough3d["output"] >> cloud2msg_main[:],
         colorize[:] >> cloud2msg_seg[:]
      	 ]

## PUBLISH
graph += [recalibrate2msg["pose_stamped_msg"] >> poseStamped_pub[:],
         recalibrate2msgTEST["pose_stamped_msg"] >> poseStamped_pubTEST[:],
         rectangle2msg["rectanglemsg0"] >> rectpub0[:],
         rectangle2msg["rectanglemsg1"] >> rectpub1[:],
         rectangle2msg["rectanglemsg2"] >> rectpub2[:],
         rectangle2msg["rectanglemsg3"] >> rectpub3[:],
         rectangle2msg["rectanglemsg4"] >> rectpub4[:],
         rectangle2msg["rectanglemsg5"] >> rectpub5[:],
         cloud2msg_main[:] >> cloud_pub_main[:],
         cloud2msg_seg[:] >> cloud_pub_seg[:],
         recalibrate2msg["surf_name"] >> surfName_pub[:]
         ]

#graph = [cloud_sub["output"] >> msg2cloud[:],
#         msg2cloud[:] >> cloud2msg[:],
#         cloud2msg[:] >> cloud_pub[:]
#	]

### EXECUTION ###
plasm = ecto.Plasm()
plasm.connect(graph)

run_plasm(options, plasm, locals=vars())
#plasm.execute(niter=1)

print "plasm_ecto executed"

