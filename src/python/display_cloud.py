#!/usr/bin/env python

import ecto, ecto_pcl
import sys
import time
import os
import cloud_treatment_ecto.cloud_treatment as cloud_treatment

plasm_ecto = ecto.Plasm()
pcdfile = 'cloud.pcd'


if len(sys.argv) > 1:
    pcdfile = sys.argv[1]

reader = cloud_treatment.PCDReaderCell(	"Reader_ecto",
					filename=pcdfile)
viewer = cloud_treatment.CloudViewerCell("Viewer_ecto",
					window_name="PCD Viewer")

graph = [reader['output'] >> viewer['input']
	]

plasm_ecto = ecto.Plasm()
plasm_ecto.connect(graph)
plasm_ecto.execute()
