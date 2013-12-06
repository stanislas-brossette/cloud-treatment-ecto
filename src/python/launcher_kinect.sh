#!/usr/bin/env bash
killall -s 9 XnSensorServer

./$@

killall -s 9 XnSensorServer
