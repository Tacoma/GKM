#!/usr/bin/python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

if len(sys.argv) < 2:
	sys.exit("Please give the path to a log file.")

fileName = str(sys.argv[1])
logFile = open(fileName, 'r')

print logFile

xGroundTruth = []
yGroundTruth = []
zGroundTruth = []

for line in logFile:
	coords = line.split(" ")
	xGroundTruth.append(float(coords[1]))
	yGroundTruth.append(float(coords[2]))
	zGroundTruth.append(float(coords[3]))

fig = plt.figure()
ax = Axes3D(fig)

ax.plot(xGroundTruth, yGroundTruth, zGroundTruth)
plt.show()
