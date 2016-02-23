#!/usr/bin/python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

if len(sys.argv) < 2:
	sys.exit("Please give the paths to three log files.")

logFile = open(str(sys.argv[1]), 'r')

#evaluation log 
xPos = []
yPos = []
zPos = []
xProjPos = []
yProjPos = []
zProjPos = []
i=0
for line in logFile:
	if i % 2 == 0:
		coords = line.split(",")
		xPos.append(float(coords[0]))
		yPos.append(float(coords[1]))
		zPos.append(float(coords[2]))
	else:
		coords = line.split(",")
		xProjPos.append(float(coords[0]))
		yProjPos.append(float(coords[1]))
		zProjPos.append(float(coords[2]))
	i = i+1
	
#static plane def
#point
pxPos = -0.35191
pyPos = 2.3416
pzPos = 1.83686
#normal
#pnxPos = 0
#pnyPos = 1
#pnzPos = 0

#calculate distances
xDistances = []
yDistances = []
zDistances = []
for i in range(0, len(xPos)):
	xDistances.append(xPos[i]-pxPos)
	yDistances.append(yPos[i]-pyPos)
	zDistances.append(zPos[i]-pzPos)

#get min and max points
#yMin = min(yPos)
#yMax = max(yPos)
yMin = min(yDistances)
yMax = max(yDistances)

#draw
fig = plt.figure()
ax = Axes3D(fig)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
#ax.plot(xPos, yPos, zPos, color="blue")
ax.plot(xDistances, yDistances, zDistances, color="red")

plt.axis([-5,5,-5,5])
plt.plot([0,0], [yMin,yMax], 'ro')
plt.plot([0,0], [yMin,yMax], '--')
plt.title(yMax-yMin)
plt.show()
