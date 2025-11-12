#!/home/fadi/scrap_ws/src/projects/scrap_burning/env/bin/python3

import rospy
from geometry_msgs.msg import Point
from scrap_burning.srv import Skeletonize
from ros_pcl_manip.srv import Voxelize
from ros_pcl_manip.srv import LoadFile
from ros_pcl_manip.srv import ToFile

import numpy as np
import skimage
from skimage.morphology import skeletonize_3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def getNeighbours(x, y, z, arr):
    nCount = -1                 # The following code includes the center point, which will add 1 always, so remove here
    for i in range(max(0, x - 1), min(x + 2, len(arr) - 1)):
        for j in range(max(0, y - 1), min(y + 2, len(arr[x]) - 1)):
            for k in range(max(0, z - 1), min(z + 2, len(arr[x,y]) - 1)):
                if arr[i, j, k]:
                    nCount += 1
    
    return nCount

# Simply calls above function with a tuple instead
def getTupleNeighbours(pt, arr):
    return getNeighbours(pt[0], pt[1], pt[2], arr)

# Returns the next adjacent neighbour, if prev is None, will return the first found, otherwise will return if !prev
def getNextNeighbour(arr, pt, prevPt=None):
    for i in range(pt[0] - 1, pt[0] + 2):
        for j in range(pt[1] - 1, pt[1]  + 2):
            for k in range(pt[2] - 1, pt[2] + 2):
                if i == pt[0] and j == pt[1] and k == pt[2]: # Skip central point
                    continue
                if arr[i, j, k] and (prevPt is None or (i, j, k) != prevPt):
                    return (i, j, k)

def getInitialPt(pts):
    for i in range(len(pts) - 1):
        for j in range(len(pts[i]) - 1):
            for k in range(len(pts[i, j]) - 1):
                if pts[i, j, k]:
                    return (i, j, k)

def getOrderedLine(pts):
    orderedPts = []
    # Get initial pt
    orderedPts.append(getInitialPt(pts))

    # Keep moving along line until we reach an endpoint
    prevPt = orderedPts[-1]
    while getTupleNeighbours(orderedPts[-1], pts) != 1:
        nextPt = getNextNeighbour(pts, orderedPts[-1], prevPt)
        prevPt = orderedPts[-1]
        orderedPts.append(nextPt)

    # Reverse order and begin moving ahead
    orderedPts.reverse()
    if len(orderedPts) == 1:
        print("First found point was an end point!")
        orderedPts.append(getNextNeighbour(pts, orderedPts[-1], prevPt))
    else:
        prevPt = orderedPts[-2]
    while getTupleNeighbours(orderedPts[-1], pts) != 1:
        nextPt = getNextNeighbour(pts, orderedPts[-1], prevPt)
        prevPt = orderedPts[-1]
        orderedPts.append(nextPt)

    return orderedPts

# Services
voxelizer = None

def skeletonize_cb(req):
    global voxelizer

    # Set leaf size (0.005 if not given)
    res = 0.005
    if req.leaf_size >= 0.001:
        res = req.leaf_size

    cloud = req.cloud

    print("Obtained cloud data, requesting voxelization")
    minPt = Point()
    maxPt = Point()
    data = voxelizer(cloud, res, False, minPt, maxPt)
    print("Voxelization complete")

    # Convert to grid array
    x = np.array([])
    y = np.array([])
    z = np.array([])
    for i in range(2, len(data.x)):
        x = np.append(x, round(data.x[i], 5))
        y = np.append(y, round(data.y[i], 5))
        z = np.append(z, round(data.z[i], 5))
    xOffset = x.min()
    yOffset = y.min()
    zOffset = z.min()
    x -= xOffset
    x = x.round(5) / res
    y -= yOffset
    y = y.round(5) / res
    z -= zOffset
    z = z.round(5) / res
    skelArr = np.zeros((int(x.max()) + 2, int(y.max()) + 2, int(z.max()) + 2), dtype='bool')
    for i in range(0, len(x)):
        skelArr[int(round(x[i])), int(round(y[i])), int(round(z[i]))] = True

    # Skeletonize array
    skeleton = skeletonize_3d(skeletonize_3d(skelArr))
    x,y,z = skelArr.nonzero()
    skx = np.array([])
    sky = np.array([])
    skz = np.array([])

    # Check neighbour count
    singleNeigh = []
    multiNeigh = []
    incNeigh = []
    for i in range(int(x.max()) + 1):
        for j in range(int(y.max()) + 1):
            for k in range(int(z.max()) + 1):
                if skeleton[i, j, k]:
                    skx = np.append(skx, i)
                    sky = np.append(sky, j)
                    skz = np.append(skz, k)
                    # Check neighbours
                    nCount = getNeighbours(i, j, k, skeleton)
                    if nCount == 2:
                        multiNeigh.append((i, j, k))
                    elif nCount == 1:
                        singleNeigh.append((i, j, k))
                    else:
                        print("Invalid neighbour count!  Got " + str(nCount))
                        incNeigh.append((i, j, k))
                        # Stop here, can't do much with this error
                        # return

    print("Found {} single-neighbours, {} dual".format(len(singleNeigh), len(multiNeigh)))
    print("{} points with incorrect neighbours".format(len(incNeigh)))

    # Order line
    orderedPts = getOrderedLine(skeleton)
    ret = []
    oX = np.array([])
    oY = np.array([])
    oZ = np.array([])
    cp = np.array([])
    cpDT = 1.0 / len(orderedPts)
    for pt in orderedPts:
        oX = np.append(oX, pt[0] * res + xOffset)
        oY = np.append(oY, pt[1] * res + yOffset)
        oZ = np.append(oZ, pt[2] * res + zOffset)
        cp = np.append(cp, len(cp) * cpDT)

    for pt in orderedPts:
        ret.append(Point(x=pt[0] * res + xOffset,
                         y=pt[1] * res + yOffset,
                         z=pt[2] * res + zOffset))

    if req.view:
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.scatter(data.x, data.y, data.z, c='#FF0000', alpha=0.1)
        ax.scatter(oX, oY, oZ, c=cp)
        plt.show()

    return {"skeleton": ret}

# CURRENTLY USED FOR TESTING
def main():
    global voxelizer

    rospy.init_node('skeletonize_node')

    # Setup service
    voxelizer = rospy.ServiceProxy('voxelize', Voxelize)
    skeletonize = rospy.Service("skeletonize", Skeletonize, skeletonize_cb)
    loader = rospy.ServiceProxy('load_pcd_file', LoadFile)
    print("Setup services, spinning")
    rospy.spin()
    
if __name__ == '__main__':
    main()
