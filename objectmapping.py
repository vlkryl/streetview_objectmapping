#!/usr/bin/env python

import sys
import os
import os.path
import time
import numpy as np
from math import radians, pi, cos, sin, asin, sqrt, atan2, tan, atan, exp, log
from scipy.cluster.hierarchy import linkage, fcluster

'''
------------------------------------------
This file contains Python (v2.7) implementation of the MRF-based tringulation procedure introduced in
"Automatic Discovery and Geotagging of Objects from Street View Imagery"
by V. A. Krylov, E. Kenny, R. Dahyot.
https://arxiv.org/abs/1708.08417

version 1.1
Copyright (c) ADAPT centre, Trinity College Dublin, 2018

------------------------------------------
The module takes the ouput of object detection and depth estimation deployed on the original image set. Each line in the input CSV file defines a detected object with FOUR floating point values: camera positions (GPS latitude and longitude), bearing from north clockwise in degrees towards the object in the panoramic image and the depth estimate. The latter may be omitted or set to zero.

The module performs triangulation, MRF optimization to establish the optimal object configuration and clustering.

The output CSV contains the list of GPS-coordinates (latitude and longitude) of identified objects of interests and a score value for each of these. The score is the number of individual views contributing to an object (greater or equal to 2).
------------------------------------------
'''

###########################################
###  I N P U T     P A R A M E T E R S  ###
###########################################

# Input CSV file
inputfilename = 'Sample dataset/Traffic lights 50/detection_input.csv'		
# Output CSV file
outputfilename = 'Sample dataset/Traffic lights 50/traffic_lights_detection.csv'		

# preset parameters
MaxObjectDstFromCam = 25	# Max distance from camera to objects (in meters)
MaxDstInCluster = 1		# Maximal size of clusters employed (in meters)

# MRF optimization parameters
ICMiterations = 15		# Number of iterations for ICM
DepthWeight = 0.2		# weight alpha in Eq.(4)
ObjectMultiView = 0.2		# weight beta in  Eq.(4)
StandAlonePrice = max(1 - DepthWeight - ObjectMultiView, 0) # weight (1-alpha-beta) in Eq. (4)

###########################################


# conversion from (lat,lon) to meters
def LatLonToMeters( lat, lon ):
    "Converts given lat/lon in WGS84 Datum to XY in Spherical Mercator EPSG:4326"
    originShift = 2 * pi * 6378137 / 2.0
    mx = lon * originShift / 180.0
    my = log( tan((90 + lat) * pi / 360.0 )) / (pi / 180.0)
    my = my * originShift / 180.0
    return mx, my

# conversion from meters to (lat,lon)
def MetersToLatLon( mx, my ):
    "Converts XY point from Spherical Mercator EPSG:4326 to lat/lon in WGS84 Datum"
    originShift = 2 * pi * 6378137 / 2.0
    lon = (mx / originShift) * 180.0
    lat = (my / originShift) * 180.0
    lat = 180 / pi * (2 * atan(exp(lat * pi / 180.0)) - pi / 2.0)
    return lat, lon


# haversine distance formula between two points specified by their GPS coordinates
def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    m = 6367000. * c
    return m

# calculating the intersection  point between two rays (specified each by camera position and depth-estimated object location)
def Intersect(Object1, Object2, MaxObjectDstFromCam) :
   latC1 = Object1[5]
   latC2 = Object2[5]
   lonC1 = Object1[6]
   lonC2 = Object2[6]

   latP1 = Object1[0]
   latP2 = Object2[0]
   lonP1 = Object1[1]
   lonP2 = Object2[1]
   
   a1 = latP1 - latC1
   b1 = latP2 - latC2
   c1 = latC2 - latC1

   a2 = lonP1 - lonC1
   b2 = lonP2 - lonC2
   c2 = lonC2 - lonC1

   if a2*b1-b2*a1 :
      y = (a1*c2 - a2*c1) / (a2*b1-b2*a1)
   else :
      return -1, -1, 0 ,0
   if a1 != 0 :
      x = (b1*y+c1) / a1
   else :
      x = (b2*y+c2) / a2

   if (x < 0) or (y < 0) :
      return -2, -2, 0, 0
   if (x > MaxObjectDstFromCam) or (y > MaxObjectDstFromCam) :
      return -3, -3, 0, 0
   mx, my = a1*x+latC1, a2*x+lonC1
   return x, y, mx, my

# calculate the MRF energy of an intersection
def CalcEnergyObject(ObjectsDst,ObjectsBase,ObjectsConnectivity,Object) :
   inters = np.count_nonzero(ObjectsConnectivity[Object,:])
   if inters == 0:
	return StandAlonePrice
   Energy = 0
   dpthmin, dpthmax = 1000, 0
   for i in range(len(ObjectsBase)) :
	if ObjectsConnectivity[Object,i]:
		dpthPen = DepthWeight*abs(ObjectsDst[Object,i] - (ObjectsBase[Object])[3]);
		Energy += dpthPen
		dpth = ObjectsDst[Object,i]
		if dpth<dpthmin:
			dpthmin = dpth
		if dpth>dpthmax:
			dpthmax = dpth
   return Energy + ObjectMultiView*(dpthmax-dpthmin)
	
# calculate the averaged object location (used after clustering)
def CalcAvrgObject(Intersects,ObjectsConnectivity,Object) :
   res = np.zeros(2)
   cnt = 0
   for i in range(Intersects.shape[0]) :
	if ObjectsConnectivity[Object,i]:
		res[:] += Intersects[Object,i,:]
		cnt += 1
   if cnt :
	return res/cnt
   return res	


# hierarchical clustering
def MyClust(intersects,MaxIntraDegreeDst) :
   Z = linkage(np.asarray(intersects))
   clusters = fcluster(Z, MaxIntraDegreeDst, criterion='distance') - 1
   NumClusters =  max(clusters) + 1
   IntersectClusters = np.zeros((NumClusters,3))
   for i in range(len(intersects)) :
      IntersectClusters[clusters[i],0] += (intersects[i])[0]
      IntersectClusters[clusters[i],1] += (intersects[i])[1]
      IntersectClusters[clusters[i],2] += 1
   return IntersectClusters



# only PAIRWISE intersections
def main( arguments ):

   start = time.time()
   ObjectsBase=[]

   if not os.path.isfile(inputfilename) :
	print 'Input file not found. Aborting.'
	return;

   if os.path.isfile(outputfilename) :
	print 'A file with the specified ouput name already exists. Aborting.'
	return

   try:
        f1=open(outputfilename,'w')
        f1.close()
   except:
        print 'A file with the specified ouput name cannot be created. Aborting.'
        return

   ###############################
   #### A L L  O B J E C T S #####
   ###############################


   with open(inputfilename,'r') as f:
      next(f)	# skip the first line
      for line in f:
	nums = line.split(',')
	if len(nums)<3:
		print 'Broken entry ignored'
	if len(nums)<4:	# if a depth estimate is not available
		lat, lon, bearing, depth = float(nums[0]), float(nums[1]), float(nums[2]), 5
	else:
		lat, lon, bearing, depth = float(nums[0]), float(nums[1]), float(nums[2]), float(nums[3])
	if depth<=0:
		depth =5

	# calculating the object positions from camera position + bearing + depth_estimate
	mx, my = LatLonToMeters(lat,lon)
	br1 = radians(bearing)
	yCP = my + depth * cos(br1) * 640/256	# depth-based positions
	xCP = mx + depth * sin(br1) * 640/256
	latp, lonp = MetersToLatLon(xCP, yCP)
	yCP = my + 1.0 * cos(br1) * 640/256	# normalized positions (at 1m distance from camera)
	xCP = mx + 1.0 * sin(br1) * 640/256
	latp1, lonp1 = MetersToLatLon(xCP, yCP)

	ObjectsBase.append( (latp1,lonp1,bearing,depth,0,lat,lon,latp,lonp) )

   print "All detected objects: {0:d}".format(len(ObjectsBase))

   #############################
   #### A D M I S S I B L E ####
   #############################

   #the maximal distance between the two camera positions observing the same object
   MaxCamDst = 1.5 * MaxObjectDstFromCam

   Intersects = []
   NumIntersects = 0
   ObjectsDst = np.zeros((len(ObjectsBase),len(ObjectsBase)))
   Intersects = np.zeros((len(ObjectsBase),len(ObjectsBase),2))
   for i in range(len(ObjectsBase)) :
      if i%1000 == 0 and i>0:
		print 'Parced {} object entries ({:.2f}%)'.format(i,100.*i/len(ObjectsBase))
      ObjectsDst[i,i] = -5
      for j in range(i+1,len(ObjectsBase)) :
	 CamDstMtrs = haversine((ObjectsBase[i])[6],(ObjectsBase[i])[5],(ObjectsBase[j])[6],(ObjectsBase[j])[5])

	 # cam_positions - same (less than 1m apart) or too far
         if CamDstMtrs < 0.5 or CamDstMtrs > MaxCamDst :
            ObjectsDst[i,j] = -4
            ObjectsDst[j,i] = -4
            continue
         ObjectsDst[i,j], ObjectsDst[j,i], Intersects[i,j,0], Intersects[i,j,1] = Intersect(ObjectsBase[i], ObjectsBase[j], MaxObjectDstFromCam)
	 Intersects[j,i,0], Intersects[j,i,1] = Intersects[i,j,0], Intersects[i,j,1]
         if ObjectsDst[i,j] > 0 :
            NumIntersects += 1

   print "All admissible intersections: {0:d}".format(NumIntersects)
 
   ObjectsConnectivity = np.zeros((len(ObjectsBase),len(ObjectsBase)),dtype=np.uint8)
   ObjectsConnectivityViableOptions = np.zeros(len(ObjectsBase),dtype=np.uint8)
   for i in range(len(ObjectsBase)) :
	ObjectsConnectivityViableOptions[i] = np.count_nonzero(ObjectsDst[i,:]>0)
   
   #############################
   ########### I C M ###########
   #############################

   np.random.seed(int(100000.0*time.time())%1000000000)
   chngcnt = 0
   for ICMiter in range(ICMiterations*len(ObjectsBase)) :
	if (ICMiter+1)%(len(ObjectsBase)) == 0:
		print 'Iteration #{}: accepted {} changes'.format((ICMiter+1)/(len(ObjectsBase)),chngcnt)
		chngcnt = 0
	testObject = np.random.randint(0, len(ObjectsBase))
	if ObjectsConnectivityViableOptions[testObject] == 0 :	# no pairing possible (standalone - )
		continue

	randnum  = 1+np.random.randint(0, ObjectsConnectivityViableOptions[testObject]) 
	curcnt = 0
	for i in range(len(ObjectsBase)) :
		if (ObjectsDst[testObject,i]>0) :
			curcnt += 1
		if curcnt == randnum :
			testObjectPair = i
			break

	EnergyOld  = CalcEnergyObject(ObjectsDst,ObjectsBase,ObjectsConnectivity,testObject)
	EnergyOld += CalcEnergyObject(ObjectsDst,ObjectsBase,ObjectsConnectivity,testObjectPair)

	ObjectsConnectivity[testObject, testObjectPair] = 1 - ObjectsConnectivity[testObject, testObjectPair]
	ObjectsConnectivity[testObjectPair, testObject] = 1 - ObjectsConnectivity[testObjectPair, testObject]

	EnergyNew  = CalcEnergyObject(ObjectsDst,ObjectsBase,ObjectsConnectivity,testObject)
	EnergyNew += CalcEnergyObject(ObjectsDst,ObjectsBase,ObjectsConnectivity,testObjectPair)
	
	if EnergyNew<=EnergyOld:
		chngcnt += 1
		continue
	
	# revert to the old configuration
	ObjectsConnectivity[testObject, testObjectPair] = 1 - ObjectsConnectivity[testObject, testObjectPair]
	ObjectsConnectivity[testObjectPair, testObject] = 1 - ObjectsConnectivity[testObjectPair, testObject]

   #############################
   #### C L U S T E R I N G ####
   #############################		

   mx, my = LatLonToMeters((ObjectsBase[0])[0], (ObjectsBase[0])[1])
   d45 = 0.707 * MaxDstInCluster * 640.0/256;
   ax, ay = MetersToLatLon(mx+d45, my+d45)
   ax1, ay1 =  MetersToLatLon(mx, my)
   MaxDegreeDstInCluster = ((ax-ax1)**2+(ay-ay1)**2)**0.5
   print MaxDegreeDstInCluster

   ICMintersect = []
   ifObjectIntersects = np.zeros(len(ObjectsBase),dtype=np.uint8)
   for i in range(len(ObjectsBase)) :
	res = CalcAvrgObject(Intersects,ObjectsConnectivity,i)
	if res[0] :
		ifObjectIntersects[i] = 1
		ICMintersect.append((res[0], res[1]))
		
   print "ICM inrersections: {0:d}".format(len(ICMintersect))
   IntersectClusters = MyClust(ICMintersect,MaxDegreeDstInCluster)

   NumClusters = IntersectClusters.shape[0]
   with open(outputfilename, "w") as inter:
    inter.write("lat,lon,score\n")
    for i in range(NumClusters) :
        inter.write("{0:f},{1:f},{2:d}\n".format(IntersectClusters[i,0]/IntersectClusters[i,2], \
	    IntersectClusters[i,1]/IntersectClusters[i,2],int(IntersectClusters[i,2])))
   print "Number of output ICM clusters: {0:d}".format(NumClusters)

   print "Elapsed total time: {0:.2f} seconds.".format(time.time() - start)



if __name__ == '__main__':
  main( sys.argv )
