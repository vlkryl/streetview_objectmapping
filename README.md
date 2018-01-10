# streetview_objectmapping

This repository contains Python (v2.7) implementation of the MRF-based tringulation procedure introduced in

"Automatic Discovery and Geotagging of Objects from Street View Imagery"
by V. A. Krylov, E. Kenny, R. Dahyot.

Technical paper: https://arxiv.org/abs/1708.08417

Video demo: https://www.youtube.com/watch?v=X0tM_iSRJMw

-----------------------------------------------------------

License:

This code is released under the MIT License (refer to the LICENSE file for details).

Copyright (c) ADAPT centre, Trinity College Dublin, 2018.

-----------------------------------------------------------

Requirements:

Python 2.7, Numpy, Scipy.

-----------------------------------------------------------

Functionality:

The module takes the ouput of object detection and depth estimation deployed on the original image set. Each line in the input CSV file defines a detected object with FOUR floating point values: camera positions (GPS latitude and longitude), bearing from north clockwise in degrees towards the object in the panoramic image and the depth estimate. The latter may be omitted or set to zero.

The module performs triangulation, MRF optimization to establish the optimal object configuration and clustering. The optimization is done by ICM.

The output CSV contains the list of GPS-coordinates (latitude and longitude) of identified objects of interests and a score value for each of these. The score is the number of individual views contributing to an object (greater or equal to 2).

Parameters:

These are modified at the top of the python script. The main parameters are:
1) input and output file locations;
2) distance parameters: maximal camera-to-object distance and cluster size;
3) MRF optimization parameters: number of ICM iterations and energy weights as defined in Eq. (4) in the technical paper.

-----------------------------------------------------------

Sample data:

The folder 'Sample dataset' conains the traffic lights dataset used in the paper:
1) 'ground_truth.csv' contains a list of 50 traffic lights in Regent Street, London, UK;
2) 'detection_input.csv' contains the output produced by the object detection and depth estimation pipelines presented in the paper. This file is used as input for the triangulation procedure.
