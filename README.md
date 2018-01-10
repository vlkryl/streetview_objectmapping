# streetview_objectmapping

This repository contains Python (v2.7) implementation of the MRF-based tringulation procedure introduced in
"Automatic Discovery and Geotagging of Objects from Street View Imagery"
by V. A. Krylov, E. Kenny, R. Dahyot.
https://arxiv.org/abs/1708.08417

version 1.1
Copyright (c) ADAPT centre, Trinity College Dublin, 2018

-----------------------------------------------------------

Functionality:

The module takes the ouput of object detection and depth estimation deployed on the original image set. Each line in the input CSV file defines a detected object with FOUR floating point values: camera positions (GPS latitude and longitude), bearing from north clockwise in degrees towards the object in the panoramic image and the depth estimate. The latter may be omitted or set to zero.

The module performs triangulation, MRF optimization to establish the optimal object configuration and clustering.

The output CSV contains the list of GPS-coordinates (latitude and longitude) of identified objects of interests and a score value for each of these. The score is the number of individual views contributing to an object (greater or equal to 2).

-----------------------------------------------------------

Sample data:

The folder 'Sample dataset' conains the Regent Street dateset with traffic lights introduced in the paper:
1) 'ground_truth.csv' contains a list of 50 traffic lights
2) 'detection_input.csv' contains the output produced by the object detection and depth estimation pipelines presented in the paper. This file is used as input for the triangulation procedure.
