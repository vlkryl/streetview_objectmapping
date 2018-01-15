# streetview_objectmapping

Python implementation of the Markov Random Field-based triangulation procedure assisted by camera-to-object depth estimates.

**Automatic Discovery and Geotagging of Objects from Street View Imagery**<br />
[Vladimir A. Krylov](https://sites.google.com/site/vlkryl/), Eamonn Kenny, and [Rozenn Dahyot](https://www.scss.tcd.ie/~dahyotr/).

[arXiv](https://www.youtube.com/watch?v=X0tM_iSRJMw "arXiv")

Version 1.1


## Requirements

The requirements to run the code are: Python 2.7, Numpy 1.8.0, Scipy 1.0.0.

The testing was done in Ubuntu 16.04.


## Functionality

The current module performs triangulation, MRF optimization to establish the optimal object configuration and clustering. The MRF optimization is performed by ICM.

#### Input

The module takes the output of detection modules deployed separately on each of the images in the input dataset. As in the technical paper, these are produced from street-level RGB imagery using object segmentation (to discover object instances) and monocular depth estimation (to establish approximate camera-to-object distances). 

Each line in the input CSV file defines a detected object by FOUR floating point values: camera positions (GPS latitude and longitude), bearing from north clockwise in degrees towards the object in the panoramic image and the depth estimate. The latter may be omitted or set to zero. A sample input file is provided in the 'sample dataset' folder.

#### Input parameters

These are modified at the top of the python script. The main parameters are:
1) input and output CSV file locations;
2) distance parameters: maximal camera-to-object distance and cluster size (both in meters);
3) MRF optimization parameters: number of ICM iterations and energy weights as defined in Eq. (4) in the technical paper.

#### Output

The output CSV file contains a list of GPS-coordinates (latitude and longitude) of identified objects of interests and a score value for each of these. The score is the number of individual views contributing to an object (for each of the discovered objects this value is greater or equal than 2).

## Sample data

Folder 'Sample dataset' contains the traffic lights dataset used in the paper:
1) 'ground_truth.csv' gives a list of 50 manually annotated traffic lights in Regent Street, London, UK;
2) 'detection_input.csv' contains the output produced by the object detection and depth estimation pipelines presented in the paper. This file is in the format used by the current MRF-based object triangulation implementation.

## Video demo

[![Video demo](https://img.youtube.com/vi/X0tM_iSRJMw/0.jpg)](https://www.youtube.com/watch?v=X0tM_iSRJMw)

## License

This code is released under the MIT License (refer to the LICENSE file for details).

Copyright (c) ADAPT centre, Trinity College Dublin, 2018.
