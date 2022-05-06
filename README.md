# cross-section-shape-context (CSSC) descriptor
This repo contains the code of CSSC descriptor for paper: "A LiDAR-based single-shot global localization solution using a cross-section shape context descriptor".

CSSC is a global descriptor for LiDAR point cloud, which describes the spatial distribution characteristics of the point cloud from two dimensions. CSSC descriptors can be utilized for loop closure detection

# Publication

# How to use
The code was tested with Ubuntu 18.04, PCL 1.8.0.

For a quick demo for evaluating similarity between two LiDAR scans using  CSSC descriptors.
```bash
git clone https://github.com/Dongxu05/CSSC.git
cd CSSC
mkdir build && cd build
cmake ..
make
./testCSSC
```

# License
