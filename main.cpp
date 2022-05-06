#include <iostream>
#include "fstream"

#include "cssc/csscM.h"

#include <pcl/io/pcd_io.h>



int main(int argc, char ** argv){

    //demo for estimating similariyt between two LiDAR scans using CSSC descriptors.
    pcPtrxyz cloud1(new pcxyz), cloud2(new pcxyz);
    pcl::io::loadPCDFile("../data/64line-1.pcd",*cloud1);
    pcl::io::loadPCDFile("../data/64line-2.pcd",*cloud2);

    csscM cssc;
    float similarity = cssc.calculateDistanceBtnPC(cloud1,cloud2).first;
    cout<<"similarity between two scans:  "<<similarity<<endl;

    return 0;
}
