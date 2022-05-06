
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>



using namespace Eigen;
using namespace std;

typedef pcl::PointXYZ xyz;
typedef pcl::PointCloud<xyz> pcxyz;
typedef pcxyz::Ptr pcPtrxyz;


class csscM {

public:
    //obtain the state of each bin in divided support region
    vector<vector<vector<int>>> getBinState(pcPtrxyz &cloud);

    //generate CSSC descriptors from state of bins
    MatrixXd getCSSC(vector<vector<vector<int>>> &_state);

    //calculate similarity between two LiDAR scans
    std::pair<double, int> calculateDistanceBtnPC (pcPtrxyz &cloud1, pcPtrxyz &cloud2 );

    //reset the parameters of the CSSC descriptor
    void resize(int rows_,int cols_ ,int hors_);

private:

    int i_cols_ = 20,i_rows_=8,i_hor_ = 40;

};
