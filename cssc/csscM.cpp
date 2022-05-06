#include "csscM.h"


MatrixXd circshift1( MatrixXd &_mat, int _num_shift )
{
    // shift columns to right direction
    assert(_num_shift >= 0);

    if( _num_shift == 0 )
    {
        MatrixXd shifted_mat( _mat );
        return shifted_mat; // Early return
    }

    MatrixXd shifted_mat = MatrixXd::Zero( _mat.rows(), _mat.cols() );
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

}

double cosdistance ( MatrixXd &_dsc1, MatrixXd &_dsc2 )
{
    int num_eff_cols = 0;
    double sum_sector_similarity = 0;

    for ( int col_idx = 0; col_idx < _dsc1.cols(); col_idx++ )
    {
        VectorXd col_dsc1 = _dsc1.col(col_idx);
        VectorXd col_dsc2 = _dsc2.col(col_idx);

        if( col_dsc1.norm() == 0 | col_dsc2.norm() == 0 )
            continue;
        double sector_similarity = col_dsc1.dot(col_dsc2) / (col_dsc1.norm() * col_dsc2.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }

    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;

}

std::vector<float> eig2stdvec( MatrixXd _eigmat )
{
    std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );
    return vec;
}

vector<vector<vector<int>>> csscM::getBinState(pcPtrxyz &cloud) {

    vector<vector<vector<int>>> state(i_rows_,vector<vector<int>>(i_cols_,vector<int>(i_hor_)));
//  64-line LiDAR vertical FOV [-24,2]
    float h_gap = (float)360/i_hor_;
    float d_gap = (float)80/i_cols_;
    float v_gap = (float)32/i_rows_;
    for (pcl::PointXYZ p : cloud->points)
    {
        float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
        float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 24;
        float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;

        int Q_dis = std::min(std::max((int)floor(dis/d_gap), 0), i_cols_-1);
        int Q_arc = std::min(std::max((int)floor(arc /v_gap), 0), i_rows_-1);
        int Q_yaw = std::min(std::max((int)floor(yaw /h_gap), 0), i_hor_-1);
        state[Q_arc][Q_dis][Q_yaw] =  state[Q_arc][Q_dis][Q_yaw]+1;
    }
    return state;
}

MatrixXd csscM::getCSSC( vector<vector<vector<int>>> &_state ) {
    MatrixXd m(i_cols_,i_hor_);
    vector<vector<vector<float>>>weight(i_rows_,vector<vector<float>>(i_cols_,vector<float>(i_hor_)));
    //calculate the point density weight
    for (int row_idx = 0; row_idx < i_rows_; ++row_idx) {
        for (int col_idx = 0; col_idx < i_cols_; col_idx++) {
            vector<int> cnts;
            for (int hor_idx = 0; hor_idx < i_hor_; ++hor_idx) {
                cnts.push_back(_state[row_idx][col_idx][hor_idx]);
            }
            std::sort(cnts.begin(),cnts.end());
            int median = cnts[20];
            for (int hor_idx = 0; hor_idx < i_hor_; ++hor_idx) {
                if (median ==0 || _state[row_idx][col_idx][hor_idx] > 2*median){
                    weight[row_idx][col_idx][hor_idx] = 1;
                }
                else
                    weight[row_idx][col_idx][hor_idx] = (float)_state[row_idx][col_idx][hor_idx]/(2*median);
            }
        }
    }

    //calculate each element in the CSSC descriptor
    for ( int col_idx = 0; col_idx < i_cols_; col_idx++ ) {
        for (int hor_idx = 0; hor_idx < i_hor_; ++hor_idx) {
            float element =0;
            for (int row_idx = 0; row_idx < i_rows_; ++row_idx) {
                if (_state[row_idx][col_idx][hor_idx]>1)
                    //each element equals to products point density and elevation weight
                    element+= weight[row_idx][col_idx][hor_idx] * pow(2,row_idx);
            }
            m(col_idx,hor_idx) = element;
        }
    }
    return m;
}

std::pair<double, int> csscM::calculateDistanceBtnPC ( pcPtrxyz &cloud1, pcPtrxyz &cloud2 ){
    vector<vector<vector<int>>> state1 = getBinState(cloud1);
    vector<vector<vector<int>>> state2 = getBinState(cloud2);

    float min_distance = 1;
    int trans = 0;
    MatrixXd m1 = getCSSC(state1);
    MatrixXd m2 = getCSSC(state2);
    //here we use force way to calculate coarse yaw angle. And similarity is obtained.
    for (int i = 0; i < i_hor_; ++i) {
        MatrixXd transM2 = circshift1(m2,i);
        float dis = cosdistance(m1,transM2);
        if (dis<min_distance){
            min_distance = dis;
            trans = i;
        }
    }
    return make_pair(min_distance,trans);
}

void csscM::resize(int rows_,int cols_ ,int hors_) {
    i_rows_ =rows_;
    i_cols_ =cols_;
    i_hor_ = hors_;
}






