#include <iostream>
#include "cassert"
#include "algorithm"
#include <numeric>
#include"Eigen/Dense"
/*
    计算一个容器内数据的均值与对角形式协方差
*/
template <typename C,typename D,typename  Getter>
void computeMeanAndCovDiag(const C& data, D& mean,D& cov_diag,Getter& getter){

    size_t len = data.size();
    assert(len > 1);
    mean = std::accumulate(data.begin(), data.end(),D::Zero().eval(),
    [&getter](const D& sum, const auto& data)->  D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);                       
}

/*
    计算容器内的均值和协方差
*/
template <typename C,int dim,typename Getter>
void computeMeanAndCovDiag(const C &data, Eigen::Matrix<double , dim, 1>& mean, Eigen::Matrix<double,dim,dim>& cov, Getter &getter){


    using D = Eigen::Matrix<double , dim, 1>;
    using E = Eigen::Matrix<double, dim, dim>;
    size_t len = data.size();
    assert(len >1);


    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(), [&getter](const D& sum,const auto& data)->D {return sum + getter(data);})/len;
    cov = std::accumulate(data.begin(), data.end(), E::Zero().eval(),
                            [&getter,&mean](const E& sum,const auto& data)-> E { D v  = getter(data) - mean;
                            return sum + v * v.transpose();})/(len -1);

}







