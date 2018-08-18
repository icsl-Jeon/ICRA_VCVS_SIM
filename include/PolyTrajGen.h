//
// Created by jbs on 18. 7. 23.
//

#ifndef TRAJ_GEN_POLYTRAJGEN_H
#define TRAJ_GEN_POLYTRAJGEN_H

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <iostream>
#include <chrono>
#include <solver.h>


using namespace Eigen;
using namespace std;

namespace TrajGen {

    /*
     * TYPEDEF
     */
    typedef VectorXd PolyCoeff;
    typedef VectorXd TimeSeries;
    typedef int PolyOrder;
    typedef int N_diff;
    typedef double Time;
    typedef double Weight;

    struct PolySpline{
        int n_seg; // number of segment
        int poly_order;
        vector<PolyCoeff> spline;
    };


    struct PolySplineXYZ {
        PolySplineXYZ() {};
        PolySpline pxs;
        PolySpline pys;
        PolySpline pzs;
        TimeSeries checkpnts; // needed when evaluting
    };

    // jerk minimization with waypoint cost ( not hard constraint )
    PolySplineXYZ min_jerk_soft(const TimeSeries&,const nav_msgs::Path&,const geometry_msgs::Twist&,Weight);
    // evaluation of spline_path with given number of interval points : for the purpose of vizualization
    nav_msgs::Path horizon_eval_spline(const PolySplineXYZ &,int);
    // for actual travel, this function will be used
    geometry_msgs::Point point_eval_spline(const PolySplineXYZ &, TrajGen::Time);



    /*TrajGen::

     * p(t)=t_vec(n_order,t,0)'p=p't_vec(n_order,t,0)
     * dot{p(t)}=t_vec(n_order,t,1)'p=p't_vec(n_order,t,1)
     * ...
     */
    VectorXd t_vec(PolyOrder ,Time,N_diff);

    /*
     * scailing matrix D : p(t)=p'D t_vec(tau) where t1<=t<=t2 , 0<=tau<=1
     */
    MatrixXd time_scailing_mat(Time,Time,PolyOrder); // consider using sparse matrix

    /*
     *  integral t_vec(n,tau,3)*t_vec(n,tau,3)' from 0 to 1
     */
    MatrixXd integral_jerk_squared(PolyOrder);


    // find belonging time interval in time series
    Index find_spline_interval(const TimeSeries&,Time);
}


// CVXGEN

namespace CVXGEN{
    void construct_qp(Params& ,MatrixXd&,MatrixXd&,MatrixXd&,MatrixXd&);
    TrajGen::PolySpline get_solution(Vars&,int ,int );
}




#endif //TRAJ_GEN_POLYTRAJGEN_H
