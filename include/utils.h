//
// Created by jbs on 18. 6. 29.
//

#ifndef ASAP_UTILS_H
#define ASAP_UTILS_H
/**
 * INCLUDE
 */

// STD
#include <iostream>
#include <cmath>
#include <string>
#include <map>
#include <deque>
#include <algorithm>


// EIGEN
#include <Eigen/Core>
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <kiro_gui_msgs/PositionArray.h>
#include <asap/SolvePath.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>


// OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

// BOOST

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>


// OCTOMAP
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <octomap_msgs/conversions.h>


// POLY_TARJ_GEN
#include "PolyTrajGen.h"


using namespace std;
using namespace Eigen;
using namespace cv;

/**
 *  DEF
 */

#define PI 3.141592
#define BB 2;
const float FREE_VISIBILITY=99;
/**
 * STRUCTURE
 */
namespace asap_ns {
    struct Params {
        int N_azim, N_elev;
        float azim_min,azim_max,elev_min,elev_max;
        int local_range,N_extrem; // used in finding local maxima
        int N_node_per_layer;
        vector<float> tracking_ds; // set of tracking distances ! caution::increasing order
        float max_interval_distance; // allowable interval distance of two nodes
        float max_interval_distance_init; // allowable interval distance of two nodes
        float w_v0; // visibility weight
        float w_j; // weight for jerk minimization. smaller value -> waypoint
        float alpha; // time varying weight factor w_v0*alpha*t_idx or w_v0*alpha^t_idx
        // like linspace
        void set_ds(float start,float end,int N){
            tracking_ds.clear();
            float delta=(end-start)/(N-1);
            for(int i=0;i<N;i++)
                tracking_ds.push_back(start+i*delta);
        }

        int N_history; // number of stacked history
        int N_pred; // number of prediction
        int t_pred; // prediction horizion [sec]
        string tracker_name; // tracker name
        string target_name; // target name
        double replanning_trigger_error; // if accumlated error exceeds this value, replan

    };

// candidate node of a layer
    struct CandidNode {
        string id;
        geometry_msgs::Point position;
        float visibility; // visibility infomation (should be [0,1])
        CandidNode() {};
        CandidNode(string id,geometry_msgs::Point point,float vis) {this->id=id;this->position=point;this->visibility=vis;};
    };

    struct Layer{
        vector<CandidNode> nodes;
        int t_idx;
    };

    typedef vector<Layer> LayerSet;
}

struct ViewVector{

    ViewVector(){};
    double azim;
    double elev;
    double ray_length;
    geometry_msgs::Point centerPnt; // starting point of view vector

    /** this method returns endpoint of view vector  **/
    geometry_msgs::Point getEndPnt(){
        geometry_msgs::Point endPnt;
        endPnt.x=centerPnt.x+ray_length*cos(elev)*cos(azim);
        endPnt.y=centerPnt.y+ray_length*cos(elev)*sin(azim);
        endPnt.z=centerPnt.z+ray_length*sin(elev);
        return endPnt;
    }


    double getYaw(){
        return azim+PI;
    }
    /** this method returns proposed view pose
     *  the Transform class can be readily used to mavros waypoint publishing
    **/
    geometry_msgs::Transform getPoseFromViewVector(){
        geometry_msgs::Transform transform;

        geometry_msgs::Vector3 viewPosition;

        viewPosition.x=getEndPnt().x;
        viewPosition.y=getEndPnt().y;
        viewPosition.z=getEndPnt().z;

        geometry_msgs::Quaternion viewYaw;

        double yaw=elev+PI;

        viewYaw=tf::createQuaternionMsgFromYaw(yaw);

        transform.translation=viewPosition;
        transform.rotation=viewYaw;

        return transform;
    }

};

struct LinearModel{
    // x=beta0+beta1*t
    double beta0;
    double beta1;
};

struct CastResult{
    MatrixXd castResultMat;
    geometry_msgs::Point light_source;
    float ray_length;
};


/**
 * TYPEDEF
 **/


typedef Vector2i IDX; // index type = (row,col)
typedef string VertexName;

typedef double Weight;
typedef boost::property<boost::edge_weight_t, Weight> WeightProperty;
typedef boost::property<boost::vertex_name_t, std::string> NameProperty;

typedef boost::adjacency_list < boost::listS, boost::vecS, boost::directedS,
        NameProperty, WeightProperty > Graph;
typedef boost::graph_traits < Graph >::vertex_descriptor Vertex;
typedef boost::property_map < Graph, boost::vertex_index_t >::type IndexMap;
typedef boost::property_map < Graph, boost::vertex_name_t >::type NameMap;

typedef boost::iterator_property_map < Vertex*, IndexMap, Vertex, Vertex& > PredecessorMap;
typedef boost::iterator_property_map < Weight*, IndexMap, Weight, Weight& > DistanceMap;
typedef vector<VertexName> GraphPath;

typedef nav_msgs::Path TargetHistory;
typedef nav_msgs::Path TargetPrediction;
typedef nav_msgs::Path ViewSequence;

typedef map<VertexName,Vertex> DescriptorMap;

/**
 * FUNCTION
 */


// signed distance Euclidian transform of binary matrix
MatrixXd SEDT(MatrixXd);
// find local max idx (row,col) of an matrix
IDX maxElem(const MatrixXd& );
// a defined number of local maximaindcies of a matrix in a given range
vector<IDX> localMaxima(const MatrixXd&,int,int );
// finding shortest path using Dijkstra's path alogrithm
GraphPath Dijkstra(Graph ,Vertex ,Vertex);
void mat_normalize(MatrixXd&); // matrix normalization with maximum value
vector<IDX> equal_dist_idx_set(int,int,int,int); // equal distribution when no cast is hit by obstacles
// Linear regressionctorXd& ,const Eigen::VectorXd& );
// evaluation with model
double model_eval(const LinearModel &,double);
// interpolation to get a waypoint
double interpolate(std::vector<double> &,std::vector<double> &,double,bool);
// extract xyz std::vector from nav_msgs
void path2vec(const nav_msgs::Path&,std::vector<double> &,std::vector<double> &,std::vector<double> &);
// linear regression
LinearModel linear_regression(const Eigen::VectorXd& ,const Eigen::VectorXd& );
// compare visibility
bool compare_visibility(asap_ns::CandidNode,asap_ns::CandidNode);


#endif //ASAP_UTILS_H
