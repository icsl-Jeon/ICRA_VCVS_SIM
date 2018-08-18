//
// Created by jbs on 18. 6. 29.
//

#ifndef ASAP_ASAP_H
#define ASAP_ASAP_H

// CUSTOM HEADERS


#include "utils.h"

class ASAP{

private:
    // parameters for ASAP algorithm
    asap_ns::Params params;
    // linspace for azimuth(col) , elevation(row) set
    Eigen::VectorXf azim_set, elev_set;

    /****
     * Target
     */

    // target history
    std::deque<geometry_msgs::Point> target_history;
    // time history of target
    std::deque<double> time_history;
    // target history regression model
    LinearModel regress_model[3]; // x,y,z

    // predicted target path
    TargetPrediction target_prediction;



    /****
     * Planning
     */

    // polyspline
    TrajGen::PolySplineXYZ splineXYZ;

    // corresponding time vector
    std::vector<double> planning_horizon; // NOTE : first component is ros::Time::now(). Thus, actual future step is remainders
    std::vector<double> planning_horizon_saved;
    // sequnce of viewpoints
    ViewSequence view_path;
    // octomap object
    std::shared_ptr<octomap::OcTree> octree_obj;




    /****
     * Graph
     */
    // graph for VP computation
    Graph g;
    // set of layers of candidate nodes
    geometry_msgs::Point graph_init_point;
    asap_ns::LayerSet cur_layer_set;

    // dictionary for vertex descriptor
    DescriptorMap descriptor_map;
    // color map for plotting the candidate node
    cv::Mat bgr[3];
    // quad_path
    trajectory_msgs::MultiDOFJointTrajectory quad_waypoint;


public:
    // constructor destructor
    ASAP(asap_ns::Params);
    ~ASAP();

    // recording
    void record(nav_msgs::Path&,nav_msgs::Path&,visualization_msgs::MarkerArray &);


    /***********************
     * Member variables
     */

    // velocity
    geometry_msgs::Twist cur_tracker_vel;

    // current position of tracker
    geometry_msgs::Point cur_tracker_pos;
    geometry_msgs::Point cur_target_pos;

    // flags
    bool state_callback_flag;
    bool octomap_callback_flag;
    bool model_regression_flag;

    // target prediction error
    double prediction_error;

    // ROS
    ros::NodeHandle nh; // getting parameters

    ros::Subscriber octomap_sub;
    ros::Subscriber states_sub;
    ros::Subscriber points_sub;
    ros::Subscriber odometry_sub;

    ros::Publisher candidNodes_marker_pub; // points of local maximum in visibility matrix
    ros::Publisher pnts_pub; // points (target prediction points)
    ros::Publisher skeleton_pub; // skeleton solved from view_path_solver
    ros::Publisher path_pub; // view path
    ros::Publisher smooth_path_pub; // smooth path publisher
    ros::Publisher target_pred_path_pub; // predicted target future history
    ros::Publisher node_pub; // node marker publisher
    ros::Publisher edge_pub; // edge arrow publisher
    ros::Publisher waypoint_viz_pub; // waypoint marker publisher
    ros::Publisher traj_pub; // trajectory publisher
    ros::Publisher BBMarker_pub; // bounding box publisher

    ros::ServiceServer solve_server; // server for solving view path
    // id
    std::string world_frame_id; // default: "world"

    // rviz
    visualization_msgs::Marker marker; // marker for candidate nodes
    visualization_msgs::Marker pnt_marker; // marker for pnts
    visualization_msgs::Marker waypoint_marker; // marker for quad waypoint
    visualization_msgs::Marker velocity_marker; // marker for velocity of tracker
    visualization_msgs::Marker skeleton_pnt_marker; // marker for pnts
    visualization_msgs::Marker node_marker; // marker for nodes
    visualization_msgs::Marker edge_marker; // marker for edges
    visualization_msgs::MarkerArray arrow_array; // array of arrow
    visualization_msgs::Marker BBMarker; // bounding box marker
    int edge_id;


    // time object
    ros::Time check_pnt;
    ros::Time init_time;
    ros::Duration check_duration;

    // tracker name
    std::string tracker_name;
    // target name
    std::string target_name;


    /***********************
     * Member funtions
     */

    /*
     * Graph
     */
    void graph_init();  // node add wtih current position of tracker
    asap_ns::Layer get_layer(geometry_msgs::Point,int);
    void add_layer(asap_ns::Layer,double,double); // add the layer and connect it with the last layer
    void graph_wrapping(); // add the last layer
    bool solve_view_path(); // update view sequence of tracker by solving Dijkstra's shortest path algorithm

    /*
     * Utils
     */
    MatrixXd castRay(geometry_msgs::Point,float,bool=false); // cast ray in octomap
    void hovering(ros::Duration,double); // execute hovering for quadrotor


    /*
     * Alogrithm
     */
    void target_regression(); // regression on history
    void target_future_prediction(); // update target future trajectory
    bool reactive_planning(double,double); // graph construction + solve path altogether
    void smooth_path_update(); // spline curve saving



    // publish
    void marker_publish(); // marker (line list) for candidate nodes
    void points_publish(); // for the purpose of test (publish the received points)
    void path_publish();   // solution path publication
    void quad_waypoint_pub(); // waypoint publish for quad

    // callback (subsrcibe)
    void points_callback(kiro_gui_msgs::PositionArray); // this callback function is for getting point from rviz
    void state_callback(const gazebo_msgs::ModelStates::ConstPtr&);
    bool solve_callback(asap::SolvePath::Request&,asap::SolvePath::Response&); // service callback
    void octomap_callback(const octomap_msgs::Octomap&);
};



#endif //ASAP_ASAP_H
