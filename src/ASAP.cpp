//
// Created by jbs on 18. 6. 29.
//


#include "ASAP.h"

using namespace asap_ns;

// constructor
ASAP::ASAP(asap_ns::Params params):nh("~"){
    // parameter parsing
    this->params=params;
    // stack of target position history
    target_history.resize(this->params.N_history);
    time_history.resize(this->params.N_history);

    prediction_error=params.replanning_trigger_error+1; // so that planning is started at the first time
    isDistMapInit = false;


    // now
	double duration=0.05; // 20Hz
	check_duration=ros::Duration(duration);
    // history horizon = duration x N_history
	check_pnt=ros::Time::now();
    init_time=ros::Time::now();


    nh.getParam("world_frame_id",world_frame_id);
    nh.getParam("inflation_length",inflation_length);


    std::cout<<"world frame id:"<<world_frame_id<<std::endl;
    target_prediction.header.frame_id=world_frame_id;
    // name

    target_name=params.target_name;
    tracker_name=params.tracker_name;

    std::cout<<"inserted tracker name: "<<tracker_name<<std::endl;

    quad_waypoint.header.frame_id=world_frame_id;

    // subscribe
    octomap_sub=nh.subscribe("/octomap_full",3,&ASAP::octomap_callback,this);
    states_sub=nh.subscribe("/gazebo/model_states",10,&ASAP::state_callback,this);

    // advertise
    path_pub=nh.advertise<nav_msgs::Path>("view_sequence",3);
    target_pred_path_pub=nh.advertise<nav_msgs::Path>("target_prediction_path",2);
    pnts_pub=nh.advertise<visualization_msgs::Marker>("clicked_pnts",2);
    candidNodes_marker_pub=nh.advertise<visualization_msgs::Marker>("candidate_nodes",2);
    skeleton_pub=nh.advertise<visualization_msgs::Marker>("skeleton",2);
    waypoint_viz_pub=nh.advertise<visualization_msgs::Marker>("current_waypoint",2);
    node_pub=nh.advertise<visualization_msgs::Marker>("nodes_in_layer",2);
    edge_pub=nh.advertise<visualization_msgs::MarkerArray>("edge_in_layer",2);
    BBMarker_pub=nh.advertise<visualization_msgs::Marker>("bounding_box_target",2);
    smooth_path_pub=nh.advertise<nav_msgs::Path>("smooth_path",2);
    traj_pub=nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+tracker_name+"/"+mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    inflation_marker_pub = nh.advertise<visualization_msgs::Marker>("inflation_region",2);

    // bgr colormap setting
    cv::Mat cvVec(1,params.N_pred,CV_8UC1);
    cv::Mat colorVec;
    for(int i=0;i<params.N_pred;i++)
        cvVec.at<uchar>(i)=i*(255/float(params.N_pred-1));

    applyColorMap(cvVec,colorVec,COLORMAP_JET);
    split(colorVec,bgr);
   

    // service
    solve_server = nh.advertiseService("solve_path",&ASAP::solve_callback,this);

    // candid nodes marker init
    marker.header.frame_id = world_frame_id;
    marker.header.stamp  = ros::Time::now();
    marker.ns = "candidate_nodes";
    marker.action = visualization_msgs::Marker::ADD;
    float scale=0.1;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = 1;
    marker.color.a = 0.5;



    // candid nodes marker init
    inflation_marker.header.frame_id = world_frame_id;
    inflation_marker.header.stamp = ros::Time::now();
    inflation_marker.ns = "inflation_markers";
    inflation_marker.action = visualization_msgs::Marker::ADD;
    // let's decide resolution later
    inflation_marker.pose.orientation.w = 1.0;
    inflation_marker.id = 0;
    inflation_marker.type = visualization_msgs::Marker::CUBE_LIST;
    inflation_marker.color.r = 0.2;
    inflation_marker.color.g = 0.2;
    inflation_marker.color.b = 0.2;
    inflation_marker.color.a = 0.3;


    octomap::point3d min(0,0,0);
    octomap::point3d max(2,2,2);
    octree_obj.reset(new octomap::OcTree(0.4));
    distmap_ptr = new DynamicEDTOctomap(1,octree_obj.get(),min,max,true);

    // skeleton marker
    float skeleton_scale=0.1;
    skeleton_pnt_marker=marker;
    skeleton_pnt_marker.ns="skeleton";
    skeleton_pnt_marker.scale.x=skeleton_pnt_marker.scale.y=skeleton_pnt_marker.scale.z=skeleton_scale;
    skeleton_pnt_marker.color.r=0; skeleton_pnt_marker.color.g=1; skeleton_pnt_marker.color.b=0;


    // waypoint marker
    waypoint_marker=marker;
    waypoint_marker.ns="current_waypoint";
    waypoint_marker.type=visualization_msgs::Marker::SPHERE;
    waypoint_marker.color.r=1; waypoint_marker.color.g=0; waypoint_marker.color.b=0;




    // pnts marker init
    pnt_marker.header.frame_id = world_frame_id;
    pnt_marker.header.stamp  = ros::Time::now();
    pnt_marker.ns = "clicked_pnts";
    pnt_marker.action = visualization_msgs::Marker::ADD;
    float len=0.1;
    pnt_marker.pose.orientation.w = 1.0;
    pnt_marker.id = 0;
    pnt_marker.type = visualization_msgs::Marker::CUBE_LIST;
    pnt_marker.scale.x = len;
    pnt_marker.scale.y = len;
    pnt_marker.scale.z = len;
    pnt_marker.color.r = 1;
    pnt_marker.color.g = 0.3;
    pnt_marker.color.a = 0.8;

    // nodes marker init
    node_marker.header.frame_id = world_frame_id;
    node_marker.header.stamp  = ros::Time::now();
    node_marker.ns = "nodes";
    node_marker.action = visualization_msgs::Marker::ADD;
    node_marker.pose.orientation.w = 1.0;
    node_marker.id = 0;
    node_marker.type = visualization_msgs::Marker::POINTS;
    node_marker.scale.x = scale/2;
    node_marker.scale.y = scale/2;
    node_marker.scale.z = scale/2;
//    node_marker.color.r = 1;
    node_marker.color.a = 0.8;
	node_marker.lifetime=ros::Duration(duration);

    // edge marker init
	edge_id=0;
	edge_marker.header.frame_id = world_frame_id;
    edge_marker.header.stamp  = ros::Time::now();
    edge_marker.ns = "edges";
    edge_marker.action = visualization_msgs::Marker::ADD;
    edge_marker.pose.orientation.w = 1.0;
    edge_marker.id = edge_id;
    edge_marker.type = visualization_msgs::Marker::ARROW;
    edge_marker.scale.x = 0.008;
    edge_marker.scale.y = 0.01;
    edge_marker.scale.z = 0.01;
    edge_marker.color.b = 0.8;
    edge_marker.color.g = 0.8;
    edge_marker.color.r = 0;
    edge_marker.color.a = 0.5;
    edge_marker.lifetime=ros::Duration(duration);



    // Bounded Box around target
    BBMarker.header.frame_id=world_frame_id;
    BBMarker.header.stamp=ros::Time::now();
    BBMarker.ns="targetBB";
    BBMarker.action=visualization_msgs::Marker::ADD;
    BBMarker.id=0;
    BBMarker.type=visualization_msgs::Marker::CUBE;

    BBMarker.pose.orientation.x = 0.0;
    BBMarker.pose.orientation.y = 0.0;
    BBMarker.pose.orientation.z = 0.0;
    BBMarker.pose.orientation.w = 1.0;
    double lx=0.5,ly=0.5,lz=0.5;

    BBMarker.scale.x = lx * 2;
    BBMarker.scale.y = ly * 2;
    BBMarker.scale.z = lz * 2 ;

    BBMarker.color.r=1.0;
    BBMarker.color.a=0.2;


    // flags
    state_callback_flag= false;
    state_callback_flag= false;
    model_regression_flag=true;


    // azimuth, elevation set constructing
    azim_set.setLinSpaced(params.N_azim,0,2*PI);
    elev_set.setLinSpaced(params.N_elev,params.elev_min,params.elev_max);



}

void ASAP::hovering(ros::Duration dur,double hovering_z){
	
        double hovering_start= ros::Time::now().toSec();
		ROS_INFO_ONCE("hovering during %f [sec]",dur.toSec());
	    Eigen::Vector3d waypoint(0, 0,hovering_z);
		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(waypoint,0, &quad_waypoint);

		while (ros::Time::now().toSec()-hovering_start<dur.toSec())
        traj_pub.publish(quad_waypoint);
	
}

void ASAP::graph_init() {
    // graph init
    g=Graph();
    descriptor_map.clear();
    cur_layer_set.clear();

    Vertex x0 = boost::add_vertex(std::string("x0"), g);
    descriptor_map.insert(make_pair("x0",x0));

    Layer base_layer;

    // current tracker position does not work unless the perfomance of controller is great

    if(this->splineXYZ.checkpnts.size())
        graph_init_point=TrajGen::point_eval_spline(this->splineXYZ,ros::Time::now().toSec());
    else
        graph_init_point=cur_tracker_pos;

    base_layer.nodes.push_back(CandidNode("x0",graph_init_point,-1));
    base_layer.t_idx=0; // initial index t0
    cur_layer_set.push_back(base_layer);

    // clear the markers
    node_marker.points.clear();
    node_marker.colors.clear();
    arrow_array.markers.clear();
	edge_id=0;
}

void ASAP::record(nav_msgs::Path& target_path_ptr, nav_msgs::Path& tracker_path_ptr, visualization_msgs::MarkerArray& bearing_vector_ptr) {

    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped tracker_pose;

    target_pose.pose.position=cur_target_pos;
    tracker_pose.pose.position=cur_tracker_pos;

    target_path_ptr.poses.push_back(target_pose);
    tracker_path_ptr.poses.push_back(tracker_pose);

    // edge marker init
    visualization_msgs::Marker arrow;
    arrow.header.frame_id=world_frame_id;
    arrow.ns = "record_arrow";
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.id = bearing_vector_ptr.markers.size();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.scale.x = 0.07;
    arrow.scale.y = 0.11;
    arrow.scale.z = 0.1;
    arrow.color.b = 0.0;
    arrow.color.g = 0.1;
    arrow.color.r = 0.2;
    arrow.color.a = 0.5;

    arrow.points.push_back(cur_tracker_pos);
    arrow.points.push_back(cur_target_pos);

    bearing_vector_ptr.markers.push_back(arrow);


}

// I think it can be optimized more
MatrixXd ASAP::castRay(geometry_msgs::Point rayStartPnt, float ray_length,bool verbose ) {


    // initialize the elements
    MatrixXd castResultBinary(params.N_elev,params.N_azim);
    castResultBinary.setConstant(-1);

    // castRay w.r.t sampling azimuth and elevation

    if (octree_obj->size()) {
//        printf("casting started with light distance %.4f\n",ray_length);

        bool ignoreUnknownCells =true;
        octomap::point3d light_start(float(rayStartPnt.x),float(rayStartPnt.y),float(rayStartPnt.z));

        // generate mesh
        // ray casting & update castResult and castResultBinary
        for (unsigned int ind_elev = 0; ind_elev < params.N_elev; ind_elev++)
            for (unsigned int ind_azim = 0; ind_azim < params.N_azim; ind_azim++) {

                octomap::point3d light_dir(float (cos(elev_set[ind_elev])*cos(azim_set[ind_azim])),
                                  float(cos(elev_set[ind_elev]) * sin(azim_set[ind_azim])),
                                  float(sin(elev_set[ind_elev])));
                octomap::point3d light_end = light_start +  light_dir * ray_length; //endpoint of ray whether the ray was hit

                if(this->collision_check(light_start,light_end))
                    castResultBinary.coeffRef(ind_elev,ind_azim) = 1;
                else
                    castResultBinary.coeffRef(ind_elev,ind_azim) = 0;

//                // if hit
//                if(octree_obj->castRay(light_start, light_dir, light_end,
//                                       ignoreUnknownCells, ray_length))
//                    castResultBinary.coeffRef(ind_elev, ind_azim) = 1;
//                else
//                    // no hit = just tracking distance
//                    castResultBinary.coeffRef(ind_elev,ind_azim)=0;
            }

        // print the cast result
        if (verbose)
            std::cout<<castResultBinary<<std::endl;
    }// if octree is not empty
    else
      printf("Octree has zero size\n");


    return castResultBinary;
}


Layer ASAP::get_layer(geometry_msgs::Point light_source,int t_idx){

    asap_ns::Params & param=this->params;
    int local_key=0;
    bool isFree;
    Layer layer;
    layer.t_idx=t_idx;
    MatrixXd sdf;

    // casting ray
    for(vector<float>::iterator it = param.tracking_ds.begin(); it!=param.tracking_ds.end();++it)
    {
        float cur_d=*it;
        MatrixXd binaryCast=castRay(light_source,cur_d);
        vector<IDX> extrema;
        // if we SEDT on zero matrix, it will be disaster
        if(binaryCast.isZero(0)) {
            isFree=true;
            std::cout << "===============================" << std::endl;
            std::cout << binaryCast << std::endl;
            std::cout << "-------------------------------" << std::endl;
            std::cout << "Equal distribution" <<std::endl;
            std::cout << "===============================" << std::endl;
            extrema = equal_dist_idx_set(binaryCast.rows(), binaryCast.cols(),1,8);
//            ROS_INFO("found extrema: %d", extrema.size());

        }
        else {

            sdf = SEDT(binaryCast); // signed distance field
            isFree=false;
            std::cout << "===============================" << std::endl;
            std::cout << binaryCast << std::endl;
            std::cout << "-------------------------------" << std::endl;
            std::cout << sdf << std::endl;
            std::cout << "===============================" << std::endl;
            // normalization should be performed
            mat_normalize(sdf); // sdf normalized
            extrema = localMaxima(sdf, params.N_extrem, params.local_range);
//            ROS_INFO("found extrema: %d", extrema.size());
        }

        // insert N_extrem many nodes
        for(vector<IDX>::iterator it_idx = extrema.begin();it_idx!=extrema.end();it_idx++) {

            IDX idx=*it_idx;
//            ROS_INFO("extrema: [%d, %d]",idx[0],idx[1]);

            float cur_azim=azim_set[idx(1)];
            float cur_elev=elev_set[idx(0)];
//            ROS_INFO("azim,elev: [%f, %f]",cur_azim,cur_elev);


            ViewVector viewVector;
            viewVector.azim=cur_azim; viewVector.elev=cur_elev; viewVector.ray_length=cur_d;
            viewVector.centerPnt=light_source;

            // node insertion
            CandidNode candidNode;
//            candidNode.id="d"+to_string(d_key)+"_"+to_string(local_key);
            candidNode.id="t"+to_string(t_idx)+"_"+to_string(local_key);
            candidNode.position=viewVector.getEndPnt();

            if(!isFree)
                candidNode.visibility=sdf(idx[0],idx[1]);
            else
                candidNode.visibility=FREE_VISIBILITY;
            layer.nodes.push_back(candidNode);
            local_key++;
        }
    }
    return layer;
}


/**
 * Time index
 * base : t0
 * layer1: t1
 * layer2: t2 ...
 */

void ASAP::add_layer(Layer layer,double d_max,double d_max0) {


    // no layer
    if (cur_layer_set.empty())
        printf("graph has not been initialized yet");

    // only base layer exits
    else if (cur_layer_set.size()==1) {
//        ROS_INFO("current tracker position: [%f, %f, %f]\n",cur_tracker_pos.x,cur_tracker_pos.y,cur_tracker_pos.z);


        vector<CandidNode>::iterator min_vis_node_it=std::min_element(layer.nodes.begin(),layer.nodes.end(),compare_visibility);

        float min_vis=min_vis_node_it->visibility;

        for (vector<CandidNode>::iterator it = layer.nodes.begin(); it != layer.nodes.end(); it++) {


            // problem may occur

            octomap::point3d P1((graph_init_point.x), graph_init_point.y, graph_init_point.z);
            octomap::point3d P2(it->position.x, it->position.y, it->position.z);

            node_marker.points.push_back(it->position);
            std_msgs::ColorRGBA c;
            double red=(bgr[2]).at<uchar>(layer.t_idx-1)/255.0;
            double green=(bgr[1]).at<uchar>(layer.t_idx-1)/255.0;
            double blue=(bgr[0]).at<uchar>(layer.t_idx-1)/255.0;
            c.r=float(red); c.g=float(green); c.b=float(blue); c.a=0.7;
            node_marker.colors.push_back(c);


            // assign vertex name to this candid node
            VertexName name = (it->id); // time info + distance info + local order info
            // register to graph

            Vertex v = boost::add_vertex(name, g); // add vertex corresponding to current node to graph
            descriptor_map.insert(make_pair(name, v));


            // connect edge
            float dist = P1.distance(P2);
            float vis = it->visibility;

            // heuristic to compare free element with those not be hit

            if(vis==FREE_VISIBILITY)
                vis=min_vis;


//            std::cout<<"dist: "<<dist<<" ";
//            std::cout<<"visibility: "<<vis<<" ";
            Weight w;
            w=dist+params.w_v0*(layer.t_idx)/vis; // we assgin higher weight for later target position
//            std::cout<<"weight: "<<w<<std::endl;
            if (dist <d_max0 and !(this->collision_check(P1,P2))){
                boost::add_edge(descriptor_map["x0"], v, w, g);
                edge_marker.points.clear();
                edge_marker.points.push_back(graph_init_point);
                edge_marker.points.push_back(it->position);
                edge_marker.id=edge_id++;
				arrow_array.markers.push_back(visualization_msgs::Marker(edge_marker));

            }

        }
//        printf("---------connecting complete--------\n");
        // insert this layer
        cur_layer_set.push_back(layer);
    }
    // we connect last layer stacked in graph
    else {

        // register to graph
        for(auto it = layer.nodes.begin(),end=layer.nodes.end();it != end;it++){
            // assign vertex name to this candidnode
            VertexName name=(it->id); // time info + distance info + local order info
            Vertex v = boost::add_vertex(name, g); // add vertex corresponding to current node to graph
            descriptor_map.insert(make_pair(name,v));

            // marker construct
            node_marker.points.push_back(it->position);
            std_msgs::ColorRGBA c;
            double red=(bgr[2]).at<uchar>(layer.t_idx-1)/255.0;
            double green=(bgr[1]).at<uchar>(layer.t_idx-1)/255.0;
            double blue=(bgr[0]).at<uchar>(layer.t_idx-1)/255.0;
            c.r=float(red); c.g=float(green); c.b=float(blue); c.a=0.7;
            node_marker.colors.push_back(c);


        }

        // connect layer with toppest layer
        Layer prev_layer= cur_layer_set.back();

        for(auto it1 = prev_layer.nodes.begin(),end1=prev_layer.nodes.end();it1 !=end1;it1++)
            for(auto it2 = layer.nodes.begin(),end2=layer.nodes.end();it2 != end2;it2++)
            {
                octomap::point3d P1(it1->position.x,it1->position.y,it1->position.z);
                octomap::point3d P2(it2->position.x,it2->position.y,it2->position.z);
                double dist=P1.distance(P2);
//                printf("node distance: %.4f\n",dist);
                if (dist<d_max and !(this->collision_check(P1,P2))){
                    Weight w;
                    w=P1.distance(P2)+params.w_v0*layer.t_idx/it2->visibility;


                    boost::add_edge(descriptor_map[it1->id], descriptor_map[it2->id], w, g);
					edge_marker.points.clear();
					edge_marker.id=edge_id++;
					edge_marker.points.push_back(it1->position);
                    edge_marker.points.push_back(it2->position);
                    arrow_array.markers.push_back(visualization_msgs::Marker(edge_marker));

                }
            }

//        printf("---------connecting complete--------\n");
        cur_layer_set.push_back(layer);

    }
}


void ASAP::graph_wrapping() {

    // register vertex to graph
    Vertex xf = boost::add_vertex(std::string("xf"), g);
    descriptor_map.insert(make_pair("xf",xf));

    // last layer append
    Layer finishing_layer;
    CandidNode dummy_node;
    dummy_node.id="xf";
    finishing_layer.nodes.push_back(dummy_node);
    // connect all the nodes in the last layer with dummy node having assigning weight
    Layer prev_layer= cur_layer_set.back();
    for(auto it = prev_layer.nodes.begin(),end=prev_layer.nodes.end();it != end;it++){
        Weight w=10; // just dummy constant
        boost::add_edge(descriptor_map[it->id],descriptor_map["xf"], w, g);
    }

    cur_layer_set.push_back(finishing_layer);
}



bool ASAP::solve_view_path() {
    // find path using and save the Path into member functions

    auto t0 = std::chrono::high_resolution_clock::now();
    GraphPath graphPath=Dijkstra(g,descriptor_map["x0"],descriptor_map["xf"]);
    auto t1 = std::chrono::high_resolution_clock::now();
    auto dt = 1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count();


//    std::cout<<"Dijkstra solving time: "<<dt<<std::endl;


//	printf("--------------Path solve----------------------");

    this->view_path=nav_msgs::Path();
	this->view_path.header.frame_id=world_frame_id;

    if (graphPath.size()){
//    std::cout<<"solved path received"<<std::endl;
    for(auto it = graphPath.begin(),end=graphPath.end();it != end;it++){
        VertexName id=*it;

//        std::cout<<"this id:"<<id<<" ";

        if (id == "x0"){
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.pose.position=graph_init_point;
            view_path.poses.push_back(poseStamped);
//			std::cout<<std::endl;
        }else if(id =="xf"){
            // skip : no insertion
        }else{
            // str =  "t1_1" or "t1_13" for example
            int t_idx=int(id[1])-'0';
            int local_idx;
            if(id.length()==5)
                local_idx=(int(id[3])-'0')*10+int(id[4])-'0'; // let's limit the N_node per layer
            else
                local_idx=int(id[3])-'0';
        	
//			std::cout<<"extracted local_idx: "<<local_idx<<std::endl;
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.pose.position=cur_layer_set[t_idx].nodes[local_idx].position;
            view_path.poses.push_back(poseStamped);
        }
    }
        return true;
    }else
        return false;
}


void ASAP::target_regression() {

    if (target_history.size()){
        VectorXd ts(target_history.size()),xs(target_history.size()),ys(target_history.size()),zs(target_history.size());
        for (int i=0;i<target_history.size();i++){
			ts.coeffRef(i)=time_history[i];
            xs.coeffRef(i)=target_history[i].x;
            ys.coeffRef(i)=target_history[i].y;
            zs.coeffRef(i)=target_history[i].z;
        }

		


//
//		std::cout<<"xs: "<<xs<<std::endl;
//		std::cout<<"ys: "<<ys<<std::endl;
//		std::cout<<"zs: "<<zs<<std::endl;


        regress_model[0]=linear_regression(ts,xs);
        regress_model[1]=linear_regression(ts,ys);
        regress_model[2]=linear_regression(ts,zs);
        
//		std::cout<<"current regression model:"<<std::endl;
//		std::cout<<"x: "<<std::endl;
//		std::cout<<"beta0: "<<regress_model[0].beta0<<" beta1: "<<regress_model[0].beta1<<std::endl;
//		std::cout<<"y: "<<std::endl;
//		std::cout<<"beta0: "<<regress_model[1].beta0<<" beta1: "<<regress_model[1].beta1<<std::endl;
//		std::cout<<"z: "<<std::endl;
//		std::cout<<"beta0: "<<regress_model[2].beta0<<" beta1: "<<regress_model[2].beta1<<std::endl;

		// reset the prediction error of target
	    prediction_error=0;


		model_regression_flag=true;
    }
    else
        ROS_WARN_ONCE("target history does not exist.");
}

void ASAP::target_future_prediction() {
    // predict the future trajectory from the last time segment until the next t_pred with N_pred

    if(model_regression_flag) {
        VectorXd pred_seq(params.N_pred);
        planning_horizon.clear();
        pred_seq.setLinSpaced(params.N_pred, ros::Time::now().toSec(), ros::Time::now().toSec() + params.t_pred);

        target_prediction = TargetPrediction();
        target_prediction.header.frame_id = world_frame_id;
        double t;


        // this is planning horizon at predicting the future path of target
		for(int insert_idx=0;insert_idx<pred_seq.size();insert_idx++)
			planning_horizon.push_back(pred_seq.coeff(insert_idx));

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position=cur_target_pos;
        geometry_msgs::PoseStamped next_poseStamped; // may not be used...


        int idx=1;
//        std::cout<<"future target feasibility check"<<std::endl;
        // why start from 1:  0 is not future value.
        for (; idx < params.N_pred; idx++) {

            t = pred_seq.coeff(idx);
            next_poseStamped.pose.position.x = model_eval(regress_model[0], t);
            next_poseStamped.pose.position.y = model_eval(regress_model[1], t);
            next_poseStamped.pose.position.z = model_eval(regress_model[2], t);

//            std::cout<<"between "<<idx-1<<" and "<<idx<<" : ";

            // inspect whether obstacles exist along the current pose and next pose
            octomap::point3d light_start(float(poseStamped.pose.position.x),
                                         float(poseStamped.pose.position.y),
                                         float(poseStamped.pose.position.z));

            octomap::point3d light_end(float(next_poseStamped.pose.position.x),
                                       float(next_poseStamped.pose.position.y),
                                       float(next_poseStamped.pose.position.z));

            octomap::point3d light_dir=light_end-light_start;
            double rayLength=light_dir.norm();
//            std::cout<<"inspection length: "<<rayLength<<"/ result: ";
            bool isHit=octree_obj->castRay(light_start,light_dir,light_end,true,rayLength);
            if(not isHit) {
                target_prediction.poses.push_back(next_poseStamped);
                poseStamped = next_poseStamped;
            } else
                break;

        } //end for


        // repeatedly push back the remainder with the last pose before break
        for(int i=idx;i<params.N_pred;i++)
            target_prediction.poses.push_back(poseStamped);
    }
}

bool ASAP::reactive_planning(double d_max,double d_max0) {

    planning_horizon_saved=planning_horizon;



    // after retreiving the target prediction (model_regression_flag=true)
    // Building graph

    graph_init();
    int t_idx=1;
//    ROS_INFO("size of prediction pnts: %d",target_prediction.poses.size());
    for (auto it = target_prediction.poses.begin(),end=target_prediction.poses.end();it != end;it++,t_idx++)
    {

        Layer layer=get_layer(it->pose.position,t_idx);
//        printf("------------------------------\n");
//        ROS_INFO("found layer: %dth predicition",t_idx);
        add_layer(layer,d_max,d_max0);
    }

    graph_wrapping();
//    ROS_INFO("finished graph");


//    // graph inspection
//
//    IndexMap index = get(boost::vertex_index, g);
//    std::cout << "vertices(g) = ";
//    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
//    std::pair<vertex_iter, vertex_iter> vp;
//    for (vp = vertices(g); vp.first != vp.second; ++vp.first) {
//        Vertex v = *vp.first;
//        std::cout << index[v] <<  " ";
//    }
//    std::cout << std::endl;
//
//    std::cout << "edges(g) = ";
//    boost::graph_traits<Graph>::edge_iterator ei, ei_end;
//    for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
//        std::cout << "(" << index[source(*ei, g)]
//                  << "," << index[target(*ei, g)] << ") ";
//    std::cout << std::endl;
//
//
//    std::cout << "number of node markers: "<<node_marker.points.size()<<std::endl;


    return solve_view_path();
}


void ASAP::smooth_path_update() {

    // map std::vector to VectorXd for time series
    TrajGen::TimeSeries ts=Map<VectorXd>(planning_horizon.data(),planning_horizon.size());


	// check if timevector is correctly stored
//	std::cout<<ros::Time::now()<<" in "<<ts.transpose()<<std::endl;

    // construct waypoints

    double w_j=params.w_j;
    this->splineXYZ = TrajGen::min_jerk_soft(ts,view_path,cur_tracker_vel,w_j);

}


void ASAP::quad_waypoint_pub() {

    // smooth path following
    if(view_path.poses.size()) {


        ros::Time now=ros::Time::now();


        geometry_msgs::Point following_point=TrajGen::point_eval_spline(this->splineXYZ,ros::Time::now().toSec());

        Eigen::Vector3d waypoint(following_point.x,following_point.y, following_point.z);
        double yaw = atan2(cur_tracker_pos.y - cur_target_pos.y, cur_tracker_pos.x - cur_target_pos.x) + PI;


        waypoint_marker.pose.position=following_point;
        waypoint_viz_pub.publish(waypoint_marker);


        quad_waypoint.header.stamp = ros::Time::now();
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(waypoint, yaw, &quad_waypoint);
       traj_pub.publish(quad_waypoint);
    }


	/**
    // skeleton path following
    if(view_path.poses.size()) {
        double now = ros::Time::now().toSec();


//        std::cout<<"now: "<<now<<" planning time horizon: [";
//        for(auto it=planning_horizon_saved.begin(),end=planning_horizon_saved.end();it!=end;it++)
//            std::cout<<*it<<" ";
//        std::cout<<"]"<<std::endl;


        std::vector<double> xs, ys, zs;
        // for interpolation
        path2vec(view_path, xs, ys, zs);
        // interpolation
        double x = interpolate(planning_horizon_saved, xs, now, true);
        double y = interpolate(planning_horizon_saved, ys, now, true);
        double z = interpolate(planning_horizon_saved, zs, now, true);

        Eigen::Vector3d waypoint(x, y, z);
        double yaw = atan2(cur_tracker_pos.y - cur_target_pos.y, cur_tracker_pos.x - cur_target_pos.x) + PI;

        quad_waypoint.header.stamp = ros::Time::now();
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(waypoint, yaw, &quad_waypoint);

        traj_pub.publish(quad_waypoint);
    }
	**/

}

void ASAP::state_callback(const gazebo_msgs::ModelStates::ConstPtr& gazebo_msg) {

    std::vector<std::string> model_names=gazebo_msg->name;
    std::vector<geometry_msgs::Pose> pose_vector=gazebo_msg->pose;
    std::vector<geometry_msgs::Twist> twist_vector=gazebo_msg->twist;

    long tracker_idx=std::find(model_names.begin(),model_names.end(),this->tracker_name)-model_names.begin();
    long target_idx=std::find(model_names.begin(),model_names.end(),this->target_name)-model_names.begin();


    //extract target state
    if (tracker_idx<model_names.size()) {
        cur_tracker_pos = pose_vector[tracker_idx].position;
        cur_tracker_vel = twist_vector[tracker_idx];
        state_callback_flag = true;
    }
    else
        ROS_WARN_ONCE("specified tracker name was not found in gazebo");

    //extract target state
    if (target_idx<model_names.size()) {

        cur_target_pos=pose_vector[target_idx].position;
        cur_target_pos.z=0.3; // let's

        // accumulating prediction error
        if(model_regression_flag) {
            ros::Time now = ros::Time::now();
            double target_pred_x = model_eval(regress_model[0], now.toSec());
            double target_pred_y = model_eval(regress_model[1], now.toSec());
            double target_pred_z = model_eval(regress_model[2], now.toSec());

            prediction_error += pow(cur_target_pos.x - target_pred_x, 2) +
                                pow(cur_target_pos.y - target_pred_y, 2) +
                                pow(cur_target_pos.z - target_pred_z, 2);

//            ROS_INFO("accum prediction error: %f",prediction_error);
        }


        // we insert every 20Hz for target history

		if (ros::Time::now()-check_pnt>check_duration)
		{
		// update check point
		check_pnt=ros::Time::now();
		// insertion
		if(target_history.size()>=this->params.N_history) {
            target_history.pop_front();
            time_history.pop_front();
        }
        // time and target insertion
        target_history.push_back(cur_target_pos);
        time_history.push_back(ros::Time::now().toSec());
    	}
	}
    else
        ROS_WARN_ONCE("specified target name was not found in gazebo");


//    //update for visualize
//    BBMarker.pose=targetPose;
//    BBMarker.header.stamp=ros::Time::now();
//
//    //extract tracker state
//    if (tracker_idx<model_names.size())
//        trackerPose=pose_vector[tracker_idx];
//    else
//        ROS_WARN("specified tracker name was not found in gazebo");

}

bool ASAP::collision_check(octomap::point3d P1,octomap::point3d P2) {

    if ((P1-P2).norm() != 0) {
        std::vector<octomap::point3d> traverse_points;
        this->octree_obj->computeRay(P1, P2, traverse_points);

        for (auto it = traverse_points.begin(); it < traverse_points.end(); it++) {
            octomap::OcTreeNode *node = octree_obj->search(*it);
            if(node) // known
            {   octomap::OcTreeKey key = octree_obj->coordToKey(*it);
                if(octree_obj->isNodeOccupied(node))
                    return true;
            }
            //unknown point : pass...

        }
        return false;
    }else{
        return false;
    }

}

bool ASAP::solve_callback(asap::SolvePath::Request& req,asap::SolvePath::Response& rep) {

    // Building graph

    graph_init();
    int t_idx=1;
    ROS_INFO("size of prediction pnts: %d",target_prediction.poses.size());
    for (auto it = target_prediction.poses.begin(),end=target_prediction.poses.end();it != end;it++,t_idx++)
    {

        Layer layer=get_layer(it->pose.position,t_idx);
        printf("------------------------------\n");
        ROS_INFO("found layer: %dth predicition",t_idx);
        add_layer(layer,params.max_interval_distance,params.max_interval_distance_init);
    }

    graph_wrapping();
    ROS_INFO("finished graph");


    // graph inspection

    IndexMap index = get(boost::vertex_index, g);
    std::cout << "vertices(g) = ";
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vp;
    for (vp = vertices(g); vp.first != vp.second; ++vp.first) {
        Vertex v = *vp.first;
        std::cout << index[v] <<  " ";
    }
//    std::cout << std::endl;

    std::cout << "edges(g) = ";
    boost::graph_traits<Graph>::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
        std::cout << "(" << index[source(*ei, g)]
                  << "," << index[target(*ei, g)] << ") ";
//    std::cout << std::endl;


    std::cout << "number of node markers: "<<node_marker.points.size()<<std::endl;


    solve_view_path();
    ROS_INFO("Dijkstra solved");

    return true;
}

void ASAP::octomap_callback(const octomap_msgs::Octomap & msg) {


    octomap_callback_flag=true;

//    octree_obj->clear();
//    octomap_msgs::readTree(octree_obj,msg);

    octree_obj.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(msg)));

    //octree update

    // EDT & inflation

    // nodes around tracker
    octomap::point3d tracker_pos(cur_tracker_pos.x,cur_tracker_pos.y,cur_tracker_pos.z);
    double lxx = this->params.tracking_ds.back() * 3;
    double lyy = this->params.tracking_ds.back() * 3;
    double lzz = 2;

    octomap::point3d inflation_min_point(-lxx,-lyy,-lzz);
    octomap::point3d inflation_max_point(lxx,lyy,lzz);


    double min_x,min_y,min_z,max_x,max_y,max_z;
    octree_obj->getMetricMin(min_x,min_y,min_z);
    octree_obj->getMetricMax(max_x,max_y,max_z);
    bool unknownAsOccupied = false;
    double maxDist = 0.5;



    DynamicEDTOctomap distmap_temp(maxDist,octree_obj.get(),
                                   tracker_pos + inflation_min_point,
                                   tracker_pos + inflation_max_point,
                                   unknownAsOccupied);



    distmap_temp.update(); // calculate distance field
    *distmap_ptr = distmap_temp;



    isDistMapInit = true;

    // update distmap
    inflation_marker.points.clear();
    inflation_marker.scale.x = float(octree_obj->getResolution());
    inflation_marker.scale.y = float(octree_obj->getResolution());
    inflation_marker.scale.z = float(octree_obj->getResolution());

    octomap::point3d viz_margin(float(octree_obj->getResolution()),
                                float(octree_obj->getResolution()),
                                float(octree_obj->getResolution()));

    double thresMax = octree_obj->getClampingThresMax();


    for (octomap::OcTree::leaf_bbx_iterator it = octree_obj->begin_leafs_bbx(inflation_min_point + tracker_pos + viz_margin ,
                                                                             inflation_max_point + tracker_pos - viz_margin ),
                 end = octree_obj->end_leafs_bbx(); it != end; ++it) {
        octomap::point3d cur_point = it.getCoordinate();
        if (distmap_ptr->getDistance(cur_point) < inflation_length){
            // inflate this voxel
            it->setLogOdds(octomap::logodds(thresMax));
            geometry_msgs::Point node_center;
            node_center.x = cur_point.x();
            node_center.y = cur_point.y();
            node_center.z = cur_point.z();
            inflation_marker.points.push_back(node_center);
        }
    }



    // free node around target
    octomap::point3d light_start(cur_target_pos.x,cur_target_pos.y,cur_target_pos.z);
    double lx=0.5,ly=0.5,lz=0.5;

    octomap::point3d freebox_min_point(-lx,-ly,-lz);
    octomap::point3d freebox_max_point(lx,ly,lz);
    double thresMin = octree_obj->getClampingThresMin();

    for (octomap::OcTree::leaf_bbx_iterator it = octree_obj->begin_leafs_bbx(freebox_min_point + light_start,
                                                                    freebox_max_point + light_start),
                 end = octree_obj->end_leafs_bbx(); it != end; ++it)
        it->setLogOdds(octomap::logodds(thresMin));

    octree_obj->updateInnerOccupancy();

    // bounding box (freed region of target)
    BBMarker.pose.position=cur_target_pos;
    BBMarker_pub.publish(BBMarker);

}


void ASAP::marker_publish() {

//    // marker construction
//    marker.points.clear();
//    for (auto layer_it = cur_layer_set.begin(),layer_end=cur_layer_set.end();layer_it != layer_end;layer_it++)
//        for (auto node_it = layer_it->nodes.begin(),node_end=layer_it->nodes.end();node_it != node_end;node_it++)
//            marker.points.push_back(node_it->position);
//
//    // marker publish
//    candidNodes_marker_pub.publish(marker);

    // node publish
    if (node_marker.points.size())
        ROS_INFO_ONCE("size of nodes : %d",node_marker.points.size());
    node_pub.publish(node_marker);

    // edge publish
    edge_pub.publish(arrow_array);
    if (isDistMapInit)
        inflation_marker_pub.publish(inflation_marker);

}


void ASAP::points_publish() {

    // marker construction
    pnt_marker.points.clear();
    for (auto it = target_prediction.poses.begin(),end=target_prediction.poses.end();it != end;it++)
        pnt_marker.points.push_back(it->pose.position);

    // marker publish
    pnts_pub.publish(pnt_marker);


    // skeleton publish

    skeleton_pnt_marker.points.clear();

    for (auto it = view_path.poses.begin(),end=view_path.poses.end();it != end;it++)
        skeleton_pnt_marker.points.push_back(it->pose.position);

    skeleton_pub.publish(skeleton_pnt_marker);

}

void ASAP::path_publish() {

    path_pub.publish(view_path);
    target_pred_path_pub.publish(target_prediction);
    nav_msgs::Path smooth_path_viz=TrajGen::horizon_eval_spline(splineXYZ,4);
    smooth_path_viz.header.frame_id=world_frame_id;
    smooth_path_pub.publish(smooth_path_viz);
}



ASAP::~ASAP() {
    delete octree_obj.get();
}


