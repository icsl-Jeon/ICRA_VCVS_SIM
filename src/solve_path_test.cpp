#include "ASAP.h"


// CVXGEN
Params params;
Vars vars;
Workspace work;
Settings settings;

int main(int argc,char **argv){

    set_defaults();
    settings.eps=1e-6;
    settings.resid_tol=1e-6;
    settings.verbose=0;
    setup_indexing();



    ros::init(argc,argv,"solving_path_test_node");
    // parameter parsing
    ros::NodeHandle nh_private("~");





    asap_ns::Params asap_params;

    asap_params.azim_min=0; asap_params.azim_max=2*PI;

    float tracking_d_min,tracking_d_max;
    int N_tracking_d;
    double solving_speed;

    std::string tracker_name,target_name;

    nh_private.getParam("tracking_d_min", tracking_d_min);
    nh_private.getParam("tracking_d_max", tracking_d_max);
    nh_private.getParam("tracking_d_N",N_tracking_d);

    asap_params.set_ds(tracking_d_min,tracking_d_max,N_tracking_d);

    nh_private.getParam("elev_min", asap_params.elev_min);
    nh_private.getParam("elev_max", asap_params.elev_max);

    nh_private.getParam("N_azim", asap_params.N_azim);
    nh_private.getParam("N_elev",asap_params.N_elev);
    nh_private.getParam("local_range",asap_params.local_range);
    nh_private.getParam("N_extrema",asap_params.N_extrem);
    nh_private.getParam("N_history",asap_params.N_history);
    nh_private.getParam("N_prediction",asap_params.N_pred);
    nh_private.getParam("t_prediction",asap_params.t_pred); // horizon
    nh_private.getParam("solving_speed",solving_speed); // horizon



    nh_private.getParam("max_interval_distance",asap_params.max_interval_distance);
    nh_private.getParam("max_interval_distance_init",asap_params.max_interval_distance_init);

    nh_private.getParam("w_v0",asap_params.w_v0);
    nh_private.getParam("w_j",asap_params.w_j);
    nh_private.getParam("alpha",asap_params.alpha);
    nh_private.getParam("tracker_name",asap_params.tracker_name);
    nh_private.getParam("target_name",asap_params.target_name);
    nh_private.getParam("replanning_trigger_error",asap_params.replanning_trigger_error);


    printf("Parameters summary: \n");
    printf("------------------------------\n");
    printf("------------------------------\n");

    printf("sampling distance: [ ");
    for(int i=0;i<asap_params.tracking_ds.size();i++)
        printf("%f ",asap_params.tracking_ds[i]);
    printf(" ]\n");

    printf("azimuth range: [%f, %f] N: %d / elevation: [%f, %f] N: %d \n",asap_params.azim_min,asap_params.azim_max,
    asap_params.N_azim,asap_params.elev_min,asap_params.elev_max,asap_params.N_elev);

    printf("local maxima search in visibility matrix : search window = %d / N_extrema = %d\n",asap_params.local_range,asap_params.N_extrem);
    printf("max interval disance: %f\n",asap_params.max_interval_distance);
    printf("nominal weight for visibility in edge connection: %f \n",asap_params.w_v0);

    std::cout<<"tracker name: "<<asap_params.tracker_name<<" target_name: "<<asap_params.target_name<<std::endl;

    ASAP asap_obj(asap_params);

    // for record
    nav_msgs::Path target_traj_record; target_traj_record.header.frame_id=asap_obj.world_frame_id;
    nav_msgs::Path tracker_traj_record; tracker_traj_record.header.frame_id=asap_obj.world_frame_id;
    visualization_msgs::MarkerArray bearing_vector_list;

    ros::Publisher target_traj_record_pub=nh_private.advertise<nav_msgs::Path>("record_target",2);
    ros::Publisher tracker_traj_record_pub=nh_private.advertise<nav_msgs::Path>("record_tracker",2);
    ros::Publisher bearing_vector_list_pub=nh_private.advertise<visualization_msgs::MarkerArray>("record_bearing_vector",2);
    double record_rate=0.4; //sec


    ROS_INFO("Always See and Picturing started");
    ros::Rate rate(30);

    // initialize
    ros::Duration(1.0).sleep();

    // planning check point
    ros::Time planning_ckp=ros::Time::now();
    ros::Time record_ckp=ros::Time::now();


    asap_obj.hovering(ros::Duration(1.0),double(1));


    // checkpoint for calculation of velocity
    ros::Time time_ckp=ros::Time::now();
    geometry_msgs::Point tracker_position_ckp=asap_obj.cur_tracker_pos;

    // main loop
    while(ros::ok()){
        if (asap_obj.octomap_callback_flag && asap_obj.state_callback_flag)
        {



            // planning once this condition is satisfied
            if(asap_obj.prediction_error>=asap_params.replanning_trigger_error
               or ((ros::Time::now()-planning_ckp).toSec()>asap_params.t_pred*0.7)) {


                // planning loop

                auto t0 = std::chrono::high_resolution_clock::now();



                asap_obj.target_regression(); // get regression model from history
                asap_obj.target_future_prediction();

                bool isSolved=false;
                double d_max=asap_params.max_interval_distance;
                double d_max0=asap_params.max_interval_distance_init;

                while(not isSolved) {
                    isSolved = asap_obj.reactive_planning(d_max,d_max0);
                    if (not isSolved){
                        ROS_WARN("try again with extension of d_max");
                        asap_obj.target_regression(); // get regression model from history
                        asap_obj.target_future_prediction();
                        d_max+=0.2; d_max0+=0.2;
                    }
                }

                planning_ckp = ros::Time::now();
                asap_obj.smooth_path_update();

                auto t1 = std::chrono::high_resolution_clock::now();
                auto dt = 1.e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count();
//                ROS_INFO_STREAM("replanning routine : "<<dt);

            }

            asap_obj.quad_waypoint_pub();
            asap_obj.path_publish(); // skeleton + smooth path publish
            asap_obj.marker_publish();
            asap_obj.points_publish();
        }


        // record

        if((ros::Time::now()-record_ckp).toSec()>record_rate){

            record_ckp=ros::Time::now();
            asap_obj.record(target_traj_record,tracker_traj_record,bearing_vector_list);

        }

        target_traj_record_pub.publish(target_traj_record);
        tracker_traj_record_pub.publish(tracker_traj_record);
        bearing_vector_list_pub.publish(bearing_vector_list);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}
