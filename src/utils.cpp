//
// Created by jbs on 18. 6. 29.
//


#include "utils.h"

Point3f sph2R3(Point3f source,float r,float azim,float elev){

    float X= source.x+r*cos(elev)*cos(azim);
    float Y= source.y+r*cos(elev)*sin(azim);
    float Z= source.z+r*sin(elev);
    return Point3f(X,Y,Z);
};


MatrixXd SEDT(MatrixXd binaryMatrix){

    int N_elev=binaryMatrix.rows();
    int N_azim=binaryMatrix.cols();

    Mat bw_cast(N_azim,N_elev,CV_64F,binaryMatrix.data());
    bw_cast.convertTo(bw_cast,CV_8UC(1));
    transpose(bw_cast,bw_cast);

//	ROS_INFO("cols of bw_cast: %d",bw_cast.cols);

    Mat concat_bw_cast;

    hconcat(bw_cast,bw_cast,concat_bw_cast);
    hconcat(bw_cast,concat_bw_cast,bw_cast);

//	ROS_INFO("cols of merged bw_cast: %d",bw_cast.cols);

    // calculate SEDT
    cv::Mat dist,dist1,dist2;
    // in the cv pack 1= white... 0=black... fuck!
    distanceTransform(1-bw_cast, dist1, CV_DIST_L2, 0);
    vector<Vec4i> hierarchy;
    vector<vector<Point> > contours;
    findContours(bw_cast,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,Point(0, 0));
    for( int i = 0; i< contours.size(); i++ ) // for each cluster
        for (int j=0;j<contours[i].size();j++)
            bw_cast.at<uchar>(contours[i][j].y,contours[i][j].x)=0;

    distanceTransform(bw_cast, dist2, CV_DIST_L2, 0);
    dist=dist1-dist2;

    cv::Mat sliced_dist=dist.colRange(N_azim+1,2*N_azim);

    MatrixXd mat;
    cv2eigen(sliced_dist,mat);

    return mat;
}

IDX maxElem(const MatrixXd& mat){
    // this function find max elements in matrix and return the (row,col) index
    int num_row=mat.rows();
    int num_col=mat.cols();
    IDX max_index;
    double max_val=numeric_limits<double >::min();
    int max_row=0,max_col=0;
	// we apply negative rejection 
    for(long i=0;i<num_row;i++)
        for(long j=0;j<num_col;j++)
            if (mat.coeff(i,j)>max_val && mat.coeff(i,j)>0) {
                max_row = i, max_col = j;
                max_val=mat.coeff(i,j);
            }

    max_index[0]=max_row;
    max_index[1]=max_col;

    return max_index;
}



// this function finds numExt many extrema(local max) in matrix
vector<IDX> localMaxima(const MatrixXd& mat,int numExt,int range){

    vector<IDX> local_extrema;


    MatrixXd temp_mat = mat.replicate(1, 1);
    int num_row = mat.rows();
    int num_col = mat.cols();

    for (int k = 0; k < numExt; k++) {

        IDX max_idx = maxElem(temp_mat);
        // save this extrema
        local_extrema.push_back(max_idx);

        // we make element in the window zeros
        int row_min = std::max(max_idx[0] - range, 0);
        int row_max = std::min(max_idx[0] + range, num_row - 1);

        int col_min = std::max(max_idx[1] - range, 0);
        int col_max = std::min(max_idx[1] + range, num_col - 1);

        //iterate through the window and set zeros
        for (int r = row_min; r <= row_max; r++)
            for (int c = col_min; c <= col_max; c++)
                temp_mat.coeffRef(r, c) = 0;
    }

    return local_extrema;

}

LinearModel linear_regression(const Eigen::VectorXd& ts,const Eigen::VectorXd& xs){
    // 1st order regression on two set of vectors

    double SS_xy=xs.dot(ts)-xs.sum()*ts.sum()/ts.size();
    double SS_xx=ts.dot(ts)-pow(ts.sum(),2)/ts.size();

    double beta1=SS_xy/SS_xx;
    double beta0;

    beta1=SS_xy/SS_xx;
    beta0=xs.mean()-beta1*ts.mean();
    LinearModel model;
    model.beta0=beta0;
    model.beta1=beta1;
    return model;
}

double model_eval(const LinearModel& model,double t){
    return model.beta0+model.beta1*t;
}


double interpolate( vector<double> &xData, vector<double> &yData, double x, bool extrapolate )
{
    int size = xData.size();

    int i = 0;                                                                  // find left end of interval for interpolation
    if ( x >= xData[size - 2] )                                                 // special case: beyond right end
    {
        i = size - 2;
    }
    else
    {
        while ( x > xData[i+1] ) i++;
    }
    double xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];      // points on either side (unless beyond ends)
    if ( !extrapolate )                                                         // if beyond ends of array and not extrapolating
    {
        if ( x < xL ) yR = yL;
        if ( x > xR ) yL = yR;
    }

    double dydx = ( yR - yL ) / ( xR - xL );                                    // gradient

    return yL + dydx * ( x - xL );                                              // linear interpolation
}


void path2vec(const nav_msgs::Path& path,std::vector<double> &xs,std::vector<double> &ys,std::vector<double> &zs){

    unsigned long path_len=path.poses.size();

    xs.resize(path_len);
    ys.resize(path_len);
    zs.resize(path_len);

    for (int i=0;i<path_len;i++){
        xs[i]=path.poses[i].pose.position.x;
        ys[i]=path.poses[i].pose.position.y;
        zs[i]=path.poses[i].pose.position.z;
    }


};


vector<IDX> equal_dist_idx_set(int row,int col,int row_sample,int col_sample){
//
//    int y_dist=ceil(sqrt(float(row)/float(col)*N_extrema));
//    int x_dist=ceil(sqrt(float(row)/float(col)*N_extrema));

    VectorXf azim_vec,elev_vec;

    azim_vec.setLinSpaced(col_sample+2,0,col-1);
    elev_vec.setLinSpaced(row_sample+2,0,row-1);

    VectorXi azim_veci(col_sample),elev_veci(row_sample);

    for(int i=1;i<=azim_vec.rows()-2;i++)
        azim_veci.coeffRef(i-1)=azim_vec.coeff(i);

    for(int i=1;i<=elev_vec.rows()-2;i++)
        elev_veci.coeffRef(i-1)=elev_vec.coeff(i);

    vector<IDX> idx_set;

    for(int r=0;r<row_sample;r++)
        for(int c=0;c<col_sample;c++) {
            IDX idx;
            idx(0)=elev_veci.coeff(r);
            idx(1)=azim_veci.coeff(c);
            idx_set.push_back(idx);
        }

    return idx_set;
}

GraphPath Dijkstra(Graph g,Vertex v0,Vertex vf){

    // Create things for Dijkstra
    std::vector<Vertex> predecessors(boost::num_vertices(g)); // To store parents
    std::vector<Weight> distances(boost::num_vertices(g)); // To store distances

    IndexMap indexMap = boost::get(boost::vertex_index, g);
    PredecessorMap predecessorMap(&predecessors[0], indexMap);
    DistanceMap distanceMap(&distances[0], indexMap);

    // Compute shortest paths from v0 to all vertices, and store the output in predecessors and distances
    // boost::dijkstra_shortest_paths(g, v0, boost::predecessor_map(predecessorMap).distance_map(distanceMap));
    // This is exactly the same as the above line - it is the idea of "named parameters" - you can pass the
    // prdecessor map and the distance map in any order.
    boost::dijkstra_shortest_paths(g, v0, boost::distance_map(distanceMap).predecessor_map(predecessorMap));

    // Output results
//    std::cout << "distances and parents:" << std::endl;
    NameMap nameMap = boost::get(boost::vertex_name, g);
//
//    BGL_FORALL_VERTICES(v, g, Graph)
//        {
//            std::cout << "distance(" << nameMap[v0] << ", " << nameMap[v] << ") = " << distanceMap[v] << ", ";
//            std::cout << "predecessor(" << nameMap[v] << ") = " << nameMap[predecessorMap[v]] << std::endl;
//        }
//
//    // Extract a shortest path
//    std::cout << std::endl;

    typedef std::vector<Graph::edge_descriptor> PathType;

    PathType path;
    Vertex v = vf; // We want to start at the destination and work our way back to the source
    for(Vertex u = predecessorMap[v]; // Start by setting 'u' to the destintaion node's predecessor
        u != v; // Keep tracking the path until we get to the source
        v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
    {
        std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, v, g);
        Graph::edge_descriptor edge = edgePair.first;

        path.push_back( edge );
    }


    if (path.size())
    {
//        ROS_INFO("path exist");
    // Write shortest path
//    std::cout << "Shortest path from v0 to v3:" << std::endl;
        float totalDistance = 0;

        GraphPath vertex_path1;
        GraphPath vertex_path2;
        for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
        {

//            ROS_INFO("path insertion");
            vertex_path1.push_back(nameMap[boost::source(*pathIterator, g)]);
            vertex_path2.push_back(nameMap[boost::target(*pathIterator, g)]);

    //        std::cout << nameMap[boost::source(*pathIterator, g)] << " -> " << nameMap[boost::target(*pathIterator, g)]
    //                  << " = " << boost::get( boost::edge_weight, g, *pathIterator ) << std::endl;
        }

        vertex_path1.push_back(vertex_path2.back());
        return vertex_path1;
    }
    else{
        ROS_WARN("path does not exist");
        return GraphPath();
    }
}


void mat_normalize(MatrixXd& mat){
    double max_value=mat.maxCoeff();

    int num_row=mat.rows();
    int num_col=mat.cols();

    for(long i=0;i<num_row;i++)
        for(long j=0;j<num_col;j++)
            mat(i,j)/=max_value;

}

bool compare_visibility(asap_ns::CandidNode node1,asap_ns::CandidNode node2){

    return (node1.visibility<node2.visibility);

}








