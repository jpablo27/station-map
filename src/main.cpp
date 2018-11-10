#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "st_dr.h"
#include "def_func.h"


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // Variables
    bool OdomMSG_ = false;

    int nc=0;
    int nc_1=0;
    int Dnc=0;

    float resolution = 0.1;

    std::vector<std::string> cloud_names; //cloud_names.resize(0);


    std::vector<double> avg_r;
    std::vector<double> avg_g;
    std::vector<double> avg_b;

    std::vector<int> largest_clouds;

    float r_ply;
    float g_ply;
    float b_ply;


    int map_c_counter;
    std::vector<std::string> map_names;

    bool flag_pause = false;


    geometry_msgs::Point Pos_ros_msg;
    geometry_msgs::Quaternion Qua_ros_msg;

	Eigen::Matrix4f posp;
    Eigen::Matrix3f mat3;
    Eigen::Affine3f matAff;


    pcl::PCLPointCloud2 sHull2;
    std::vector<std::vector<pcl::Vertices>> all_polygons;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> all_hulls;

    //Pose Variables and Rigid Transformations
    double Xd,Yd,Zd,qx,qy,qz,qw;

    char name_ch[50];
    bool first_o=false;
    bool first_p=false;

    std::vector<float> errors;
    std::vector<station_map::c_msgs> centroids;
    std::vector<int> num_po;

    bool yaHay=false;
    bool yaHay2=false;

void keyboard_cbs(const pcl::visualization::KeyboardEvent &event, void* junk);

void stcb(const station_map::drone_MSG& msg){
    first_o=true;
    nc_1=nc;
    nc=msg.drone_hulls.size();


    Dnc =nc-nc_1;

    for(int i=0; i<nc; i++){


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
        sHull2.data.clear();

        pcl_conversions::toPCL(msg.drone_hulls[i],sHull2);
        pcl::fromPCLPointCloud2(sHull2,*temp_hull);

        all_hulls.push_back(temp_hull);

        std::vector<pcl::Vertices> poly_temp;
        station_map::drone_ver dv_temp;

        dv_temp = msg.v_hulls[i];

        pcl_conversions::toPCL(dv_temp.v_hulls,poly_temp);

        all_polygons.push_back(poly_temp);

    }

    qw = msg.drone_pose.orientation.w;
    qx = msg.drone_pose.orientation.x;
    qy = msg.drone_pose.orientation.y;
    qz = msg.drone_pose.orientation.z;

    Xd = msg.drone_pose.position.x;
    Yd = msg.drone_pose.position.y;
    Zd = msg.drone_pose.position.z;

    avg_r = msg.hulls_colors.red;
    avg_g = msg.hulls_colors.green;
    avg_b = msg.hulls_colors.blue;

    largest_clouds=msg.largest_hulls;

    yaHay = true;
}

void odomCB(const nav_msgs::Odometry& data){

    OdomMSG_ = true;
    Pos_ros_msg = data.pose.pose.position;
    Qua_ros_msg = data.pose.pose.orientation;


    mat3 = Eigen::Quaternionf(Qua_ros_msg.w,
        Qua_ros_msg.x,
        Qua_ros_msg.y,
        Qua_ros_msg.z).toRotationMatrix();

    posp.block(0,0,3,3) = mat3;
    matAff.matrix() = posp;


    matAff(0,3) = data.pose.pose.position.x;
    matAff(1,3) = data.pose.pose.position.y;
    matAff(2,3) = data.pose.pose.position.z;

}

void ercb(const station_map::e_msgs& msg){
    first_p = true;
    errors.clear();
    errors = msg.errors;
    centroids.clear();
    centroids = msg.centroids;
    num_po.clear();
    num_po = msg.number_op;
    yaHay2 = true;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // Open 3D viewer and add point cloud
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("GROUND STATION VIEWER"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();
    viewer->setCameraPosition(-6.41523, 0.223382, 2.25379,3.21672, 5.06064, 1.83923,0.090303, -0.094849, 0.991387,0);

    return (viewer);
}


int main(int argc,char** argv){

	ROS_INFO("LISTENING TO THE DRONE");


	ros::init(argc,argv,"st_map_node_");
	ros::NodeHandle nh_;

	ros::Subscriber st_sub_=nh_.subscribe("/UAV/hull",1,stcb);
    ros::Subscriber er_sub =nh_.subscribe("/UAV/errors",1,ercb);
    ros::Subscriber odomS_ =nh_.subscribe("/zed/odom",1,odomCB);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummy_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	viewer = rgbVis(dummy_cloud);

	viewer->registerKeyboardCallback(keyboard_cbs);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ob_camera(new pcl::PointCloud<pcl::PointXYZRGB>);
	draw_camera(ob_camera);

	std::vector<pcl::Vertices> polygons_cam;


    polygons_cam.resize(1);

    for (int i=0; i<16;i++){
        polygons_cam[0].vertices.push_back(i);
    }

    viewer->addPointCloud<pcl::PointXYZRGB>(ob_camera, "camera");
    viewer->updatePolygonMesh<pcl::PointXYZRGB>(ob_camera, polygons_cam, "camera");
    viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,0.5,"camera");

    nc_1=0;
    posp.setIdentity();
    matAff.setIdentity();

    mat3.setIdentity();

    map_c_counter=0;
    map_names.resize(0);
    largest_clouds.resize(0);

    cv::Mat coltest = cv::Mat(1,1,CV_8UC3);

    ros::Rate loop_rate(10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr P_map(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointXYZ searchPoint;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//buscador aarbol
    float radius = 1.0f;

    bool kdfirts = true;


    while(ros::ok()){

        viewer->spinOnce(1);
        ros::spinOnce();
        loop_rate.sleep();

        viewer->updatePointCloudPose("camera", matAff);

        if(all_hulls.size()!=largest_clouds.size()){
            //No entrar
            yaHay = false;
            yaHay2 = false;
        }     
        if(yaHay&&yaHay2){

            if(kdfirts){//Inicializacion del mapa
                init_planes(searchPoint,centroids,resolution,P_map,name_ch,viewer,all_hulls,coltest,avg_r, avg_g, avg_b, largest_clouds,r_ply, g_ply, b_ply,all_polygons,errors,num_po);
                // Flag to avoid entering here again
                if(P_map->points.size() > 0){
                    kdfirts = false;
                }
            }else{// Main mapping 
                            //Buscar dentro de este mapa
                kdtree.setInputCloud (P_map);
                map_c_counter = P_map->points.size();
                new_q_planes(searchPoint,centroids,resolution,P_map,name_ch,viewer,all_hulls,coltest,avg_r, avg_g, avg_b, largest_clouds,r_ply, g_ply, b_ply,all_polygons,map_c_counter,kdtree,radius,errors,num_po);

            }

        }

        yaHay = false;
        yaHay2 = false;

        all_hulls.clear();
        avg_r.clear();
        avg_g.clear();
        avg_b.clear();
        largest_clouds.clear();
        all_polygons.clear();
    }


	return 0;
}


void keyboard_cbs(const pcl::visualization::KeyboardEvent &event, void* junk){
    if (event.getKeySym() == "q"){

        std::cout << "FINISHED LISTENING" <<std::endl;
        ros::shutdown();
    }



    if (event.getKeySym() == "n" && event.keyDown()){
        cout<<"Clear"<<endl;
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        map_c_counter=0;
        map_names.clear();
        cloud_names.clear();

        nc_1=0;
        nc=0;
    }

    if (event.getKeySym() == "b" && event.keyDown()){
        flag_pause = !flag_pause;
        std::cout << "Pause?: " << flag_pause << std::endl;
        nc=0;
        Dnc=0;
        nc_1=0;
    }


}
