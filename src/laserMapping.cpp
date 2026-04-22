// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;

bool   leapslam_runtime_log_en = false;
/**************************/

float res_last[100000] = {0.0};
float res_last_reflective[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   point_selected_surf_reflective[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
int lidar_type;
int feats_reflective_size = 0;
int effct_feat_num_reflective = 0;
double eigen_thres = 100;
double intensity_thres = 200;
int h_weight = 10;
double test = 0;
double filter_size_reflective = 0;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<PointVector>  Nearest_Points_reflective; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr featsFromMap_reflective(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_reflective_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_reflective_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_reflective_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr normvec_reflective(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri_reflective(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect_reflective(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterReflective;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;
KD_TREE<PointType> ikdtree_reflective;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

double basic_motion_length = 10.0;
double reflective_beacon_diameter = 0.11;
double reflective_beacon_height = 0.30;
double car_ball_radius = 0.6;


void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);

    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();


        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }
        if(lidar_type == MARSIM)
            lidar_end_time = meas.lidar_beg_time;

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            // 体素滤波后的占据的中点
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            // 体素滤波后的点到原来的点的距离
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            // 如果临近点在体素块内,则该点无序降采样
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && 
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && 
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) 
                PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{

    // std::cout << "stepped into publish_frame_world" << std::endl;
    if(scan_pub_en)
    {
        // std::cout << "1 stepped into scan_pub_en" << std::endl;

        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}
void publish_map_reflective(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap_reflective, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

string now_lidar_time;
string degen_filename = string(string(ROOT_DIR) + "DEGEN/lidar_degeneracy.txt");
string degen_filename2 = string(string(ROOT_DIR) + "DEGEN/lidar_degeneracy_2.txt");

std::ofstream degen_file(degen_filename);
std::ofstream degen_file2(degen_filename2);

// 遍历所有点，并计算其点面距离和雅可比矩阵
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + 
                        pabcd(1) * point_world.y + 
                        pabcd(2) * point_world.z + 
                        pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    
    // ==================reflective points=================
    // cjs on 20250627
    laserCloudOri_reflective->clear(); 
    corr_normvect_reflective->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_reflective_size; i++)
    {
        PointType &point_body  = feats_reflective_body->points[i]; 
        PointType &point_world = feats_reflective_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points_reflective[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            // ikdtree_reflective.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf_reflective[i] = points_near.size() < NUM_MATCH_POINTS ? 
                                                false : 
                                                pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf_reflective[i]) continue;

        // 
        VF(4) pabcd;    // 平面法向量
        point_selected_surf_reflective[i] = false;  // 确认点是否在平面上
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            // 点面距离
            float pd2 = pabcd(0) * point_world.x + 
                        pabcd(1) * point_world.y + 
                        pabcd(2) * point_world.z + 
                        pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm()); 

            if (s > 0.5)
            {
                point_selected_surf_reflective[i] = true;
                normvec_reflective->points[i].x = pabcd(0);
                normvec_reflective->points[i].y = pabcd(1);
                normvec_reflective->points[i].z = pabcd(2);
                normvec_reflective->points[i].intensity = pd2;
                res_last_reflective[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num_reflective = 0;

    for (int i = 0; i < feats_reflective_size; i++)
    {
        if (point_selected_surf_reflective[i])
        {
            laserCloudOri_reflective->points[effct_feat_num_reflective] = feats_reflective_body->points[i];
            corr_normvect_reflective->points[effct_feat_num_reflective] = normvec_reflective->points[i];
            total_residual += res_last_reflective[i];
            effct_feat_num_reflective ++;
        }
    }

    if (effct_feat_num_reflective < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Reflective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num_reflective;
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    // int weight = 0;


    int weight = h_weight;
    int effct_num_total = effct_feat_num + effct_feat_num_reflective * weight;

    Eigen::MatrixXd hx_origin = MatrixXd::Zero(effct_feat_num, 12);

    ekfom_data.h_x = MatrixXd::Zero(effct_num_total, 12); //23
    ekfom_data.h.resize(effct_num_total);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x,
                                                norm_p.y,
                                                norm_p.z,
                                                VEC_FROM_ARRAY(A),
                                                VEC_FROM_ARRAY(B),
                                                VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, 
                                                norm_p.y, 
                                                norm_p.z, 
                                                VEC_FROM_ARRAY(A), 
                                                0.0, 
                                                0.0, 
                                                0.0, 
                                                0.0, 
                                                0.0, 
                                                0.0;
            hx_origin.row(i) = ekfom_data.h_x.row(i);
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }

    Eigen::MatrixXd hxThx_ = ekfom_data.h_x.transpose() * ekfom_data.h_x;    
    
    // Eigen::Matrix3d mat_hxThx_transl = h_x_transl.transpose() * h_x_transl;
    Eigen::Matrix3d mat_hxThx_transl_ = hxThx_. template block<3,3>(0, 0);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_transl_(mat_hxThx_transl_);
    Eigen::Vector3d eigen_values_transl_ = es_transl_.eigenvalues();

    // 平移信息特征值分析
    const double min_eigen_transl_ = eigen_values_transl_.minCoeff();
    const double mid_eigen_transl_ = eigen_values_transl_[1];
    const double max_eigen_transl_ = eigen_values_transl_.maxCoeff();

    double degen_t_ = min_eigen_transl_ / (mid_eigen_transl_ + max_eigen_transl_ + min_eigen_transl_);

    // std::cout << "111 effct_feat_num: " << effct_feat_num << std::endl;
    // std::cout << "111 Translation Eigen values:" << std::endl << eigen_values_transl_ << std::endl;
    // std::cout << "111 Translation Eigen vectors: " << std::endl << es_transl_.eigenvectors() << std::endl;
    // // 输出 degen-t
    // std::cout << "111 degen-t: " << degen_t_ << std::endl;

    // // 对旋转信息（3-5列）进行退化分析
    Eigen::MatrixXd h_x_rot_ = ekfom_data.h_x.middleCols(3, 3);  // 从第3列开始，取3列
    Eigen::Matrix3d mat_hxThx_rot_ = h_x_rot_.transpose() * h_x_rot_;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_rot_(mat_hxThx_rot_);
    Eigen::Vector3d eigen_values_rot_ = es_rot_.eigenvalues();

    // // 旋转信息特征值分析
    // const double min_eigen_rot_ = eigen_values_rot_.minCoeff();
    // const double mid_eigen_rot_ = eigen_values_rot_[1];
    // const double max_eigen_rot_ = eigen_values_rot_.maxCoeff();

    // double degen_r_ = min_eigen_rot_ / (mid_eigen_rot_ + max_eigen_rot_ + min_eigen_rot_);
    // double degen_r_ = 1 / sqrt(min_eigen_rot_); // 旋转退化程度，特征值越小，退化程度越高

    // std::cout << "111 Rotation Eigen values:" << std::endl << eigen_values_rot_ << std::endl;   
    // std::cout << "111 Rotation Eigen vectors: " << std::endl << es_rot_.eigenvectors() << std::endl;

    // degen_file << now_lidar_time << " " << degen_t_ << " " << degen_r_ << " " << min_eigen_transl_ << " " << mid_eigen_transl_ << " " << max_eigen_transl_ << std::endl;


    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    // =================== for refelective ============================

    // 如果场景退化（degen_t 小于 阈值0.4），才引入高可定位点
    // if (degen_t_ > test)
    // {
    //     ekfom_data.h.conservativeResize(effct_feat_num);
    //     ekfom_data.h_x = ekfom_data.h_x.topRows(effct_feat_num);
    // }
    // else
    // {
    for (int w = 0; w < weight; w ++)
    {
        for (int i = 0; i < effct_feat_num_reflective; i++)
        {
            // std::cout << "Total: " << effct_num_total << std::endl;
            // std::cout << "Normal points: " << effct_feat_num << std::endl;
            int j = i + effct_feat_num + effct_feat_num_reflective * w;
            // std::cout << "[j]" << j << std::endl;
            const PointType &laser_p = laserCloudOri_reflective->points[i];
            V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            M3D point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            M3D point_crossmat;
            point_crossmat<<SKEW_SYM_MATRX(point_this);

            /*** get the normal vector of closest surface/corner ***/
            const PointType &norm_p = corr_normvect_reflective->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            /*** calculate the Measuremnt Jacobian matrix H ***/
            V3D C(s.rot.conjugate() *norm_vec);
            V3D A(point_crossmat * C);
            if (extrinsic_est_en)
            {
                V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
                ekfom_data.h_x.block<1, 12>(j,0) << norm_p.x,
                                                    norm_p.y, 
                                                    norm_p.z, 
                                                    VEC_FROM_ARRAY(A), 
                                                    VEC_FROM_ARRAY(B), 
                                                    VEC_FROM_ARRAY(C);
            }
            else
            {
                ekfom_data.h_x.block<1, 12>(j,0) << norm_p.x, 
                                                    norm_p.y, 
                                                    norm_p.z, 
                                                    VEC_FROM_ARRAY(A), 
                                                    0.0, 
                                                    0.0, 
                                                    0.0, 
                                                    0.0, 
                                                    0.0, 
                                                    0.0;
            }
            
            /*** Measuremnt: distance to the closest surface/corner ***/
            ekfom_data.h(j) = -norm_p.intensity;

            // std::cout << "[j]" << j << " h: " << ekfom_data.h(j) << std::endl;
        }
    }

    // ==============================================================================================================
    Eigen::MatrixXd hxThx = ekfom_data.h_x.transpose() * ekfom_data.h_x;

    
    Eigen::MatrixXd h_x_transl = ekfom_data.h_x.leftCols(3);
    // Eigen::Matrix3d mat_hxThx_transl = h_x_transl.transpose() * h_x_transl;
    Eigen::Matrix3d mat_hxThx_transl = hxThx. template block<3,3>(0, 0);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_transl(mat_hxThx_transl);
    Eigen::Vector3d eigen_values_transl = es_transl.eigenvalues();

    // 平移信息特征值分析
    const double min_eigen_transl = eigen_values_transl.minCoeff();
    const double mid_eigen_transl = eigen_values_transl[1];
    const double max_eigen_transl = eigen_values_transl.maxCoeff();

    // double degen_t = min_eigen_transl / (mid_eigen_transl + max_eigen_transl + min_eigen_transl);
    double degen_t = 1 / sqrt(min_eigen_transl); // 平移退化程度，特征值越小，退化程度越高

    // std::cout << "222 Translation Eigen values:" << std::endl << eigen_values_transl << std::endl;
    // std::cout << "222 Translation Eigen vectors: " << std::endl << es_transl.eigenvectors() << std::endl;
    // 输出 degen-t
    // std::cout << "222 degen-t: " << degen_t << std::endl;

    // 对旋转信息（3-5列）进行退化分析
    Eigen::MatrixXd h_x_rot = hx_origin.middleCols(3, 3);  // 从第3列开始，取3列
    Eigen::Matrix3d mat_hxThx_rot = h_x_rot.transpose() * h_x_rot;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_rot(mat_hxThx_rot);
    Eigen::Vector3d eigen_values_rot = es_rot.eigenvalues();

    // // 旋转信息特征值分析
    const double min_eigen_rot = eigen_values_rot.minCoeff();
    const double mid_eigen_rot = eigen_values_rot[1];
    const double max_eigen_rot = eigen_values_rot.maxCoeff();

    double degen_r = 1 / sqrt(min_eigen_rot);

    // std::cout << "222 Rotation Eigen values:" << std::endl << eigen_values_rot << std::endl;
    // std::cout << "222 Rotation Eigen vectors: " << std::endl << es_rot.eigenvectors() << std::endl;


    // save to file

    // degen_file2 << now_lidar_time << " " << degen_t << " " << degen_r << " " << min_eigen_transl << " " << mid_eigen_transl << " " << max_eigen_transl << " " << min_eigen_rot << " " << mid_eigen_rot << " " << max_eigen_rot << std::endl;


    // eigen_thres = 10;
    // 
    // if (mid_eigenvalue / min_eigenvalue > eigen_thres)
    // {
    //     std::cout << "退化" << std::endl;
    //     // 3. 挑选最小特征值对应的特征向量作为退化方向
    //     Eigen::Vector3d eigen_vector = es.eigenvectors().col(0);


    //     // 4. 遍历所有点的法向量，对比以得到各个法向量的贡献度
    //     Eigen::Vector3d h_x_this;
    //     double cos_theta;
    //     std::vector<int> index_delete;
    //     // 仅遍历非反光点
    //     // 保留远离90度的点
    //     for (int i = 0; i < effct_feat_num; i++)
    //     {
    //         h_x_this = h_x_tranl.row(i);
    //         // cos_theta 从 -1 到 1
    //         cos_theta = h_x_this.dot(eigen_vector) / (h_x_this.norm() * eigen_vector.norm());
    //         const double cos_theta_thres = 30 / 180 * M_PI;
    //         if (std::abs(cos_theta) > cos_theta_thres)
    //         {
    //             index_delete.push_back(i);
    //         }
    //     }

    //     // 5. 贡献度小于某一程度则直接删除该行
    //     // 保留贡献度大于阈值的点
    //     // 逆向遍历升序索引（从后往前）
    //     for (int i = index_delete.size() - 1; i >= 0; i--)
    //     {
    //         int index = index_delete[i];
    //         // 确保索引在当前有效范围内（避免越界）
    //         if (index >= effct_num_total) continue;
    //         ekfom_data.h_x.row(index) = ekfom_data.h_x.row(effct_num_total - 1);
    //         ekfom_data.h(index) = ekfom_data.h(effct_num_total - 1);
    //         effct_num_total--;
    //     }

    //     // 6. 删除贡献度小的后得到最终的h与h_x
    //     // 只保留非反光点
    //     ekfom_data.h_x = ekfom_data.h_x.topRows(effct_num_total);
    //     ekfom_data.h = ekfom_data.h.topRows(effct_num_total);
    // }


    solve_time += omp_get_wtime() - solve_start_;
}


// void h_share_model_reflective(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
// {
//     laserCloudOri->clear(); 
//     corr_normvect->clear(); 
//     total_residual = 0.0; 

//     /** closest surface search and residual computation **/
//     #ifdef MP_EN
//         omp_set_num_threads(MP_PROC_NUM);
//         #pragma omp parallel for
//     #endif
//     for (int i = 0; i < feats_reflective_size; i++)
//     {
//         PointType &point_body  = feats_reflective_body->points[i]; 
//         PointType &point_world = feats_reflective_world->points[i]; 

//         /* transform to world frame */
//         V3D p_body(point_body.x, point_body.y, point_body.z);
//         V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
//         point_world.x = p_global(0);
//         point_world.y = p_global(1);
//         point_world.z = p_global(2);
//         point_world.intensity = point_body.intensity;

//         vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

//         auto &points_near = Nearest_Points_reflective[i];

//         if (ekfom_data.converge)
//         {
//             /** Find the closest surfaces in the map **/
//             ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
//             point_selected_surf_reflective[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
//         }

//         if (!point_selected_surf_reflective[i]) continue;

//         VF(4) pabcd;
//         point_selected_surf_reflective[i] = false;
//         if (esti_plane(pabcd, points_near, 0.1f))
//         {
//             float pd2 = pabcd(0) * point_world.x + 
//                         pabcd(1) * point_world.y + 
//                         pabcd(2) * point_world.z + 
//                         pabcd(3);
//             float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

//             if (s > 0.9)
//             {
//                 point_selected_surf_reflective[i] = true;
//                 normvec_reflective->points[i].x = pabcd(0);
//                 normvec_reflective->points[i].y = pabcd(1);
//                 normvec_reflective->points[i].z = pabcd(2);
//                 normvec_reflective->points[i].intensity = pd2;
//                 res_last_reflective[i] = abs(pd2);
//             }
//         }
//     }
    
//     effct_feat_num_reflective = 0;

//     for (int i = 0; i < feats_reflective_size; i++)
//     {
//         if (point_selected_surf_reflective[i])
//         {
//             laserCloudOri->points[effct_feat_num_reflective] = feats_down_body->points[i];
//             corr_normvect->points[effct_feat_num_reflective] = normvec_reflective->points[i];
//             total_residual += res_last_reflective[i];
//             effct_feat_num_reflective ++;
//         }
//     }

//     if (effct_feat_num_reflective < 1)
//     {
//         ekfom_data.valid = false;
//         ROS_WARN("No Effective Points! \n");
//         return;
//     }

//     res_mean_last = total_residual / effct_feat_num_reflective;
    
//     /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
//     ekfom_data.h_x = MatrixXd::Zero(effct_feat_num_reflective, 12); //23
//     ekfom_data.h.resize(effct_feat_num_reflective);

//     for (int i = 0; i < effct_feat_num_reflective; i++)
//     {
//         const PointType &laser_p  = laserCloudOri->points[i];
//         V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
//         M3D point_be_crossmat;
//         point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
//         V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
//         M3D point_crossmat;
//         point_crossmat<<SKEW_SYM_MATRX(point_this);

//         /*** get the normal vector of closest surface/corner ***/
//         const PointType &norm_p = corr_normvect->points[i];
//         V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

//         /*** calculate the Measuremnt Jacobian matrix H ***/
//         V3D C(s.rot.conjugate() *norm_vec);
//         V3D A(point_crossmat * C);
//         if (extrinsic_est_en)
//         {
//             V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
//             ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x,
//                                                 norm_p.y, 
//                                                 norm_p.z, 
//                                                 VEC_FROM_ARRAY(A), 
//                                                 VEC_FROM_ARRAY(B), 
//                                                 VEC_FROM_ARRAY(C);
//         }
//         else
//         {
//             ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, 
//                                                 norm_p.y, 
//                                                 norm_p.z, 
//                                                 VEC_FROM_ARRAY(A), 
//                                                 0.0, 
//                                                 0.0, 
//                                                 0.0, 
//                                                 0.0, 
//                                                 0.0, 
//                                                 0.0;
//         }

//         /*** Measuremnt: distance to the closest surface/corner ***/
//         ekfom_data.h(i) = -norm_p.intensity;
//     }
// }


bool stopFlagA = false;
bool lastStopFlagA = false;
int stabilityCountA = 0;        // 动态窗口，提高稳定性
void stop_flag_cbk_a(const nav_msgs::Odometry::ConstPtr& odom_msg) 
{
    // 定义速度阈值，可根据实际情况调整
    const double linear_threshold = 0.03;
    const double angular_threshold = 0.01;

    // 获取线速度和角速度
    double linear_vel = odom_msg->twist.twist.linear.x;
    double angular_vel = odom_msg->twist.twist.angular.z;

    // 判断小车是否静止
    if (std::abs(linear_vel) < linear_threshold && std::abs(angular_vel) < angular_threshold) {
        stabilityCountA = std::min(stabilityCountA + 1, 2);
        if (stabilityCountA < 2) return;

        stopFlagA = true;
        if (lastStopFlagA != stopFlagA) {
            ROS_INFO("Robot A is stationary, throw IMU and LiDAR measurements");
        }
    } else {
        stabilityCountA = 0;
        stopFlagA = false;
        if (lastStopFlagA!= stopFlagA) {
            ROS_INFO("Robot A is NOT stationary, updating regularly");
        }
    }
    lastStopFlagA = stopFlagA;
}


bool stopFlagB = false;
bool lastStopFlagB = false;
int stabilityCountB = 0;        // 动态窗口，提高稳定性
bool ikdtree_reflective_to_clear = false;
bool ikdtree_reflective_to_build = false;
// 从动到静止需要窗口；静止到动不要窗口
void stop_flag_cbk_b(const nav_msgs::Odometry::ConstPtr& odom_msg) 
{
    // 定义速度阈值，可根据实际情况调整
    const double linear_threshold = 0.03;
    const double angular_threshold = 0.01;

    // 获取线速度和角速度
    double linear_vel = odom_msg->twist.twist.linear.x;
    double angular_vel = odom_msg->twist.twist.angular.z;

    // 判断小车是否静止
    if (std::abs(linear_vel) < linear_threshold && std::abs(angular_vel) < angular_threshold) {
        stabilityCountB = std::min(stabilityCountB + 1, 5);
        if (stabilityCountB < 3) return;

        stopFlagB = true;
        if (lastStopFlagB != stopFlagB) {
            ROS_INFO("Robot B is stationary, throw IMU and LiDAR measurements");
            ikdtree_reflective_to_build = true;
        }
    } else {
        stabilityCountB = 0;
        stopFlagB = false;
        if (lastStopFlagB!= stopFlagB) {
            ROS_INFO("Robot B is NOT stationary, updating regularly");
            ikdtree_reflective_to_clear = true;
        }
    }
    lastStopFlagB = stopFlagB;

}


void getReflectivePoints(const PointCloudXYZI::Ptr& cloud_in, PointCloudXYZI::Ptr& cloud_out)
{
    int cloud_size = cloud_in->points.size();

    for (int i = 0; i < cloud_size; i++)
    {
        if (cloud_in->points[i].intensity > intensity_thres)
        {
            cloud_out->points.push_back(cloud_in->points[i]);
        }
    }
}

// get high-localizability points, in a Euclidean Clustering and Geometric Verification way
// three-stage: 
//1. cluster with Euclidean Clustering; 
//2. Euclidean Clustering; 
//3. Geometric Verification
// remove H.point from cloud_in(undistort_)
// // 20260421 js-ch3n
// void extractHighLocalizabilityFeatures(const PointCloudXYZI::Ptr& cloud_in, 
//                                        PointCloudXYZI::Ptr& cloud_out)
// {
//     cloud_out->clear();
//     if (cloud_in->empty()) return;

//     // ==========================================
//     // 参数配置
//     // ==========================================
//     // const double intensity_thres = 250.0; 
    
//     const double cluster_tolerance = 0.2;     
//     const int min_cluster_size = 10;          
//     const int max_cluster_size = 1000;        
    
//     const double H_ref = reflective_beacon_height;                  
//     const double epsilon_H = 0.1;             
//     const double tau_L = 5.0;                 

//     const double max_distance_thres = basic_motion_length;  
    
//     // --- 新增：挖球剔除的半径 ---
//     const double remove_radius = car_ball_radius;         // 剔除球体的半径 (米)
//     const double remove_radius_sq = remove_radius * remove_radius; // 预计算平方，省去开根号的算力

//     // ==========================================
//     // 1. 基于反射强度的初步筛选 
//     // ==========================================
//     PointCloudXYZI::Ptr cloud_candidates(new PointCloudXYZI());
    
//     for (size_t i = 0; i < cloud_in->points.size(); ++i)
//     {
//         if (cloud_in->points[i].intensity >= intensity_thres)
//         {
//             cloud_candidates->points.push_back(cloud_in->points[i]);
//         }
//     }
    
//     cloud_candidates->width = cloud_candidates->points.size();
//     cloud_candidates->height = 1;
//     cloud_candidates->is_dense = true;

//     if (cloud_candidates->empty()) return;

//     // ==========================================
//     // 2. 空间聚类 
//     // ==========================================
//     pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
//     tree->setInputCloud(cloud_candidates);

//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<PointType> ec;
//     ec.setClusterTolerance(cluster_tolerance);
//     ec.setMinClusterSize(min_cluster_size);
//     ec.setMaxClusterSize(max_cluster_size);
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(cloud_candidates);
//     ec.extract(cluster_indices);

//     // ==========================================
//     // 3. 几何先验模型验证 
//     // ==========================================
    
//     // --- 核心改动：用一个 vector 专门收集有效反光板的质心 ---
//     std::vector<Eigen::Vector3f> valid_centroids;

//     for (const auto& indices : cluster_indices)
//     {
//         PointCloudXYZI::Ptr current_cluster(new PointCloudXYZI());
//         for (const auto& idx : indices.indices)
//         {
//             current_cluster->points.push_back(cloud_candidates->points[idx]);
//         }

//         Eigen::Vector4f centroid;
//         pcl::compute3DCentroid(*current_cluster, centroid);
        
//         // --- 距离校验 ---
//         double distance_to_origin = std::sqrt(centroid[0] * centroid[0] + 
//                                               centroid[1] * centroid[1] + 
//                                               centroid[2] * centroid[2]);
//         if (distance_to_origin > max_distance_thres)
//         {
//             continue;
//         }

//         // --- 3.1 高度校验 ---
//         PointType min_pt, max_pt;
//         pcl::getMinMax3D(*current_cluster, min_pt, max_pt);
//         double H_j = max_pt.z - min_pt.z; 

//         if (std::abs(H_j - H_ref) > epsilon_H)
//         {
//             continue; 
//         }

//         // --- 3.2 形状校验/线性度 ---
//         pcl::PCA<PointType> pca;
//         pca.setInputCloud(current_cluster);
//         Eigen::Vector3f eigen_values = pca.getEigenValues();

//         if (eigen_values[1] <= 1e-6) continue;

//         double L_j = eigen_values[0] / eigen_values[1];

//         // 最终确认为高可信度特征点！
//         if (L_j > tau_L)
//         {
//             *cloud_out += *current_cluster;
            
//             // 把确认是反光板的质心坐标 (x, y, z) 存下来
//             valid_centroids.push_back(Eigen::Vector3f(centroid[0], centroid[1], centroid[2]));
//         }
//     }

//     // ==========================================
//     // 4. 执行原点云“挖球”操作 (剔除质心附近的点)
//     // ==========================================
//     if (!valid_centroids.empty())
//     {
//         PointCloudXYZI clean_cloud;
//         clean_cloud.reserve(cloud_in->size() - cloud_out->size());

//         // 遍历整个原始点云
//         for (size_t i = 0; i < cloud_in->size(); ++i)
//         {
//             const auto& pt = cloud_in->points[i];
//             bool is_inside_any_sphere = false;

//             // 检查当前点是否在任意一个反光板的“半径球”内
//             for (const auto& center : valid_centroids)
//             {
//                 float dx = pt.x - center.x();
//                 float dy = pt.y - center.y();
//                 float dz = pt.z - center.z();
                
//                 // 计算距离的平方，避免耗时的 sqrt 操作
//                 if ((dx * dx + dy * dy + dz * dz) <= remove_radius_sq)
//                 {
//                     is_inside_any_sphere = true;
//                     break; // 只要在一个球内，直接宣判死刑，跳出检查下一个点
//                 }
//             }

//             // 如果不在任何球内，保留该点
//             if (!is_inside_any_sphere) 
//             {
//                 clean_cloud.push_back(pt);
//             }
//         }
        
//         // 替换底层点云数据
//         cloud_in->swap(clean_cloud); 
//     }
// }

void extractHighLocalizabilityFeatures(const PointCloudXYZI::Ptr& cloud_in, 
                                       PointCloudXYZI::Ptr& cloud_out)
{
    cloud_out->clear();
    if (cloud_in->empty()) return;

    // ==========================================
    // 参数配置
    // ==========================================
    // 假设 intensity_thres 和 basic_motion_length 已在外部定义
    const double max_distance_thres = basic_motion_length; 
    
    // 预计算距离的平方，避免在循环里频繁调用耗时的 sqrt 函数
    const double max_distance_sq = max_distance_thres * max_distance_thres;

    // ==========================================
    // 提取逻辑：仅保留满足强度且在距离范围内的点
    // ==========================================
    for (const auto& pt : cloud_in->points)
    {
        // 1. 判断反射强度
        if (pt.intensity >= intensity_thres)
        {
            // 2. 计算点到雷达原点 (0,0,0) 的三维欧氏距离平方
            double dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;

            // 如果在要求的距离范围内，则认为是需要的点
            if (dist_sq <= max_distance_sq)
            {
                cloud_out->points.push_back(pt);
            }
        }
    }

    // 重置点云属性
    cloud_out->width = cloud_out->points.size();
    cloud_out->height = 1;
    cloud_out->is_dense = true;
}

double get_curr_mem_usage() {
    std::ifstream status_file("/proc/self/status");
    std::string line;
    while (std::getline(status_file, line)) {
        if (line.substr(0, 6) == "VmRSS:") {
            // 找到 VmRSS 行，提取数字（单位通常是 kB）
            double value = std::stod(line.substr(7));
            return value / 1024.0; // 转换为 MB
        }
    }
    return 0.0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<double>("mapping/fov_degree",fov_deg,180);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);

    nh.param<bool>("leapslam_runtime_log_enable", leapslam_runtime_log_en, 0);

    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

    nh.param<double>("eigen_thres", eigen_thres, 100);
    nh.param<double>("intensity_thres", intensity_thres, 250);
    nh.param<double>("test", test, 10);
    nh.param<int>("h_weight", h_weight, 10);

    nh.param<double>("filter_size_reflective", filter_size_reflective, 0.03);
    nh.param<double>("basic_motion_length", basic_motion_length, 10.0);
    nh.param<double>("reflective_beacon_diameter", reflective_beacon_diameter, 0.11);
    nh.param<double>("reflective_beacon_height", reflective_beacon_height, 0.30);
    nh.param<double>("car_ball_radius", car_ball_radius, 0.6);

    p_pre->lidar_type = lidar_type;
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;
    
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(point_selected_surf_reflective, true, sizeof(point_selected_surf_reflective));
    memset(res_last, -1000.0f, sizeof(res_last));
    memset(res_last_reflective, -1000.0f, sizeof(res_last_reflective));

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    downSizeFilterReflective.setLeafSize(filter_size_reflective, filter_size_reflective, filter_size_reflective);


    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->lidar_type = lidar_type;
    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubLaserCloudMapRefletive = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_reflective_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/Odometry", 100000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 100000);

// ===============STOP FLAG==================
// cjs 20250624
    std::string odom_topic_a;
    nh.param<string>("odom_topic_a", odom_topic_a, "/odom_a");
    ros::Subscriber sub_stop_flag_a = nh.subscribe(odom_topic_a, 10000, stop_flag_cbk_a);

    std::string odom_topic_b;
    nh.param<string>("odom_topic_b", odom_topic_b, "/odom_b");
    ros::Subscriber sub_stop_flag_b = nh.subscribe(odom_topic_b, 10000, stop_flag_cbk_b);



//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();

    // vector<TumPose> tum_poses;
    int count_loop = 0;
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        if(sync_packages(Measures)) 
        {
            count_loop++;
            
            cout << "packages sync" << endl;
            now_lidar_time = std::to_string(Measures.lidar_beg_time);
            if (flg_EKF_inited && stopFlagA && scan_count > 100)
                continue;

            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;
            double t_degen_start, t_degen_end;
            double t_high_loc_start, t_high_loc_end;
            double t_mitigation_start, t_mitigation_end;

            // 20260413 cjs added, for runtime evaluation that is mentioned in the review comments
            // double tt0, tt1, leapslam_runtime_loop;
            double leapslam_runtime_loop;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            // ===== imu process =====

            t_degen_start = omp_get_wtime();
            
            p_imu->Process(Measures, kf, feats_undistort);

            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            // ===== normal points process =====

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();

            t_degen_end = omp_get_wtime();
            
            // ===== get reflective points =====

            // cjs on 20250626
            // 不能用feats_down_body，这是体素滤波后的，intensity也被平均掉了
            feats_reflective_body->clear();
            getReflectivePoints(feats_undistort, feats_reflective_body);
            // extractHighLocalizabilityFeatures(feats_undistort, feats_reflective_body);
            downSizeFilterReflective.setInputCloud(feats_reflective_body);
            downSizeFilterReflective.filter(*feats_reflective_down_body);
            feats_reflective_size = feats_reflective_down_body->points.size();

            /*** initialize the map kdtree ***/
            // std::cout << "[debug]" << count_loop << ": before ikdtree building" << std::endl;
            if (ikdtree.Root_Node == nullptr || ikdtree_reflective.Root_Node == nullptr)
            {
                if(ikdtree.Root_Node == nullptr)
                {
                    if(feats_down_size > 5)
                    {
                        ikdtree.set_downsample_param(filter_size_map_min);
                        feats_down_world->resize(feats_down_size);
                        for(int i = 0; i < feats_down_size; i++)
                        {
                            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                        }
                        ikdtree.Build(feats_down_world->points);
                    }
                }

                // initialize kdtree for high intensity points
                if(ikdtree_reflective.Root_Node == nullptr)
                {
                    if(feats_reflective_size > 5)
                    {   
                        ikdtree_reflective.set_downsample_param(filter_size_map_min);
                        feats_reflective_world->resize(feats_reflective_size);
                        for(int i = 0; i < feats_reflective_size; i++)
                        {
                            pointBodyToWorld(&(feats_reflective_down_body->points[i]), &(feats_reflective_world->points[i]));
                        }
                        ikdtree_reflective.Build(feats_reflective_world->points);
                    }
                    std::cout << "CONSTRUCT ikdtree_reflective size: " << ikdtree_reflective.size() << std::endl;
                }
                continue;
            }
            

            // check Robot B
            // cjs on 20250626
            // if (0)
            if (ikdtree_reflective_to_build)
            {
                int last_ikdtree_size = ikdtree_reflective.size();
                feats_reflective_world->points.resize(feats_reflective_size);
                for(int i = 0; i < feats_reflective_size; i++)
                {
                    pointBodyToWorld(&(feats_reflective_down_body->points[i]), &(feats_reflective_world->points[i]));
                }
                ikdtree_reflective.reconstruct(feats_reflective_world->points);
                std::cout << "REBUILD ikdtree_reflective size: " << ikdtree_reflective.size() << " from " << last_ikdtree_size << std::endl;
                ikdtree_reflective_to_build = false;
            }

            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);
            feats_reflective_world->resize(feats_reflective_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }
            if(0){ // If you need to see reflective map point, change to "if(1)"
                PointVector ().swap(ikdtree_reflective.PCL_Storage);
                ikdtree_reflective.flatten(ikdtree_reflective.Root_Node, ikdtree_reflective.PCL_Storage, NOT_RECORD);
                featsFromMap_reflective->clear();
                featsFromMap_reflective->points = ikdtree_reflective.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            Nearest_Points_reflective.resize(feats_reflective_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            t_high_loc_end = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);


            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            
            t_mitigation_end = omp_get_wtime();

            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);

            // publish_map_reflective(pubLaserCloudMapRefletive);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
            
            // log 3 modules runtime and mem
            // 1. total runtime of the loop (mitigation_end - degen_start)
            // 2. runtime of 3 modules
            if (leapslam_runtime_log_en)
            {                
                t_high_loc_start = t_degen_end;
                t_mitigation_start = t_high_loc_end;

                leapslam_runtime_loop = t_mitigation_end - t_degen_start; 
                FILE *fp_leapslam;
                string log_dir = root_dir + "/Log/leapslam_runtime_log.csv";
                fp_leapslam = fopen(log_dir.c_str(),"a");
                fprintf(fp_leapslam,"%.8fs, %.8fs, %.8fs, %.8fs, %.2fMB\n", leapslam_runtime_loop, 
                                                                            t_degen_end - t_degen_start, 
                                                                            t_high_loc_end - t_high_loc_start, 
                                                                            t_mitigation_end - t_mitigation_start, 
                                                                            get_curr_mem_usage());
                fclose(fp_leapslam);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    degen_file.close();
    degen_file2.close();
    
     
     cout << "degen_file saved to: " << degen_filename << endl;
     cout << "degen_file2 saved to: " << degen_filename2 << endl;

    return 0;
}
