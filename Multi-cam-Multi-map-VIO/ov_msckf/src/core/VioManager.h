/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef OV_MSCKF_VIOMANAGER_H
#define OV_MSCKF_VIOMANAGER_H


#include <string>
#include <algorithm>
#include <fstream>
#include <Eigen/StdVector>
#include <boost/filesystem.hpp>

#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
#include "track/TrackSIM.h"
#include "init/InertialInitializer.h"
#include "types/LandmarkRepresentation.h"
#include "types/Landmark.h"

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "update/UpdaterZeroVelocity.h"
#include "update/UpdaterOptimize.h"

#include "VioManagerOptions.h"
#include "sim/Simulator.h"

//#include "match/KeyframeDatabase.h"
#include "match/MatchRobust.h"
#include "match/MatchBrief.h"
#include "match/MatchORB.h"

#include <thread>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;
using namespace DBoW2;
using namespace std;
namespace ov_msckf {



    /**
     * @brief Core class that manages the entire system
     *
     * This class contains the state and other algorithms needed for the MSCKF to work.
     * We feed in measurements into this class and send them to their respective algorithms.
     * If we have measurements to propagate or update with, this class will call on our state to do that.
     */
    class VioManager {


    public:


        /**
         * @brief Default constructor, will load all configuration variables
         * @param params_ Parameters loaded from either ROS or CMDLINE
         */
        VioManager(VioManagerOptions& params_, Simulator* sim=nullptr);


        /**
         * @brief Feed function for inertial data
         * @param timestamp Time of the inertial measurement
         * @param wm Angular velocity
         * @param am Linear acceleration
         */
        void feed_measurement_imu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am);


        /**
         * @brief Feed function for a single camera
         * @param timestamp Time that this image was collected
         * @param img0 Grayscale image
         * @param cam_id Unique id of what camera the image is from
         */
        void feed_measurement_monocular(double timestamp, cv::Mat& img0, size_t cam_id);


        /**
         * @brief Feed function for stereo camera pair
         * @param timestamp Time that this image was collected
         * @param img0 Grayscale image
         * @param img1 Grayscale image
         * @param cam_id0 Unique id of what camera the image is from
         * @param cam_id1 Unique id of what camera the image is from
         */
        void feed_measurement_stereo(double timestamp, cv::Mat& img0, cv::Mat& img1, size_t cam_id0, size_t cam_id1);

        /**
         * @brief Feed function for stereo camera pair
         * @param timestamp Time that this image was collected
         * @param img0 Grayscale image
         * @param img1 Grayscale image
         * @param img2 Grayscale image
         * @param img3 Grayscale image
         * @param cam_id0 Unique id of what camera the image is from
         * @param cam_id1 Unique id of what camera the image is from
         * @param cam_id2 Unique id of what camera the image is from
         * @param cam_id3 Unique id of what camera the image is from
         */
        void feed_measurement_four(double timestamp, cv::Mat& img0, cv::Mat& img1, cv::Mat& img2, cv::Mat& img3, size_t cam_id0, size_t cam_id1,  size_t cam_id2, size_t cam_id3); 
        
        /**
         * @brief Feed function for a synchronized simulated cameras
         * @param timestamp Time that this image was collected
         * @param camids Camera ids that we have simulated measurements for
         * @param feats Raw uv simulated measurements
         */
        void feed_measurement_simulation(double timestamp, const std::vector<int> &camids, const std::vector<std::vector<std::pair<size_t,Eigen::VectorXf>>> &feats);

        /**
         * @brief Given a state, this will initialize our IMU state.
         * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
         */
        void initialize_with_gt(Eigen::Matrix<double,17,1> imustate) {

            // Initialize the system
            state->_imu->set_value(imustate.block(1,0,16,1));
            state->_imu->set_fej(imustate.block(1,0,16,1));
            state->_timestamp = imustate(0,0);
            startup_time = imustate(0,0);
            is_initialized_vio = true;

            // Cleanup any features older then the initialization time
            trackFEATS->get_feature_database()->cleanup_measurements(state->_timestamp);
            if(trackARUCO != nullptr) {
                trackARUCO->get_feature_database()->cleanup_measurements(state->_timestamp);
            }

            // Print what we init'ed with
            printf(GREEN "[INIT]: INITIALIZED FROM GROUNDTRUTH FILE!!!!!\n" RESET);
            printf(GREEN "[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\n" RESET,state->_imu->quat()(0),state->_imu->quat()(1),state->_imu->quat()(2),state->_imu->quat()(3));
            printf(GREEN "[INIT]: bias gyro = %.4f, %.4f, %.4f\n" RESET,state->_imu->bias_g()(0),state->_imu->bias_g()(1),state->_imu->bias_g()(2));
            printf(GREEN "[INIT]: velocity = %.4f, %.4f, %.4f\n" RESET,state->_imu->vel()(0),state->_imu->vel()(1),state->_imu->vel()(2));
            printf(GREEN "[INIT]: bias accel = %.4f, %.4f, %.4f\n" RESET,state->_imu->bias_a()(0),state->_imu->bias_a()(1),state->_imu->bias_a()(2));
            printf(GREEN "[INIT]: position = %.4f, %.4f, %.4f\n" RESET,state->_imu->pos()(0),state->_imu->pos()(1),state->_imu->pos()(2));

        }

        void feed_measurement_control(bool is_static)
        {
            _is_static = is_static;
            if (is_static)
                printf(YELLOW "[STATIC]: STATIC CONTROL!!!!!\n" RESET);
        }




        /// If we are initialized or not
        bool initialized() {
            return is_initialized_vio;
        }

        /// Timestamp that the system was initialized at
        double initialized_time() {
            return startup_time;
        }

        /// Accessor to get the current state
        State* get_state() {
            return state;
        }

        /// Accessor to get the current propagator
        Propagator* get_propagator() {
            return propagator;
        }

        /// Get feature tracker
        TrackBase* get_track_feat() {
            return trackFEATS;
        }

        /// Get aruco feature tracker
        TrackBase* get_track_aruco() {
            return trackARUCO;
        }

        /// Returns 3d features used in the last update in global frame
        std::vector<Eigen::Vector3d> get_good_features_MSCKF() {
            return good_features_MSCKF;
        }

        /// Returns 3d SLAM features in the global frame
        std::vector<Eigen::Vector3d> get_features_SLAM() {
            std::vector<Eigen::Vector3d> slam_feats;
            for (auto it=state->_clones_IMU.begin(); it != state->_clones_IMU.end(); it++)
            for (auto &f : state->_features_SLAM) {
                if((int)f.first <= state->_options.max_aruco_features) continue;
                if(LandmarkRepresentation::is_relative_representation(f.second->_feat_representation)) {
                    // Assert that we have an anchor pose for this feature
                    assert(f.second->_anchor_cam_id!=-1);
                    // Get calibration for our anchor camera
                    Eigen::Matrix<double, 3, 3> R_ItoC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->Rot();
                    Eigen::Matrix<double, 3, 1> p_IinC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->pos();
                    // Anchor pose orientation and position
                    Eigen::Matrix<double,3,3> R_GtoI = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->Rot();
                    Eigen::Matrix<double,3,1> p_IinG = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->pos();
                    // Feature in the global frame
                    slam_feats.push_back(R_GtoI.transpose() * R_ItoC.transpose()*(f.second->get_xyz(false) - p_IinC) + p_IinG);
                } else {
                    slam_feats.push_back(f.second->get_xyz(false));
                }
            }
            return slam_feats;
        }

        /// Returns 3d ARUCO features in the global frame
        std::vector<Eigen::Vector3d> get_features_ARUCO() {
            std::vector<Eigen::Vector3d> aruco_feats;
            for (auto &f : state->_features_SLAM) {
                if((int)f.first > state->_options.max_aruco_features) continue;
                if(LandmarkRepresentation::is_relative_representation(f.second->_feat_representation)) {
                    // Assert that we have an anchor pose for this feature
                    assert(f.second->_anchor_cam_id!=-1);
                    // Get calibration for our anchor camera
                    Eigen::Matrix<double, 3, 3> R_ItoC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->Rot();
                    Eigen::Matrix<double, 3, 1> p_IinC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->pos();
                    // Anchor pose orientation and position
                    Eigen::Matrix<double,3,3> R_GtoI = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->Rot();
                    Eigen::Matrix<double,3,1> p_IinG = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->pos();
                    // Feature in the global frame
                    aruco_feats.push_back(R_GtoI.transpose() * R_ItoC.transpose()*(f.second->get_xyz(false) - p_IinC) + p_IinG);
                } else {
                    aruco_feats.push_back(f.second->get_xyz(false));
                }
            }
            return aruco_feats;
        }

        /// Return true if we did a zero velocity update
        bool did_zero_velocity_update(){
            return did_zupt_update;
        }

        /// Return the zero velocity update image
        cv::Mat get_zero_velocity_update_image() {
            return zupt_image;
        }

        /// Returns the last timestamp we have marginalized (true if we have a state)
        bool hist_last_marg_state(double &timestamp, Eigen::Matrix<double,7,1> &stateinG) {
            if(hist_last_marginalized_time != -1) {
                timestamp = hist_last_marginalized_time;
                stateinG = hist_stateinG.at(hist_last_marginalized_time);
                return true;
            } else {
                timestamp = -1;
                stateinG.setZero();
                return false;
            }
        }

        /// Returns historical feature positions, and measurements times and uvs used to get its estimate.
        void hist_get_features(std::unordered_map<size_t,Eigen::Vector3d> &feat_posinG,
                               std::unordered_map<size_t, std::unordered_map<size_t, std::vector<Eigen::VectorXf>>> &feat_uvs,
                               std::unordered_map<size_t, std::unordered_map<size_t, std::vector<Eigen::VectorXf>>> &feat_uvs_norm,
                               std::unordered_map<size_t, std::unordered_map<size_t, std::vector<double>>> &feat_timestamps) {
            feat_posinG = hist_feat_posinG;
            feat_uvs = hist_feat_uvs;
            feat_uvs_norm = hist_feat_uvs_norm;
            feat_timestamps = hist_feat_timestamps;
        }






        void loadPoseGraph_multimap(std::string map_save_path,std::vector<std::string> pose_graph_filenames, std::vector<std::string> keyframe_pose_filenames, std::vector<std::string> querry_pose_filenames);

        void loadPoseGraph_multimap_visualization(std::string map_save_path, std::vector<std::string> keyframe_pose_filenames);
        
        void ComputeKFPose(Keyframe *cur_kf);

        bool ComputeRelativePose(Keyframe *cur_kf, Keyframe *loop_kf,Vector3d &p_loopIncur, Matrix3d &R_loopTocur);

        bool ComputeRelativePose(Keyframe *cur_kf, Keyframe *loop_kf,Vector3d &p_loopIncur, Matrix3d &R_loopTocur, int init_cur_kf_cam_id);


  
    
        void get_multiKF_match_feature(double timestamp, vector<Keyframe*> loop_kfs, int &index, vector<cv::Point2f>& uv_loop,vector<cv::Point3f>& uv_3d_loop, vector<cv::Point2f>& uv, vector<Feature*>& feats_with_loop);

        void get_multi_map_KF_match_feature(double timestamp, vector<Keyframe*> loop_kfs, int &index, map<int,vector<cv::Point2f>>& uv_loops,map<int,vector<cv::Point3f>>& uv_3d_loops, map<int,vector<cv::Point2f>>& uv_norms, vector<Feature*>& feats_with_loop);    
        

        vector<Keyframe*> get_loop_kfs(State *state);

        
        
        double get_approx_ts(double timestamp)
        {
           double ts1=((timestamp*100)-3)/100.0;  //minus 0.03s
            double ts2=((timestamp*100)+3)/100.0;  //add 0.03s
            vector<double> ts;
            for(int i=0;i<query_timestamps.size();i++)
            {
                if(query_timestamps[i]<ts2&&query_timestamps[i]>ts1)
                {
                    ts.push_back(query_timestamps[i]);
                }
            }
            if(ts.empty())
            {
                return -1.0;
            }
            double min_error=10000;
            double approx_ts=-1.0;
            for(int i=0;i<ts.size();i++)
            {
                double error=abs(ts[i]-timestamp);
                if(error<min_error)
                {
                    min_error=error;
                    approx_ts=ts[i];
                }
            }
            return approx_ts;

        }

        

    // protected:


        /**
         * @brief This function will try to initialize the state.
         *
         * This should call on our initializer and try to init the state.
         * In the future we should call the structure-from-motion code from here.
         * This function could also be repurposed to re-initialize the system after failure.         *
         * @return True if we have successfully initialized
         */
        bool try_to_initialize();

        // /**
        //  * @brief This function will will re-triangulate all features in the current frame
        //  *
        //  * For all features that are currently being tracked by the system, this will re-triangulate them.
        //  * This is useful for downstream applications which need the current pointcloud of points (e.g. loop closure).
        //  * This will try to triangulate *all* points, not just ones that have been used in the update.
        //  *
        //  * @param message Contains our timestamp, images, and camera ids
        //  */
        // void retriangulate_active_tracks(Keyframe* cur_kf_0,Keyframe* cur_kf_1,Keyframe* cur_kf_2,Keyframe* cur_kf_3);


        /**
         * @brief This will do the propagation and feature updates to the state
         * @param timestamp The most recent timestamp we have tracked to
         */
        void do_feature_propagate_update(double timestamp,Keyframe* cur_kf_0= nullptr,Keyframe* cur_kf_1= nullptr);
        void mv_do_feature_propagate_update(double timestamp,Keyframe* cur_kf_0= nullptr,Keyframe* cur_kf_1= nullptr, Keyframe* cur_kf_2= nullptr,Keyframe* cur_kf_3= nullptr);


        /**
         * @brief This function will update our historical tracking information.
         * This historical information includes the best estimate of a feature in the global frame.
         * For all features it also has the normalized and raw pixel coordinates at each timestep.
         * The state is also recorded after it is marginalized out of the state.
         * @param features Features using in the last update phase
         */
        void update_keyframe_historical_information(const std::vector<Feature*> &features);


        /// Manager parameters
        VioManagerOptions params;

        /// Our master state object :D
        State* state;

        /// Propagator of our state
        Propagator* propagator;

        Simulator* simulator;

        /// Our sparse feature tracker (klt or descriptor)
        TrackBase* trackFEATS = nullptr;

        /// Our aruoc tracker
        TrackBase* trackARUCO = nullptr;

        /// State initializer
        InertialInitializer* initializer;

        /// Boolean if we are initialized or not
        bool is_initialized_vio = false;

        /// Our MSCKF feature updater
        UpdaterMSCKF* updaterMSCKF;

        /// Our MSCKF feature updater
        UpdaterSLAM* updaterSLAM;

        /// Our aruoc tracker
        UpdaterZeroVelocity* updaterZUPT = nullptr;

        UpdaterOptimize* updaterOptimize;

        ///Our matcher
        MatchBase* matchKF;

        thread* th_matchKF;

        /// Good features that where used in the last update
        std::vector<Eigen::Vector3d> good_features_MSCKF;

        // Timing statistic file and variables
        std::ofstream of_statistics;
        boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;
        boost::posix_time::ptime relinear_t1,relinear_t2;
        // Track how much distance we have traveled
        double timelastupdate = -1;
        double distance = 0;

        // Startup time of the filter
        double startup_time = -1;

        // If we did a zero velocity update
        bool did_zupt_update = false;
        cv::Mat zupt_image;
        cv::Mat zupt_image1,zupt_image2;

        double relinear_time=0;

        ///added by zzq
        //we need the Keyframe database
//        KeyframeDatabase* keyframeDatabase;
//

        BriefVocabulary* voc;

        double last_kf_match_time =-1;

        int reset_map_count=0;

        int optimize_count=0;

        int count_1=0;
        int count_2=0;
        int count_3=0;
        int count_4=0;

        int last_optimize_time=0;

        double duration=0;

        double optimize_time=0;

        int max_kf_id;

        int sim_feats_num=0;

        size_t max_kf_id_in_database;

        vector<double> stop_time;

        //if we get the initial relative pose between map and vio
        bool map_initialized =false;

        //relative tranformation from current VIO reference frame to Map reference frame.
        PoseJPL* _pose_C_to_M;
        Vector3d p_vio_in_map;
        Matrix3d R_vio_to_map;

        int count_msckf_with_kf=0;
        int count_slam_with_kf=0;
        int count_slam_delay_with_kf=0;

        vector<double> query_timestamps;

        map<double,vector<double>> matching_info;

        std::ofstream of_transform;

        map<int,vector<PoseJPL*>> map_poses;

        /// Feature initializer used to triangulate all active tracks
        std::shared_ptr<FeatureInitializer> active_tracks_initializer;

        // Historical information of the filter (last marg time, historical states, features seen from all frames)
        double hist_last_marginalized_time = -1;
        //map between timestamp and state
        std::map<double,Eigen::Matrix<double,7,1>> hist_stateinG;
        //map between feature id and its 3d location in globalframe
        std::unordered_map<size_t, Eigen::Vector3d> hist_feat_posinG;
        std::unordered_map<size_t, std::unordered_map<size_t, std::vector<Eigen::VectorXf>>> hist_feat_uvs;
        std::unordered_map<size_t, std::unordered_map<size_t, std::vector<Eigen::VectorXf>>> hist_feat_uvs_norm;
        std::unordered_map<size_t, std::unordered_map<size_t, std::vector<double>>> hist_feat_timestamps;  //feature id  --> <cam_id, timestamps>

        std::queue<double> online_selected_kf_ts;

        bool _is_static;
    };


}



#endif //OV_MSCKF_VIOMANAGER_H
