#include <csignal>

#include "sim/Simulator.h"
#include "core/VioManager.h"
#include "utils/dataset_reader.h"
#include "utils/parse_cmd.h"
#include "utils/colors.h"
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>

#ifdef ROS_AVAILABLE
#include <ros/ros.h>
#include "core/RosVisualizer.h"
#include "utils/parse_ros.h"
#endif


using namespace ov_msckf;
using namespace std;

Simulator* sim;
VioManager* sys;
#ifdef ROS_AVAILABLE
RosVisualizer* viz_;
#endif




// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
    std::exit(signum);
}


// Main function
int main(int argc, char** argv)
{

    // Read in our parameters
    VioManagerOptions params;
    #ifdef ROS_AVAILABLE
        ros::init(argc, argv, "test_simulation");
        ros::NodeHandle nh("~");
        params = parse_ros_nodehandler(nh);
    #else
        params = parse_command_line_arguments(argc, argv);
    #endif

    // Create our VIO system
    sim = new Simulator(params);
    sys = new VioManager(params);
    #ifdef ROS_AVAILABLE
        viz_ = new RosVisualizer(nh, sys, sim);
    #endif

    string map_cam_pose_file=params.map_save_path+"circle_21_cam.txt";
    string map_cam_pose_file_turb=params.map_save_path+"traj_estimate_circle21_cam.txt";
    string matching_file_name=params.map_save_path + params.pose_graph_filename;
    string save_matching_file_name="/tmp/sim_single_matching.txt";
    string save_multi_matching_file_name="/tmp/sim_multi_matching.txt";
   
    if(params.multi_match)
    {
        cout<<"in generate map multiple"<<endl;
        ifstream fi_matching;
        fi_matching.open(matching_file_name.data());
        assert(fi_matching.is_open());
        ofstream fo_matching_res;
        if (boost::filesystem::exists(save_multi_matching_file_name)) {
            boost::filesystem::remove(save_multi_matching_file_name);
            printf(YELLOW "[STATS]: found old file found, deleted...\n" RESET);
        }
        // Create the directory that we will open the file in
        boost::filesystem::path p(save_multi_matching_file_name);
        boost::filesystem::create_directories(p.parent_path());
        // Open our statistics file!
        fo_matching_res.open(save_multi_matching_file_name, std::ofstream::out | std::ofstream::app);
        assert(fo_matching_res.is_open());

        string str,result;
        int line_num=0;
        double query_ts,kf_ts;
        string query_timestamp,kf_timestamp;
        int match_num=0;
        int match_num_per_img=0;
        int total_line_num_per_img=0;
        vector<string> matching_kfs_ts;
        vector<string> matching_kfs_name;
        vector<int> matching_matches_num;
        while (getline(fi_matching,str))
        {
            if(line_num==0)
            {
                stringstream line(str);
                cout<<str<<endl;
                line>>result; //query_timestamp
                query_timestamp=result;
                line>>result; //match_num_per_img
                match_num_per_img=stoi(result);
                total_line_num_per_img=2*match_num_per_img;
                line_num++;
                matching_kfs_ts.clear();
                matching_kfs_name.clear();
                matching_matches_num.clear();
                continue;
            }
            if(line_num<=total_line_num_per_img)
            {
                if(line_num%2==1)//query_timestamp, keyframe_timestamp, match_number
                {
                /****** for eight *****/
                stringstream line(str);
                line>>result;
                query_timestamp=result;
                query_ts=stod(result);
                query_ts=round(query_ts*100)/100.0;
                  cout<<"***query_ts"<<to_string(query_ts)<<endl;
                line>>result;
                matching_kfs_ts.push_back(result);
                kf_ts=stod(result);
                kf_ts=floor(kf_ts*100)/100.0;  //保留两位小数
                  cout<<"***kf_ts"<<to_string(kf_ts)<<endl;
                line>>result;
                match_num=stoi(result);
                matching_matches_num.push_back(match_num);
            
                }
                
                if(line_num==total_line_num_per_img)
                {
                    line_num=0;
                }
                else
                {
                    line_num++;
                }
                
            }
            if(line_num==0)  //this only happened when we traverse all matching information of the current query image
            {
                //now we get all mathcing kf timestamp, and we need to find their gt and current query image gt;

                //frist get current query image gt;
                
                Matrix3d R_GtoI;
                Matrix3d R_GtoI_turb;
                Vector3d p_IinG;
                Vector3d p_IinG_turb;
                bool found=false;
                found=sim->spline.get_pose(query_ts, R_GtoI, p_IinG);
                found=sim->spline_turb.get_pose(query_ts,R_GtoI_turb,p_IinG_turb);
               cout<<"found "<<found<<endl;
               cout<<"current pos: "<<p_IinG.transpose()<<endl<<"currentpos turb: "<<p_IinG_turb.transpose()<<endl;
                if(found==false)
                   continue;
                Eigen::Matrix<double,3,3> R_ItoC = quat_2_Rot(params.camera_extrinsics.at(0).block(0,0,4,1));
                Eigen::Matrix<double,3,1> p_IinC = params.camera_extrinsics.at(0).block(4,0,3,1);
                Matrix3d R_CtoG=R_GtoI.transpose()*R_ItoC.transpose();
                Vector3d p_CinG=p_IinG-R_CtoG*p_IinC;
                Matrix3d R_CtoG_turb=R_GtoI_turb.transpose()*R_ItoC.transpose();
                Vector3d p_CinG_turb=p_IinG_turb-R_CtoG_turb*p_IinC;


                vector<string> kfs_ts_with_gt;
                vector<int> matches_num;
                vector<Matrix3d> kfs_pose_R;
                vector<Vector3d> kfs_pose_t;
                vector<Matrix3d> kfs_pose_R_turb;
                vector<Vector3d> kfs_pose_t_turb;
                sim->featmatching.clear();
                sim->featmatchingturb.clear();
                sim->id_matching=0;
                sim->uvsmatching.clear();
                //now get the all matching kf gt of the current query image
                for(int i=0;i<match_num_per_img;i++)
                {
                    double t=stod(matching_kfs_ts[i]);
                    t=floor(t*100)/100.0; //used for eight
                     cout<<"match kfs ts i: "<<to_string(t)<<endl;
                    ifstream fi_kf_gt;
                    ifstream fi_kf_gt_turb;
                    fi_kf_gt.open(map_cam_pose_file.data());
                    fi_kf_gt_turb.open(map_cam_pose_file_turb.data());
                    assert(fi_kf_gt.is_open());
                    assert(fi_kf_gt_turb.is_open());
                    string str2,res2;
                    PoseJPL* kf_cam=new PoseJPL();
                    found=false;
                    while(getline(fi_kf_gt,str2))  //timestamp,tx,ty,tz,qx,qy,qz,qw
                    {
                        /**for eight**/
                        stringstream line2(str2);
                        line2>>res2;
                        double ts=stod(res2);
                        double timestp=ts;
                        ts=floor(ts*100)/100.0; //保留两位小数
                        if(ts==t)
                        {
                            line2>>res2;
                            float tx=atof(res2.c_str());
                            line2>>res2;
                            float ty=atof(res2.c_str());
                            line2>>res2;
                            float tz=atof(res2.c_str());
                            line2>>res2;
                            float qx=atof(res2.c_str());
                            line2>>res2;
                            float qy=atof(res2.c_str());
                            line2>>res2;
                            float qz=atof(res2.c_str());
                            line2>>res2;
                            float qw=atof(res2.c_str());
                            //here we load the quaternion in the form of Hamilton,we need to tranform it into JPL
                            Quaterniond q1(qw,qx,qy,qz);  
                            Quaterniond q1_JPL(q1.toRotationMatrix().transpose());
                            Eigen::Matrix<double,7,1> q_kfInw;
                            q_kfInw<<q1_JPL.x(),q1_JPL.y(),q1_JPL.z(),q1_JPL.w(),tx, ty, tz;
                            kf_cam->set_value(q_kfInw);
                           cout<<"get kf pose"<<endl;
                            found=true;
                            break;
                        }

                    }
                    fi_kf_gt.close();
                    PoseJPL* kf_cam_turb=new PoseJPL();
                    bool found_2=false;
                    while(getline(fi_kf_gt_turb,str2))  //timestamp,tx,ty,tz,qx,qy,qz,qw
                    {

                        /**for eight**/
                        stringstream line2(str2);
                        line2>>res2;
                        double ts=stod(res2);
                        double timestp=ts;
                        ts=floor(ts*100)/100.0; //保留两位小数
                        if(ts==t)
                        {
                            line2>>res2;
                            float tx=atof(res2.c_str());
                            line2>>res2;
                            float ty=atof(res2.c_str());
                            line2>>res2;
                            float tz=atof(res2.c_str());
                            line2>>res2;
                            float qx=atof(res2.c_str());
                            line2>>res2;
                            float qy=atof(res2.c_str());
                            line2>>res2;
                            float qz=atof(res2.c_str());
                            line2>>res2;
                            float qw=atof(res2.c_str());
                            //here we load the quaternion in the form of Hamilton,we need to tranform it into JPL
                            Quaterniond q1(qw,qx,qy,qz);
                            Quaterniond q1_JPL(q1.toRotationMatrix().transpose());
                            Eigen::Matrix<double,7,1> q_kfInw;
                            q_kfInw<<q1_JPL.x(),q1_JPL.y(),q1_JPL.z(),q1_JPL.w(),tx, ty, tz;
                            kf_cam_turb->set_value(q_kfInw);
                           cout<<"get kf pose"<<endl;
                            found_2=true;
                            break;
                        }

                    }
                    fi_kf_gt_turb.close();
                    if(found==true&&found_2==true)
                    {
                      cout<<"found all kf pose"<<endl;
                        kfs_ts_with_gt.push_back(matching_kfs_ts[i]);  //for YQ and eight
                            matches_num.push_back(matching_matches_num[i]);
                        kfs_pose_R.push_back(kf_cam->Rot());
                        kfs_pose_t.push_back(kf_cam->pos());
                        kfs_pose_R_turb.push_back(kf_cam_turb->Rot());
                        kfs_pose_t_turb.push_back(kf_cam_turb->pos());
                    }

                }

                //if num of kf is one, we skip it, as we cannot triangulate.
                if(kfs_ts_with_gt.size()==0||kfs_ts_with_gt.size()==1)
                {
                    continue;
                }
                assert(kfs_pose_R.size()==kfs_pose_t.size());
                assert(kfs_pose_R.size()==kfs_pose_R_turb.size());
                assert(kfs_pose_R.size()==kfs_pose_t_turb.size());
                assert(kfs_pose_R.size()==matches_num.size());
                assert(kfs_pose_R.size()==kfs_ts_with_gt.size());
                cout<<"num kf: "<<kfs_ts_with_gt.size()<<endl;

//                fo_matching_res<<query_timestamp<<" "<<to_string(kfs_ts_with_gt.size())<<endl;
                vector<string> res_kfs_ts;
                vector<int> res_match_num;
                vector<vector<Eigen::VectorXf>> res_uvs_cur;
                vector<vector<Eigen::VectorXf>> res_uvs_kf;
                vector<vector<Eigen::VectorXd>> res_pts3d_kf;

                sim->gen_matching(kfs_pose_R,kfs_pose_t,kfs_pose_R_turb,kfs_pose_t_turb,
                                  R_CtoG,p_CinG,matches_num,kfs_ts_with_gt,res_kfs_ts,
                                  res_uvs_cur,res_uvs_kf,res_pts3d_kf);

                if(res_kfs_ts.size()>0)
                    fo_matching_res<<query_timestamp<<" "<<to_string(res_kfs_ts.size())<<endl;
                for(int i=0;i<res_kfs_ts.size();i++)
                {
                    fo_matching_res<<query_timestamp<<" "<<res_kfs_ts[i]<<" "<<to_string(res_pts3d_kf[i].size())<<endl;
                    for(int j=0;j<res_pts3d_kf[i].size();j++)
                    {
                        fo_matching_res<<to_string(res_uvs_cur[i][j](0))<<" "<<to_string(res_uvs_cur[i][j](1))<<" ";
                        fo_matching_res<<to_string(res_uvs_kf[i][j](0))<<" "<<to_string(res_uvs_kf[i][j](1))<<" ";
                        fo_matching_res<<to_string(res_pts3d_kf[i][j](0))<<" "<<to_string(res_pts3d_kf[i][j](1))<<" "<<to_string(res_pts3d_kf[i][j](2))<<" ";
                    }
                    fo_matching_res<<endl;
                }
            }
        }
        fi_matching.close();
        fo_matching_res.close();    
        
    }
    else
    {
        cout<<"in generate map single"<<endl;
        ifstream fi_matching;
        fi_matching.open(matching_file_name.data());
        assert(fi_matching.is_open());
        ofstream fo_matching_res;
        if (boost::filesystem::exists(save_matching_file_name)) {
            boost::filesystem::remove(save_matching_file_name);
            printf(YELLOW "[STATS]: found old file found, deleted...\n" RESET);
        }
        // Create the directory that we will open the file in
        boost::filesystem::path p(save_matching_file_name);
        boost::filesystem::create_directories(p.parent_path());
        // Open our statistics file!
        fo_matching_res.open(save_matching_file_name, std::ofstream::out | std::ofstream::app);
        assert(fo_matching_res.is_open());

        string str,result;
        int line_num=0;
        double query_ts,kf_ts;
        string query_timestamp,kf_timestamp;
        int match_num=0;
        int match_num_per_img=0;
        int total_line_num_per_img=0;
        vector<string> matching_kfs_ts;
        vector<string> matching_kfs_name;
        vector<int> matching_matches_num;
        while (getline(fi_matching,str))
        {
            if(line_num==0)
            {
                stringstream line(str);
                line>>result; //query_timestamp
                cout<<str<<endl;
                query_timestamp=result;
                line>>result; //match_num_per_img
                match_num_per_img=stoi(result);
                total_line_num_per_img=2*match_num_per_img;
                line_num++;
                matching_kfs_ts.clear();
                matching_kfs_name.clear();
                matching_matches_num.clear();
                continue;
            }
            if(line_num<=total_line_num_per_img)
            {
                if(line_num%2==1)//query_timestamp, keyframe_timestamp, match_number
                {
                    /****** for eight *****/
                    stringstream line(str);
                    line>>result;
                    query_timestamp=result;
                    query_ts=stod(result);
                    query_ts=round(query_ts*100)/100.0;
                    line>>result;
                    matching_kfs_ts.push_back(result);
                    kf_ts=stod(result);
                    kf_ts=floor(kf_ts*100)/100.0;  //保留两位小数
                    line>>result;
                    match_num=stoi(result);
                    matching_matches_num.push_back(match_num);
                }
                
                if(line_num==total_line_num_per_img)
                {
                    line_num=0;
                }
                else
                {
                    line_num++;
                }
                
            }
            if(line_num==0)  //this only happened when we traverse all matching information of the current query image
            {
                //now we get all mathcing kf timestamp, and we need to find their gt and current query image gt;

                //frist get current query image gt;
                
                Matrix3d R_GtoI;
                Matrix3d R_GtoI_turb;
                Vector3d p_IinG;
                Vector3d p_IinG_turb;
                bool found=false;
                found=sim->spline.get_pose(query_ts, R_GtoI, p_IinG);
                found=sim->spline_turb.get_pose(query_ts,R_GtoI_turb,p_IinG_turb);
                cout<<"found pose: "<<found<<endl;
                if(found==false)
                   continue;
                Eigen::Matrix<double,3,3> R_ItoC = quat_2_Rot(params.camera_extrinsics.at(0).block(0,0,4,1));
                Eigen::Matrix<double,3,1> p_IinC = params.camera_extrinsics.at(0).block(4,0,3,1);
                Matrix3d R_CtoG=R_GtoI.transpose()*R_ItoC.transpose();
                Vector3d p_CinG=p_IinG-R_CtoG*p_IinC;
                Matrix3d R_CtoG_turb=R_GtoI_turb.transpose()*R_ItoC.transpose();
                Vector3d p_CinG_turb=p_IinG_turb-R_CtoG_turb*p_IinC;


                vector<string> kfs_ts_with_gt;
                vector<int> matches_num;
                vector<Matrix3d> kfs_pose_R;
                vector<Vector3d> kfs_pose_t;
                vector<Matrix3d> kfs_pose_R_turb;
                vector<Vector3d> kfs_pose_t_turb;
                sim->featmatching.clear();
                sim->featmatchingturb.clear();
                sim->id_matching=0;
                sim->uvsmatching.clear();
                //now get the all matching kf gt of the current query image
                for(int i=0;i<match_num_per_img;i++)
                {
                    double t=stod(matching_kfs_ts[i]);
                    // t=floor(t*10)/10.0; //used for YQ
                    t=floor(t*100)/100.0; //used for eight
                    ifstream fi_kf_gt;
                    ifstream fi_kf_gt_turb;
                    fi_kf_gt.open(map_cam_pose_file.data());
                    fi_kf_gt_turb.open(map_cam_pose_file_turb.data());
                    assert(fi_kf_gt.is_open());
                    assert(fi_kf_gt_turb.is_open());
                    string str2,res2;
                    PoseJPL* kf_cam=new PoseJPL();
                    found=false;
                    while(getline(fi_kf_gt,str2))  //timestamp,tx,ty,tz,qx,qy,qz,qw
                    {
                        /**for eight**/
                        stringstream line2(str2);
                        line2>>res2;
                        double ts=stod(res2);
                        double timestp=ts;
                        ts=floor(ts*100)/100.0; //保留两位小数
                        if(ts==t)
                        {
                            line2>>res2;
                            float tx=atof(res2.c_str());
                            line2>>res2;
                            float ty=atof(res2.c_str());
                            line2>>res2;
                            float tz=atof(res2.c_str());
                            line2>>res2;
                            float qx=atof(res2.c_str());
                            line2>>res2;
                            float qy=atof(res2.c_str());
                            line2>>res2;
                            float qz=atof(res2.c_str());
                            line2>>res2;
                            float qw=atof(res2.c_str());
                            //here we load the quaternion in the form of Hamilton,we need to tranform it into JPL
                            Quaterniond q1(qw,qx,qy,qz);  
                            Quaterniond q1_JPL(q1.toRotationMatrix().transpose());
                            Eigen::Matrix<double,7,1> q_kfInw;
                            q_kfInw<<q1_JPL.x(),q1_JPL.y(),q1_JPL.z(),q1_JPL.w(),tx, ty, tz;
                            kf_cam->set_value(q_kfInw);
                            found=true;
                            break;
                        }

                    }
                    fi_kf_gt.close();
                    PoseJPL* kf_cam_turb=new PoseJPL();
                    bool found_2=false;
                    while(getline(fi_kf_gt_turb,str2))  //timestamp,tx,ty,tz,qx,qy,qz,qw
                    {
                         // /**for Euroc**/
                        // stringstream line2(str2);
                        // line2>>res2;
                        // double ts=stod(res2);
                        // double timestp=ts;
                        // ts=floor(ts*10000)/10000.0; //保留2位小数for kaist 4 for euroc
                        /**for YQ**/
                        // stringstream line2(str2);
                        // line2>>res2;
                        // double ts=stod(res2);
                        // double timestp=ts;
                        // ts=floor(ts*10)/10.0; //保留一位小数
                        /**for eight**/
                        stringstream line2(str2);
                        line2>>res2;
                        double ts=stod(res2);
                        double timestp=ts;
                        ts=floor(ts*100)/100.0; //保留两位小数
                        if(ts==t)
                        {
                            line2>>res2;
                            float tx=atof(res2.c_str());
                            line2>>res2;
                            float ty=atof(res2.c_str());
                            line2>>res2;
                            float tz=atof(res2.c_str());
                            line2>>res2;
                            float qx=atof(res2.c_str());
                            line2>>res2;
                            float qy=atof(res2.c_str());
                            line2>>res2;
                            float qz=atof(res2.c_str());
                            line2>>res2;
                            float qw=atof(res2.c_str());
                            //here we load the quaternion in the form of Hamilton,we need to tranform it into JPL
                            Quaterniond q1(qw,qx,qy,qz);
                            Quaterniond q1_JPL(q1.toRotationMatrix().transpose());
                            Eigen::Matrix<double,7,1> q_kfInw;
                            q_kfInw<<q1_JPL.x(),q1_JPL.y(),q1_JPL.z(),q1_JPL.w(),tx, ty, tz;
                            kf_cam_turb->set_value(q_kfInw);
                            found_2=true;
                            break;
                        }

                    }
                    fi_kf_gt_turb.close();
                    if(found==true&&found_2==true)
                    {
                        kfs_ts_with_gt.push_back(matching_kfs_ts[i]);  //for YQ and eight
                        matches_num.push_back(matching_matches_num[i]);
                        kfs_pose_R.push_back(kf_cam->Rot());
                        kfs_pose_t.push_back(kf_cam->pos());
                        kfs_pose_R_turb.push_back(kf_cam_turb->Rot());
                        kfs_pose_t_turb.push_back(kf_cam_turb->pos());
                    }

                }

                //if num of kf is one, we skip it, as we cannot triangulate.
                if(kfs_ts_with_gt.size()==0||kfs_ts_with_gt.size()==1)
                {
                    continue;
                }
                assert(kfs_pose_R.size()==kfs_pose_t.size());
                assert(kfs_pose_R.size()==kfs_pose_R_turb.size());
                assert(kfs_pose_R.size()==kfs_pose_t_turb.size());
                assert(kfs_pose_R.size()==matches_num.size());
                assert(kfs_pose_R.size()==kfs_ts_with_gt.size());
                cout<<"num kf: "<<kfs_ts_with_gt.size()<<endl;

//                fo_matching_res<<query_timestamp<<" "<<to_string(kfs_ts_with_gt.size())<<endl;
                vector<string> res_kfs_ts;
                vector<int> res_match_num;
                vector<vector<Eigen::VectorXf>> res_uvs_cur;
                vector<vector<Eigen::VectorXf>> res_uvs_kf;
                vector<vector<Eigen::VectorXd>> res_pts3d_kf;

                sim->gen_matching(kfs_pose_R,kfs_pose_t,kfs_pose_R_turb,kfs_pose_t_turb,
                                  R_CtoG,p_CinG,matches_num,kfs_ts_with_gt,res_kfs_ts,
                                  res_uvs_cur,res_uvs_kf,res_pts3d_kf);

                
                if(!res_kfs_ts.empty())
                {
                    cout<<"ref_kfs_ts size:"<< res_kfs_ts.size()<<endl;
                    int index=-1;
                    int max=-1;
                    for(int i=0;i<res_kfs_ts.size();i++)
                    {
                         cout<<res_pts3d_kf[i].size()<<endl;
                         if(int(res_pts3d_kf[i].size())>max)
                         {
                             max=res_pts3d_kf[i].size();
                             index=i;
                         }
                    }
                    cout<<"index: "<<index<<endl;
                    assert(index>=0);
                   fo_matching_res<<query_timestamp<<" "<<res_kfs_ts[index]<<" "<<to_string(res_pts3d_kf[index].size())<<endl;
                    int i=index;
                    for(int j=0;j<res_pts3d_kf[i].size();j++)
                    {
                        fo_matching_res<<to_string(res_uvs_cur[i][j](0))<<" "<<to_string(res_uvs_cur[i][j](1))<<" ";
                        fo_matching_res<<to_string(res_uvs_kf[i][j](0))<<" "<<to_string(res_uvs_kf[i][j](1))<<" ";
                        fo_matching_res<<to_string(res_pts3d_kf[i][j](0))<<" "<<to_string(res_pts3d_kf[i][j](1))<<" "<<to_string(res_pts3d_kf[i][j](2))<<" ";
                    }
                    fo_matching_res<<endl;
                }
                 
            }
        }
        fi_matching.close();
        fo_matching_res.close();    

    }

    cout<<"num of gen matching points: "<<sim->featmatching.size()<<endl;
    ros::Publisher pub_sim_matching;
    pub_sim_matching = nh.advertise<sensor_msgs::PointCloud2>("/ov_msckf/points_matching_map", 2);
    ROS_INFO("Publishing: %s", pub_sim_matching.getTopic().c_str());

    return EXIT_SUCCESS;


}
