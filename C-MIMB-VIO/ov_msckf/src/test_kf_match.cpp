#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "core/RosVisualizer.h"
#include "utils/dataset_reader.h"
#include "utils/parse_ros.h"
#include <dirent.h>
#include <unistd.h>

using namespace ov_msckf;
using namespace cv;
using namespace std;

class Feat{
     
     public:
        Feat():success_triang(false){}
    
        ~Feat(){}
     
     //map<image_timstamp, (id,u,v)>
        map<double,Vector3d> uvs;

        map<double,Vector3d> uvs_norm;

        size_t id;

        Vector3d point_3d_W;

        Vector3d point_3d_A;

        double anchor_img_id=-1;

        bool success_triang;

};

VioManager* sys;
// Buffer data
double time_buffer = -1;
cv::Mat img0_buffer, img1_buffer;


double fx=458.654/2;
double fy=457.296/2;
double cx=367.215/2;
double cy=248.375/2;
double k1=-5.6143027800000002e-02;
double k2=1.3952563200000001e-01;
double k3=-1.2155906999999999e-03;
double k4=-9.7281389999999998e-04;

void callback_monocular(const sensor_msgs::ImageConstPtr& msg0);
//void callback_stereo(const sensor_msgs::CompressedImageConstPtr& msg0, const sensor_msgs::CompressedImageConstPtr& msg1);
void callback_stereo(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1);

void callback_cam(const sensor_msgs::ImageConstPtr& msg);

bool single_triangulation(Feat* feat, map<double, ov_type::PoseJPL *>& image_poses);

bool single_gaussnewton(Feat* feat, map<double, ov_type::PoseJPL *>& image_poses);

double compute_error(map<double, ov_type::PoseJPL *>& image_poses,
                     Feat* feat, double alpha, double beta, double rho);




vector<string> getFiles(string cate_dir)
{
	vector<string> files;//存放文件名
	DIR *dir;
	struct dirent *ptr;
	char base[1000];
    cout<<"in getFiles"<<endl;
 
	if ((dir=opendir(cate_dir.c_str())) == NULL)
        {
		perror("Open dir error...");
                exit(1);
        }
 
	while ((ptr=readdir(dir)) != NULL)
	{
		if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
		        continue;
		else if(ptr->d_type == 8)    ///file
			files.push_back(ptr->d_name);
		else if(ptr->d_type == 10)    ///link file
			continue;
		else if(ptr->d_type == 4)    ///dir
		{
			files.push_back(ptr->d_name);
			/*
		        memset(base,'\0',sizeof(base));
		        strcpy(base,basePath);
		        strcat(base,"/");
		        strcat(base,ptr->d_nSame);
		        readFileList(base);
			*/
		}
	}
	closedir(dir);
 
	//排序，按从小到大排序
	sort(files.begin(), files.end());
	return files;
}


// Main function
int main(int argc, char** argv)
{
      
    ros::init(argc, argv, "test_kf_match");
    ros::NodeHandle nh("~");
    //Create our VIO system
    VioManagerOptions params = parse_ros_nodehandler(nh);
    cout<<"finish load parameters.............."<<endl;
    string filepath="/home/zzq/Code/map_open_vins_ekf_ros/src/open_vins/triangulate_match/JYM_test_20210421/matches/";
    string filepath_3d="/home/zzq/Code/map_open_vins_ekf_ros/src/open_vins/triangulate_match/JYM_test_20210421/matches_3d/";
    // string filepath_imgname="/home/zzq/Code/map_open_vins_ros/src/open_vins/triangulate_match/leftImageTimestamp.txt";
    vector<string> files;
    files=getFiles(filepath);
    cout<<"files size: "<<files.size()<<endl;
    for(int i=0;i<files.size();i++)
    {
        cout<<files[i]<<endl;
    }   

    // //load each file
    vector<int> error_load;
    for(int file_num=0;file_num<files.size();file_num++)
    {

        size_t feature_num=0;
        //feature database
        
        map<size_t,Feat*> feats_db;
        //every image(timestamp) and it corresponding Feature Vector4d(localid,u,v,globalid)
        map<double,vector<Vector4d>> images_feature;

        // sys = new VioManager(params);

        cout<<"finish initialization!..........."<<endl;
        
        ifstream if_match;
        if_match.open((filepath+files[file_num]).data());
        assert(if_match.is_open());
        cout<<"processing "<<files[file_num]<<"....."<<endl;
         
        ofstream of_3d;
        if (boost::filesystem::exists(filepath_3d+files[file_num])) {
            boost::filesystem::remove(filepath_3d+files[file_num]);
            printf(YELLOW "[STATS]: found old file found, deleted...\n" RESET);
        }
        // Create the directory that we will open the file in
        boost::filesystem::path p(filepath_3d+files[file_num]);
        boost::filesystem::create_directories(p.parent_path());


        //image1 timestamp with vector contains matched images. For each matched images, map the timestamp and matched points
        map<double,vector<map<double,vector<VectorXd>>>> image_matches;
        map<double,PoseJPL*> image_poses;

        map<double,string> image_name_map;

        string str,result;
        int line_number=1;
        ifstream if_pose;
        string pose_file_name="/home/zzq/Code/map_open_vins_ekf_ros/src/open_vins/ov_data/euroc_mav/MH_01_easy_cam.txt";
        ifstream if_imgname;
        double timestamp1,timestamp2;
        int matches_num;
        vector<VectorXd> matched_point;
        string image_name1,image_name2;
        bool success_load=true;
        bool skip=false;
        
        while(getline(if_match,str))
        {
         if(line_number%2==1) //单数行  imagename1,imagename2,match number
         {
             stringstream line(str);
            line>>result; //image name1;
            image_name1=result;  //0000000010.png for example
            string image_name_front;
            string image_name_back;
            image_name_front=image_name1.substr(0,10);
            image_name_front=image_name_front+".";
            image_name_back=image_name1.substr(10,4); //两位小数 for kaist, 4 for euroc
            string image_name_final1=image_name_front+image_name_back;
            timestamp1=stod(image_name_final1);

              cout<<"timestamp1: "<<to_string(timestamp1)<<" image_name1: "<<image_name1<<endl;
            image_name_map.insert({timestamp1,image_name1});

            line>>result; //image_name2
            image_name2=result;
            image_name_front=image_name2.substr(0,10);
            image_name_front=image_name_front+".";
            image_name_back=image_name2.substr(10,4);
            string image_name_final2=image_name_front+image_name_back;
            timestamp2=stod(image_name_final2);

               cout<<"timestamp2: "<<to_string(timestamp2)<<" iamge_name2: "<<image_name2<<endl;
            
            image_name_map.insert({timestamp2,image_name2});

            line>>result;
            matches_num=stoi(result);
            vector<map<double,vector<VectorXd>>> matched_image;
           
            if(image_poses.find(timestamp1)==image_poses.end())//load pose
            {
                if_pose.open(pose_file_name.data());
                assert(if_pose.is_open());
                string str1,res1;
                float tx,ty,tz,qx,qy,qz,qw;
                int flag=0;
                while(getline(if_pose,str1))
                {
                    stringstream line1(str1);
                    line1>>res1;
                    double timestamp=stod(res1); //timestamp;
                    timestamp= floor(timestamp*10000)/10000.0; //保留2位小数for kaist,4 for euroc
                    string timestamp_str=to_string(timestamp).substr(0,15); //13forkaist,15foreuroc
                    
                    if(timestamp_str==image_name_final1)
                    {
                        line1>>res1;
                        tx=atof(res1.c_str());
                        line1>>res1;
                        ty=atof(res1.c_str());
                        line1>>res1;
                        tz=atof(res1.c_str());
                        line1>>res1;
                        qx=atof(res1.c_str());
                        line1>>res1;
                        qy=atof(res1.c_str());
                        line1>>res1;
                        qz=atof(res1.c_str());
                        line1>>res1;
                        qw=atof(res1.c_str());

                        Quaterniond q1(qw,qx,qy,qz);  
                        Quaterniond q1_JPL(q1.toRotationMatrix().transpose());
                        Eigen::Matrix<double,7,1> q_kfInw;
                        q_kfInw<<q1_JPL.x(),q1_JPL.y(),q1_JPL.z(),q1_JPL.w(),tx, ty, tz;
                        PoseJPL* pose=new PoseJPL();
                        pose->set_value(q_kfInw);
                        image_poses.insert({timestamp1,pose});
                        flag=1;
                        break;
                    }

                }
                if_pose.close();

                if(flag!=1)
                {
                    success_load=false;
                    break;
                }
                assert(flag==1);
            }
            if(image_poses.find(timestamp2)==image_poses.end())//load pose
            {
                if_pose.open(pose_file_name.data());
                assert(if_pose.is_open());
                string str1,res1;
                float tx,ty,tz,qx,qy,qz,qw;
                int flag=0;
                while(getline(if_pose,str1))
                {
                    stringstream line1(str1);
                    line1>>res1;
                    double timestamp=stod(res1); //timestamp;
                    
                    timestamp= floor(timestamp*10000)/10000.0; //保留4位小数
                    string timestamp_str=to_string(timestamp).substr(0,15);
                    if(timestamp_str==image_name_final2)
                    {
                        line1>>res1;
                        tx=atof(res1.c_str());
                        line1>>res1;
                        ty=atof(res1.c_str());
                        line1>>res1;
                        tz=atof(res1.c_str());
                        line1>>res1;
                        qx=atof(res1.c_str());
                        line1>>res1;
                        qy=atof(res1.c_str());
                        line1>>res1;
                        qz=atof(res1.c_str());
                        line1>>res1;
                        qw=atof(res1.c_str());

                        Quaterniond q1(qw,qx,qy,qz);  
                        Quaterniond q1_JPL(q1.toRotationMatrix().transpose());
                        Eigen::Matrix<double,7,1> q_kfInw;
                        q_kfInw<<q1_JPL.x(),q1_JPL.y(),q1_JPL.z(),q1_JPL.w(),tx, ty, tz;
                        PoseJPL* pose=new PoseJPL();
                        pose->set_value(q_kfInw);
                        image_poses.insert({timestamp2,pose});
                        flag=1;
                        break;  
                    }

                }
                if_pose.close();

                if(flag!=1)
                {
                    success_load=false;
                    break;
                }
                assert(flag==1);
            }
            if(success_load==false)
            {
                error_load.push_back(file_num);
                //boost::filesystem::remove(filepath_3d+files[file_num]);
                line_number++;
                skip=true;
                continue;
            }
            else
            {
                image_name_map.insert({timestamp1,image_name1});
                image_name_map.insert({timestamp2,image_name2});
                if(image_matches.find(timestamp1)==image_matches.end())
                {
                    image_matches.insert({timestamp1,matched_image});
                }
                vector<Vector4d> points;
                if(images_feature.find(timestamp1)==images_feature.end())
                {
                    images_feature.insert({timestamp1,points});
                }
                if(images_feature.find(timestamp2)==images_feature.end())
                {
                    images_feature.insert({timestamp2,points});
                }
            }
            

         }
         else if(line_number%2==0)//偶数行
         {
           if(skip==true)
           {
               line_number++;
               continue;
           }
           stringstream line(str);
           matched_point.clear();
           for(int j=0;j<matches_num;j++)
           {
               
               line>>result; //id1;
               double id1=stod(result);
               line>>result;
               double uv1_x=stod(result);
               line>>result;
               double uv1_y=stod(result);
               line>>result;
               double id2=stod(result);
               line>>result;
               double uv2_x=stod(result);
               line>>result;
               double uv2_y=stod(result);
               Matrix<double,7,1> match;
               match<<id1,uv1_x,uv1_y,id2,uv2_x,uv2_y,-1;  //-1 means this match is not used yet
               matched_point.push_back(match);
           }
 
           map<double,vector<VectorXd>> m;
           m.insert({timestamp2,matched_point});
           assert(image_matches.find(timestamp1)!=image_matches.end());
           image_matches.at(timestamp1).push_back(m);
         }
         
         
         line_number++;
        }
        if_match.close();
        

        int num=0;
        for(auto image: image_matches)
        {
            double root_image=image.first;
            auto matches_root=image.second;
            for(int i=0;i<matches_root.size();i++)
            {
              auto m=matches_root[i];
              for(auto pair: m) //for every matched image with root image
              {
                  double match_image_ts=pair.first;
                  cout<<"image "<<to_string(root_image)<<" match with image "<<to_string(match_image_ts)<<endl;
                  vector<VectorXd> match_points=pair.second;
                  for(int j=0;j<match_points.size();j++) //for each match point
                  {
                      Vector3d pt1;
                      Vector3d pt2;
                      pt1<<match_points[j](0,0),match_points[j](1,0),match_points[j](2,0);
                      pt2<<match_points[j](3,0),match_points[j](4,0),match_points[j](5,0);
                      assert(images_feature.find(root_image)!=images_feature.end());
                      assert(images_feature.find(match_image_ts)!=images_feature.end());
                      bool flag=false;
                      for(int k=0;k<images_feature[root_image].size();k++)
                      {
                          Vector3d point=images_feature[root_image][k].block(0,0,3,1);

                          if(images_feature[root_image][k].block(0,0,3,1)==pt1) //the featrue pt1 has already in images_feature[root_image]
                          {
                              flag=true;
                              size_t id=images_feature[root_image][k](3,0);
                              assert(feats_db.find(id)!=feats_db.end());
                              auto feat=feats_db.find(id);
                              feat->second->uvs.insert({match_image_ts,pt2});
                              Vector4d p=Vector4d::Zero();
                              p<<pt2(0),pt2(1),pt2(2),double(id);
                              images_feature[match_image_ts].push_back(p);

                              break;
                          }

                      }
                      if(flag==false)  //the feature pt1 is not in images_feature[root_image]
                      {
                          
                          bool mark=false;
                          for(int k=0;k<images_feature[match_image_ts].size();k++)
                          {
                              if(images_feature[match_image_ts][k].block(0,0,3,1)==pt2)
                              {
                                  //although feature pt1 is not in root_image, pt2 has already in match_image_ts, 
                                  //which means this feature has matches before.
                                  //For example: for landmark1, it was observed by image1 and image3 before (when root image is image1),
                                  //             now, our root image is image2, we find that this landmark1 is new for image2, however, 
                                  //             it is already observed in image3 before, then we should not create the new feat.
                                  //             instead, we need to make feature_link with image1,image2,image3.  
                                  size_t id=images_feature[match_image_ts][k](3,0);
                                  assert(feats_db.find(id)!=feats_db.end());
                                  auto feat=feats_db.find(id);
                                  feat->second->uvs.insert({root_image,pt1});
                                  Vector4d p=Vector4d::Zero();
                                  p<<pt1(0),pt1(1),pt1(2),double(id);
                                  images_feature[root_image].push_back(p);
                                  mark=true;
                                  break;
                              }
                          }
                          if(mark==false) //assign a new feature
                          {
                            Feat* feat=new Feat();
                            feat->id=feature_num;
                            feat->uvs.insert({root_image,pt1});
                            feat->uvs.insert({match_image_ts,pt2});
                            Vector4d p=Vector4d::Zero();
                            p<<pt1(0),pt1(1),pt1(2),double(feat->id);
                            images_feature[root_image].push_back(p);
                            p=Vector4d::Zero();
                            p<<pt2(0),pt2(1),pt2(2),double(feat->id);
                            images_feature[match_image_ts].push_back(p);
                            feats_db.insert({feature_num,feat});
                            feature_num++;
                          }
                          

                      }

                  }
              }

            }


        }
        cout<<"feature database size: "<<feats_db.size()<<endl;

       cv::Matx33d camK;
        camK(0, 0) = fx;
        camK(0,1)=0;
        camK(0,2)=cx;
        camK(1,0)=0;
        camK(1,1)=fy;
        camK(1,2)=cy;
        camK(2,0)=0;
        camK(2,1)=0;
        camK(2,2)=1;
       cv::Vec4d camD;
        camD(0) = k1;
        camD(1) = k2;
        camD(2) = k3;
        camD(3) = k4;
       //undistort uv point to get uv_norm
       for(auto feat:feats_db)
       {           
           for(auto uv:feat.second->uvs)
           {
              cv::Point2f pt;
              pt.x=float(uv.second(1));
              pt.y=float(uv.second(2));
              cv::Mat mat(1, 2, CV_32F);
              mat.at<float>(0, 0) = pt.x;
              mat.at<float>(0, 1) = pt.y;
            
              mat = mat.reshape(2); // Nx1, 2-channel
            // Undistort it!
            cv::fisheye::undistortPoints(mat, mat, camK, camD);
            // Construct our return vector
            cv::Point2f pt_out;
            mat = mat.reshape(1); // Nx2, 1-channel
            pt_out.x = mat.at<float>(0, 0);
            pt_out.y = mat.at<float>(0, 1);
            Vector3d uv_norm(uv.second(0),double(pt_out.x),double(pt_out.y));
            feat.second->uvs_norm.insert({uv.first,uv_norm});
            
            
           }

       }

       string image_dir="/media/zzq/SAMSUNG/DataSet/kaist/urban38/urban38-pankyo_img/urban38-pankyo/image/stereo_left/";

        int success_num=0;
        for(auto feat: feats_db)
        {
            bool success=single_triangulation(feat.second,image_poses);
            if(!success)
            {
               
               continue;
            }
            success=single_gaussnewton(feat.second,image_poses);
            if(!success)
            {
                
                continue;
            }
            feat.second->success_triang=true;
            success_num++;
            
        }
        cout<<"feature_num: "<<feats_db.size()<<endl;
        cout<<"success_num: "<<success_num<<endl;

        
        //check reproject error
        int success_filter=success_num;
        for(auto feat: feats_db)
        {
                Feat* f=feat.second;
                if(f->success_triang)
                {
                   Vector3d pt_w=f->point_3d_W;
                    for(auto uv: f->uvs)
                    {
                        double image_id=uv.first;
                        Matrix3d R_W_C=image_poses.at(image_id)->Rot();
                        Vector3d p_W_C=image_poses.at(image_id)->pos();
                        Vector3d pt_c=R_W_C.transpose()*(pt_w-p_W_C);
                        pt_c(0)=pt_c(0)/pt_c(2);
                        pt_c(1)=pt_c(1)/pt_c(2);
                        pt_c(2)=pt_c(2)/pt_c(2);

                        // Calculate distorted coordinates for fisheye
                        double r = std::sqrt(pt_c(0)*pt_c(0)+pt_c(1)*pt_c(1));
                        double theta = std::atan(r);
                        double theta_d = theta+k1*std::pow(theta,3)+k2*std::pow(theta,5)+k3*std::pow(theta,7)+k4*std::pow(theta,9);

                        // Handle when r is small (meaning our xy is near the camera center)
                        double inv_r = (r > 1e-8)? 1.0/r : 1.0;
                        double cdist = (r > 1e-8)? theta_d * inv_r : 1.0;

                        // Calculate distorted coordinates for fisheye
                        double x1 = pt_c(0)*cdist;
                        double y1 = pt_c(1)*cdist;
                        pt_c(0) = fx*x1 + cx;
                        pt_c(1) = fy*y1 + cy;
                        

                        double error_x=abs(uv.second(1)-pt_c(0));
                        double error_y=abs(uv.second(2)-pt_c(1));
                        if(error_x>3||error_y>3)
                        {
                            f->success_triang=false;
                            success_filter--;
                            break;
                        }
                        

                    }
                }
                
           
            
        }
        cout<<"****************************feature_num: "<<feats_db.size()<<endl;
        cout<<"****************************success_num: "<<success_num<<endl;
        cout<<"****************************success_filter: "<<success_filter<<endl;


        // Open our statistics file!
        of_3d.open(filepath_3d+files[file_num], std::ofstream::out | std::ofstream::app);
        assert(of_3d.is_open());
        for(auto image: image_poses)
        {
            double image_ts=image.first;
            of_3d<<image_name_map[image_ts]<<" ";
            int num=0;
            for(auto feat : feats_db)
            {
                Feat* f=feat.second;
                if(f->success_triang)
                {
                    if(f->uvs.find(image_ts)!=f->uvs.end())
                    {
                        num++;
                    }
                }
            }
            of_3d<<num<<endl;
            for(auto feat : feats_db)
            {
                Feat* f=feat.second;
                if(f->success_triang)
                {
                    if(f->uvs.find(image_ts)!=f->uvs.end())
                    {
                        of_3d<<to_string(int(f->uvs[image_ts](0)))<<" "<<
                        f->uvs[image_ts](1)<<" "<<f->uvs[image_ts](2)<<" "<<f->point_3d_W(0)<<" "<<f->point_3d_W(1)<<" "<<f->point_3d_W(2)<<" ";
                    }
                }
            }
            of_3d<<endl;
        }
        of_3d.close();
        if(success_filter==0)
        {
            boost::filesystem::remove(filepath_3d+files[file_num]);
        }

        // Finally delete our system
        // delete sys;
  
    } 
    cout<<"error load file nums: "<<error_load.size()<<endl;
    for(int i=0;i<error_load.size();i++)
    {
        cout<<"error load file : "<<error_load[i]<<endl;
    }
     
     
        // Done!
        return EXIT_SUCCESS; 
      

}

bool single_triangulation(Feat* feat, map<double, ov_type::PoseJPL *>& image_poses) {


    // Total number of measurements
    // Also set the first measurement to be the anchor frame
    int total_meas = 0;
    total_meas=feat->uvs.size();
    
    // Our linear system matrices
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2*total_meas, 3);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(2*total_meas, 1);

    // Location in the linear system matrices
    size_t c = 0;

    // Get the position of the anchor pose
    double anchor_id=0;
    for(auto image: feat->uvs)
    {
        anchor_id=image.first;
        break;
    }
    feat->anchor_img_id=anchor_id;
    PoseJPL* anchor_pose=image_poses.at(anchor_id);
    Eigen::Matrix<double,3,3> R_GtoA = anchor_pose->Rot().transpose();
    Eigen::Matrix<double,3,1> p_AinG = anchor_pose->pos();
    // Loop through each image for this feature
    for(auto image: feat->uvs_norm)
    {
        // Get the position of this image in the global
        Eigen::Matrix<double, 3, 3> R_GtoCi = image_poses.at(image.first)->Rot().transpose();
        Eigen::Matrix<double, 3, 1> p_CiinG = image_poses.at(image.first)->pos();

        // Convert current position relative to anchor
        Eigen::Matrix<double,3,3> R_AtoCi;
        R_AtoCi.noalias() = R_GtoCi*R_GtoA.transpose();
        Eigen::Matrix<double,3,1> p_CiinA;
        p_CiinA.noalias() = R_GtoA*(p_CiinG-p_AinG);
        // Get the UV coordinate normal
        Eigen::Matrix<double, 3, 1> b_i;
        
        b_i << feat->uvs_norm.at(image.first)(1), feat->uvs_norm.at(image.first)(2), 1;
        b_i = R_AtoCi.transpose() * b_i;
        b_i = b_i / b_i.norm();
        Eigen::Matrix<double,2,3> Bperp = Eigen::Matrix<double,2,3>::Zero();
        Bperp << -b_i(2, 0), 0, b_i(0, 0), 0, b_i(2, 0), -b_i(1, 0);

        // Append to our linear system
        A.block(2 * c, 0, 2, 3) = Bperp;
        b.block(2 * c, 0, 2, 1).noalias() = Bperp * p_CiinA;
        c++;
        
    }

    // Solve the linear system
    Eigen::MatrixXd p_f = A.colPivHouseholderQr().solve(b);

    // Check A and p_f
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd singularValues;
    singularValues.resize(svd.singularValues().rows(), 1);
    singularValues = svd.singularValues();
    //condition number: max_eigenvalue/min_eigenvalue. by zzq
    double condA = singularValues(0, 0) / singularValues(singularValues.rows() - 1, 0);

    // If we have a bad condition number, or it is too close
    // Then set the flag for bad (i.e. set z-axis to nan)
    //eruoc 1000 0.25 40
    //kaist 5000 0.25 150
    if (std::abs(condA) > 1000|| p_f(2,0) < 0.25 || p_f(2,0) > 40 || std::isnan(p_f.norm())) {
        return false;
    }

    // Store it in our feature object
    feat->point_3d_A= p_f;
    feat->point_3d_W = R_GtoA.transpose()*feat->point_3d_A + p_AinG;
    
    Vector3d uv_norm=Vector3d::Zero();
    uv_norm<<p_f(0)/p_f(2),p_f(1)/p_f(2),1;
    Matrix3d R_W_A=image_poses[feat->anchor_img_id]->Rot();
    Vector3d p_W_A=image_poses[feat->anchor_img_id]->pos();
    Vector3d p_A=R_W_A.transpose()*(feat->point_3d_W-p_W_A);

    return true;

}


bool single_gaussnewton(Feat* feat, map<double, ov_type::PoseJPL *>& image_poses) {

    //Get into inverse depth
    double rho = 1/feat->point_3d_A(2);
    double alpha = feat->point_3d_A(0)/feat->point_3d_A(2);
    double beta = feat->point_3d_A(1)/feat->point_3d_A(2);

    // Optimization parameters
    double lam = 1e-3;
    double eps = 10000;
    int runs = 0;

    // Variables used in the optimization
    bool recompute = true;
    Eigen::Matrix<double,3,3> Hess = Eigen::Matrix<double,3,3>::Zero();
    Eigen::Matrix<double,3,1> grad = Eigen::Matrix<double,3,1>::Zero();

    // Cost at the last iteration
    double cost_old = compute_error(image_poses,feat,alpha,beta,rho);

    // Get the position of the anchor pose
    Eigen::Matrix<double,3,3> R_GtoA = image_poses.at(feat->anchor_img_id)->Rot().transpose();
    Eigen::Matrix<double,3,1> p_AinG = image_poses.at(feat->anchor_img_id)->pos();

    // Loop till we have either
    // 1. Reached our max iteration count
    // 2. System is unstable
    // 3. System has converged
    while (runs < 20 && lam <  1e10 && eps > 1e-6) {

        // Triggers a recomputation of jacobians/information/gradients
        if (recompute) {

            Hess.setZero();
            grad.setZero();

            double err = 0;
            for(auto image:feat->uvs_norm)
            {
                // Get the position of this image in the global
                Eigen::Matrix<double, 3, 3> R_GtoCi = image_poses.at(image.first)->Rot().transpose();
                Eigen::Matrix<double, 3, 1> p_CiinG = image_poses.at(image.first)->pos();

                // Convert current position relative to anchor
                Eigen::Matrix<double,3,3> R_AtoCi;
                R_AtoCi.noalias() = R_GtoCi*R_GtoA.transpose();
                Eigen::Matrix<double,3,1> p_CiinA;
                p_CiinA.noalias() = R_GtoA*(p_CiinG-p_AinG);
                Eigen::Matrix<double,3,1> p_AinCi;
                p_AinCi.noalias() = -R_AtoCi*p_CiinA;

                double hi1 = R_AtoCi(0, 0) * alpha + R_AtoCi(0, 1) * beta + R_AtoCi(0, 2) + rho * p_AinCi(0, 0);
                    double hi2 = R_AtoCi(1, 0) * alpha + R_AtoCi(1, 1) * beta + R_AtoCi(1, 2) + rho * p_AinCi(1, 0);
                    double hi3 = R_AtoCi(2, 0) * alpha + R_AtoCi(2, 1) * beta + R_AtoCi(2, 2) + rho * p_AinCi(2, 0);
                    // Calculate jacobian
                    double d_z1_d_alpha = (R_AtoCi(0, 0) * hi3 - hi1 * R_AtoCi(2, 0)) / (pow(hi3, 2));
                    double d_z1_d_beta = (R_AtoCi(0, 1) * hi3 - hi1 * R_AtoCi(2, 1)) / (pow(hi3, 2));
                    double d_z1_d_rho = (p_AinCi(0, 0) * hi3 - hi1 * p_AinCi(2, 0)) / (pow(hi3, 2));
                    double d_z2_d_alpha = (R_AtoCi(1, 0) * hi3 - hi2 * R_AtoCi(2, 0)) / (pow(hi3, 2));
                    double d_z2_d_beta = (R_AtoCi(1, 1) * hi3 - hi2 * R_AtoCi(2, 1)) / (pow(hi3, 2));
                    double d_z2_d_rho = (p_AinCi(1, 0) * hi3 - hi2 * p_AinCi(2, 0)) / (pow(hi3, 2));
                    Eigen::Matrix<double, 2, 3> H;
                    H << d_z1_d_alpha, d_z1_d_beta, d_z1_d_rho, d_z2_d_alpha, d_z2_d_beta, d_z2_d_rho;
                    // Calculate residual
                    Eigen::Matrix<float, 2, 1> z;
                    z << hi1 / hi3, hi2 / hi3;
                    Eigen::Matrix<float,2,1> uv_norm;
                    uv_norm<<feat->uvs_norm.at(image.first)(1),feat->uvs_norm.at(image.first)(2);
                    Eigen::Matrix<float, 2, 1> res = uv_norm - z;

                    // Append to our summation variables
                    err += std::pow(res.norm(), 2);
                    grad.noalias() += H.transpose() * res.cast<double>();
                    Hess.noalias() += H.transpose() * H;
            }
            
        }

        // Solve Levenberg iteration
        Eigen::Matrix<double,3,3> Hess_l = Hess;
        for (size_t r=0; r < (size_t)Hess.rows(); r++) {
            Hess_l(r,r) *= (1.0+lam);
        }

        Eigen::Matrix<double,3,1> dx = Hess_l.colPivHouseholderQr().solve(grad);
        //Eigen::Matrix<double,3,1> dx = (Hess+lam*Eigen::MatrixXd::Identity(Hess.rows(), Hess.rows())).colPivHouseholderQr().solve(grad);

        // Check if error has gone down
        double cost = compute_error(image_poses,feat,alpha+dx(0,0),beta+dx(1,0),rho+dx(2,0));

        // Check if converged
        if (cost <= cost_old && (cost_old-cost)/cost_old < 1e-6) {
            alpha += dx(0, 0);
            beta += dx(1, 0);
            rho += dx(2, 0);
            eps = 0;
            break;
        }

        // If cost is lowered, accept step，and shrink lam to make next step larger
        // Else inflate lambda to make next step smaller (try to make more stable)
        if (cost <= cost_old) {
            recompute = true;
            cost_old = cost;
            alpha += dx(0, 0);
            beta += dx(1, 0);
            rho += dx(2, 0);
            runs++;
            lam = lam/10;
            eps = dx.norm();
        } else {
            recompute = false;
            lam = lam*10;
            continue;
        }
    }

    // Revert to standard, and set to all
    feat->point_3d_A(0) = alpha/rho;
    feat->point_3d_A(1) = beta/rho;
    feat->point_3d_A(2) = 1/rho;

    // Get tangent plane to x_hat
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(feat->point_3d_A);
    Eigen::MatrixXd Q = qr.householderQ();

    // Max baseline we have between poses
    double base_line_max = 0.0;

    // Check maximum baseline
    // Loop through each camera for this feature
    //TODO: What the geometry meaning of base_line?
    for(auto const& image:feat->uvs_norm)
    {
        Eigen::Matrix<double,3,1> p_CiinG  = image_poses.at(image.first)->pos();
            // Convert current position relative to anchor
            Eigen::Matrix<double,3,1> p_CiinA = R_GtoA*(p_CiinG-p_AinG);
            // Dot product camera pose and nullspace
            double base_line = ((Q.block(0,1,3,2)).transpose() * p_CiinA).norm();
            if (base_line > base_line_max) base_line_max = base_line;
    }

    // Check if this feature is bad or not
    // 1. If the feature is too close
    // 2. If the feature is invalid
    // 3. If the baseline ratio is large
    //euroc  0.25 40 40
    //kaist 0.25 150 500
    if(feat->point_3d_A(2) < 0.25
        || feat->point_3d_A(2) > 40
        || (feat->point_3d_A.norm() / base_line_max) > 40
        || std::isnan(feat->point_3d_A.norm())) {
        return false;
    }

    // Finally get position in global frame
    feat->point_3d_W = R_GtoA.transpose()*feat->point_3d_A+ p_AinG;
    return true;

}

double compute_error(map<double, ov_type::PoseJPL *>& image_poses,
                     Feat* feat, double alpha, double beta, double rho) {

    // Total error
    double err = 0;

    // Get the position of the anchor pose
    Eigen::Matrix<double,3,3> R_GtoA = image_poses.at(feat->anchor_img_id)->Rot().transpose();
    Eigen::Matrix<double,3,1> p_AinG = image_poses.at(feat->anchor_img_id)->pos();

    // Loop through each image for this feature
    for(auto image:feat->uvs_norm)
    {
         // Get the position of this image in the global
        Eigen::Matrix<double, 3, 3> R_GtoCi = image_poses.at(image.first)->Rot().transpose();
        Eigen::Matrix<double, 3, 1> p_CiinG = image_poses.at(image.first)->pos();

        // Convert current position relative to anchor
        Eigen::Matrix<double,3,3> R_AtoCi;
        R_AtoCi.noalias() = R_GtoCi*R_GtoA.transpose();
        Eigen::Matrix<double,3,1> p_CiinA;
        p_CiinA.noalias() = R_GtoA*(p_CiinG-p_AinG);
        Eigen::Matrix<double,3,1> p_AinCi;
        p_AinCi.noalias() = -R_AtoCi*p_CiinA;

        // Middle variables of the system
            //alpha: x/z in anchor ;beta:y/z in anchor; rho: 1/z in anchor
            double hi1 = R_AtoCi(0, 0) * alpha + R_AtoCi(0, 1) * beta + R_AtoCi(0, 2) + rho * p_AinCi(0, 0);
            double hi2 = R_AtoCi(1, 0) * alpha + R_AtoCi(1, 1) * beta + R_AtoCi(1, 2) + rho * p_AinCi(1, 0);
            double hi3 = R_AtoCi(2, 0) * alpha + R_AtoCi(2, 1) * beta + R_AtoCi(2, 2) + rho * p_AinCi(2, 0);

        // Calculate residual
            Eigen::Matrix<float, 2, 1> z;
            z << hi1 / hi3, hi2 / hi3;
            Eigen::Matrix<float,2,1> uv_norm;
            uv_norm<<feat->uvs_norm.at(image.first)(1),feat->uvs_norm.at(image.first)(2);
            Eigen::Matrix<float, 2, 1> res = uv_norm - z;
            // Append to our summation variables
            err += pow(res.norm(), 2);
    }

    return err;

}

void callback_monocular(const sensor_msgs::ImageConstPtr& msg0) {


    // Get the image
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Fill our buffer if we have not
    if(img0_buffer.rows == 0) {
        time_buffer = cv_ptr->header.stamp.toSec();
        img0_buffer = cv_ptr->image.clone();
        return;
    }

    // send it to our VIO system
    sys->feed_measurement_monocular(time_buffer, img0_buffer, 0);

    // move buffer forward
    time_buffer = cv_ptr->header.stamp.toSec();
    img0_buffer = cv_ptr->image.clone();

    cout.precision(15);
    cout<<"time : "<<cv_ptr->header.stamp.toSec()<<endl;

}



void callback_stereo(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1) {

    // Get the image
    cout<<"in callback_stereo"<<endl;
    cv_bridge::CvImageConstPtr cv_ptr0;
    try {
        cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
//        cv_ptr0 = cv_bridge::toCvCopy(msg0, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Get the image
    cv_bridge::CvImageConstPtr cv_ptr1;
    try {
        cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
//        cv_ptr1 = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    // Fill our buffer if we have not
    cout<<"fill buffer"<<endl;
    if(img0_buffer.rows == 0 || img1_buffer.rows == 0) {
        time_buffer = cv_ptr0->header.stamp.toSec();
        img0_buffer = cv_ptr0->image.clone();
        time_buffer = cv_ptr1->header.stamp.toSec();
        img1_buffer = cv_ptr1->image.clone();
        return;
    }

    // send it to our VIO system
    cout<<"before test_feature_match"<<endl;
    sys->test_feature_match(time_buffer, img0_buffer, img1_buffer, 0, 1);

   


    // move buffer forward
    time_buffer = cv_ptr0->header.stamp.toSec();
    img0_buffer = cv_ptr0->image.clone();
    time_buffer = cv_ptr1->header.stamp.toSec();
    img1_buffer = cv_ptr1->image.clone();

}




