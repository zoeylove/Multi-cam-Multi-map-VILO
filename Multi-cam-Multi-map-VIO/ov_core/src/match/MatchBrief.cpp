#include "MatchBrief.h"
// #include "core/VioManager.h"
// #include "core/VioManagerOptions.h"
// using namespace ov_msckf;
using namespace ov_core;

void MatchBrief::loadVocabulary(string voc_file)
{
  cout<<"loading vocabulary..."<<endl;

  voc = new BriefVocabulary(voc_file);

  _db.setVocabulary(*voc,false,0);

  cout<<"done!"<<endl;
}

void MatchBrief::loadPriorMap(string map_file, std::vector<double> PriorMap_intrinsics_cam0, std::vector<double>PriorMap_intrinsics_cam1, std::vector<double> PriorMap_intrinsics_cam2, std::vector<double> PriorMap_intrinsics_cam3, double keyframe_cov_ori, double keyframe_cov_pos)
{
  cout<<"load map..."<<map_file<<endl;
  cout<<"PriorMap_intrinsics_cam0: ";
  for(int i=0;i<PriorMap_intrinsics_cam0.size();i++)
  {
    cout<<PriorMap_intrinsics_cam0[i]<<" ";
  }
  cout<<endl;
  cout<<"PriorMap_intrinsics_cam1: ";
  for(int i=0;i<PriorMap_intrinsics_cam1.size();i++)
  {
    cout<<PriorMap_intrinsics_cam1[i]<<" ";
  }
  cout<<endl;
  cout<<"PriorMap_intrinsics_cam2: ";
  for(int i=0;i<PriorMap_intrinsics_cam2.size();i++)
  {
    cout<<PriorMap_intrinsics_cam2[i]<<" ";
  }
  cout<<endl;
  cout<<"PriorMap_intrinsics_cam3: ";
  for(int i=0;i<PriorMap_intrinsics_cam3.size();i++)
  {
    cout<<PriorMap_intrinsics_cam3[i]<<" ";
  }
  cout<<endl;
  
  ifstream f;
  f.open(map_file.data());
  assert(f.is_open());
  string str1,res1,str2,res2;
  
  vector<PoseJPL*> poses,querry_poses,single_camera_poses;
  poses.clear();
  
  int map_seq=0;
  int max_kf_id = 0;
  double kf_ts;
  while(getline(f,str1))  //img_name,image_id,tx,ty,tz,qx,qy,qz,qw
  {
    stringstream line1(str1);

    line1>>res1;
    string img_name = res1;
    line1>>res1;
    int id=stoi(res1);
    line1>>res1;
    float tx=atof(res1.c_str());
    line1>>res1;
    float ty=atof(res1.c_str());
    line1>>res1;
    float tz=atof(res1.c_str());
    line1>>res1;
    float qx=atof(res1.c_str());;
    line1>>res1;
    float qy=atof(res1.c_str());
    line1>>res1;
    float qz=atof(res1.c_str());
    line1>>res1;
    float qw=atof(res1.c_str());
    //here we load the quaternion in the form of Hamilton,we need to tranform it into JPL
    Quaterniond q1(qw,qx,qy,qz);  
    Quaterniond q1_JPL(q1.toRotationMatrix().transpose());
    Eigen::Matrix<double,7,1> q_kfInw;
    PoseJPL* pose=new PoseJPL();
    q_kfInw<<q1_JPL.x(),q1_JPL.y(),q1_JPL.z(),q1_JPL.w(),tx, ty, tz;
    
    int pos = img_name.find_last_of('/'); // match windows results
    int pos2 = img_name.find(".png");
    int match_cam_id = 0;
    std::string kf_cam_name = img_name.substr(0, pos);
    if (kf_cam_name == "stereo_left")
        match_cam_id = 0;
    else if (kf_cam_name == "stereo_right")
        match_cam_id = 1;
    else if (kf_cam_name == "left")
        match_cam_id = 2;
    else if (kf_cam_name == "right")
        match_cam_id = 3;
    else
    {
        match_cam_id = -1;
        std::cout<<img_name<<endl;
        std::cout<<kf_cam_name<<endl;
        printf(RED "Wrong kf camera name\n" RESET);
        continue;
    }

    std::string kf_string = img_name.substr(pos+1, pos2 - pos-1); 
    kf_string.at(0) = match_cam_id+1+'0';
    double kf_ts = stod(kf_string);
    kf_ts=floor(kf_ts*1000000000)/1000000000.0;  //nine decimals
    Keyframe* kf = new Keyframe();
    kf->time_stamp = kf_ts;
    kf->index = id;   // Image id
    kf->_Pose_KFinWorld = new PoseJPL();           
    kf->_Pose_KFinWorld->set_value(q_kfInw);
    kf->_Pose_KFinWorld->set_fej(q_kfInw);
    kf->_Pose_KFinWorld->set_linp(q_kfInw);
    kf->P=Matrix<double,6,6>::Identity();


    // TODO give the cov param 
    kf->P.block(0,0,3,3)=MatrixXd::Identity(3,3)*keyframe_cov_ori;
    kf->P.block(3,3,3,3)=MatrixXd::Identity(3,3)*keyframe_cov_pos;

    // Set new keyframe;
    max_kf_id++;
    kf->cam_id=match_cam_id;  //matched camera id 

    VectorXd intrinsics;
    intrinsics.conservativeResize(9,1);

    if (match_cam_id == 0)
      intrinsics << PriorMap_intrinsics_cam0[0], PriorMap_intrinsics_cam0[1], PriorMap_intrinsics_cam0[2], 
                  PriorMap_intrinsics_cam0[3], PriorMap_intrinsics_cam0[4], PriorMap_intrinsics_cam0[5],
                  PriorMap_intrinsics_cam0[6], PriorMap_intrinsics_cam0[7], PriorMap_intrinsics_cam0[8];
    else if (match_cam_id == 1)
      intrinsics << PriorMap_intrinsics_cam1[0], PriorMap_intrinsics_cam1[1], PriorMap_intrinsics_cam1[2], 
                  PriorMap_intrinsics_cam1[3], PriorMap_intrinsics_cam1[4], PriorMap_intrinsics_cam1[5],
                  PriorMap_intrinsics_cam1[6], PriorMap_intrinsics_cam1[7], PriorMap_intrinsics_cam1[8];
    else if (match_cam_id == 2)
      intrinsics << PriorMap_intrinsics_cam2[0], PriorMap_intrinsics_cam2[1], PriorMap_intrinsics_cam2[2], 
                  PriorMap_intrinsics_cam2[3], PriorMap_intrinsics_cam2[4], PriorMap_intrinsics_cam2[5],
                  PriorMap_intrinsics_cam2[6], PriorMap_intrinsics_cam2[7], PriorMap_intrinsics_cam2[8];
    else if (match_cam_id == 3)
      intrinsics << PriorMap_intrinsics_cam3[0], PriorMap_intrinsics_cam3[1], PriorMap_intrinsics_cam3[2], 
                  PriorMap_intrinsics_cam3[3], PriorMap_intrinsics_cam3[4], PriorMap_intrinsics_cam3[5],
                  PriorMap_intrinsics_cam3[6], PriorMap_intrinsics_cam3[7], PriorMap_intrinsics_cam3[8];

    kf->_intrinsics=intrinsics;
    kf->sequence=map_seq;
    kf->time_stamp=kf_ts;
    kf->image_name=kf_string;

    kfdatabase->update_kf_new(kf);
    
    pose->set_value(q_kfInw);
    poses.push_back(pose);
    if(match_cam_id == 0)
      single_camera_poses.push_back(pose);
  }

  f.close();
}

void MatchBrief::ExtractFeatureAndDescriptor(Keyframe& kf)
{
  cv::Mat img;
  cv::equalizeHist(kf.image, img);

  std::vector<cv::KeyPoint> pts0_ext;
  Grider_FAST::perform_griding(img, pts0_ext, 500, 5, 3, 15, true);

  std::vector<cv::KeyPoint> pts0_ext_norm;
  pts0_ext_norm.resize(pts0_ext.size());

  for(int i=0;i<pts0_ext.size();i++)
  {
    cv::Point2f pt = pts0_ext[i].pt;
    cv::Point2f pt_norm = undistort_point(pt);     
    cv::KeyPoint kp;
    kp.pt = pt_norm; 
    pts0_ext_norm.at(i) = kp;
  }

  kf.keypoints = pts0_ext;
  kf.keypoints_norm = pts0_ext_norm;

  vector<BRIEF::bitset> descriptors;
  extractor(img,pts0_ext,descriptors);
  kf.brief_descriptors = descriptors;

}


bool MatchBrief::DetectLoop(Keyframe& kf)
{
    
    //first query; then add this frame into database!
    QueryResults ret;
    _db.query(kf.brief_descriptors, ret, 4, kfdatabase->size());
    
    cv::Mat compressed_image;
    if (_options.show_image)
    {
        int feature_num = kf.keypoints.size();
        cv::resize(kf.image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        
    }
    // ret[0] is the nearest neighbour's score. threshold change with neighour score
    bool find_loop = false;

    for(int i=0;i<ret.size();i++)
    {
      cout<<ret[i].Score<<" ";
    }
    cout<<endl;
    if(ret.size()==0)
      return false;

    cv::Mat loop_result;
    if (_options.show_image)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    // visual loop result 
    if (_options.show_image)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            Keyframe* tmp = kfdatabase->get_keyframe(tmp_index);
            cv::Mat tmp_image = tmp->image.clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
            cv::imshow("loop_result", loop_result);
            cv::waitKey(20);
        }

    }
    // a good match with its nerghbour
    if (ret.size() >= 1 &&ret[0].Score > 0.05)
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)
            {          
                find_loop = true;
                int tmp_index = ret[i].Id;
                if (_options.show_image )
                {
                    int tmp_index = ret[i].Id;
                    Keyframe* tmp = kfdatabase->get_keyframe(tmp_index);
                    cv::Mat tmp_image = tmp->image.clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                    cv::hconcat(loop_result, tmp_image, loop_result);
                    cv::imshow("loop_result", loop_result);
                    cv::waitKey(20);
                }
            }
            
        }
    
    vector<int> loop_index;
    if (find_loop)
    {
        
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if(ret[i].Score>0.05)
            {
              loop_index.push_back(ret[i].Id);
            }                
        }

        kf.loop_image_id_vec=loop_index;

        return true;
    }
    loop_index.clear();
    match_map.insert({kf.time_stamp,loop_index});

    return false;
    

}

void MatchBrief::MatchingWithLoop(Keyframe& kf)
{
  std::cout <<"In matching with loop"<< std::endl;
  
  for(int i=0;i<kf.loop_image_id_vec.size();i++)
  {
    Keyframe* loop_kf = kfdatabase->get_keyframe(kf.loop_image_id_vec[i]);
    vector<cv::Point2f> matched_loop_kf_uv, matched_loop_kf_uv_norm;
    vector<cv::Point3f> matched_loop_kf_3dpt;
    vector<cv::Point2f> matched_query_kf_uv, matched_query_kf_uv_norm;

    kf.get_matches(loop_kf,matched_loop_kf_uv,matched_loop_kf_uv_norm,matched_loop_kf_3dpt, matched_query_kf_uv, matched_query_kf_uv_norm);

    assert(matched_loop_kf_3dpt.size()==matched_loop_kf_uv.size());
    assert(matched_loop_kf_3dpt.size()==matched_loop_kf_uv_norm.size());
    assert(matched_query_kf_uv_norm.size()==matched_query_kf_uv.size());
    assert(matched_query_kf_uv_norm.size()==matched_loop_kf_3dpt.size());

    if(matched_query_kf_uv.size() < 10)
        continue; //TODO:

    // Do RANSAC outlier rejection (note since we normalized the max pixel error is now in the normalized cords)
    std::vector<uchar> mask_rsc;
    double max_focallength = std::max(camera_k_OPENCV(0,0),camera_k_OPENCV(1,1));
    cv::findFundamentalMat(matched_query_kf_uv_norm, matched_loop_kf_uv_norm, cv::FM_RANSAC, 1/max_focallength, 0.9, mask_rsc);


    int j = 0;
    for (int k = 0; k < int(matched_query_kf_uv.size()); k++)
    {
      if (mask_rsc[k] == 1)
        {
          matched_loop_kf_uv[j] = matched_loop_kf_uv[k];
          matched_loop_kf_uv_norm[j] = matched_loop_kf_uv_norm[k];
          matched_loop_kf_3dpt[j] = matched_loop_kf_3dpt[k];
          matched_query_kf_uv[j] = matched_query_kf_uv[k];
          matched_query_kf_uv_norm[j] = matched_loop_kf_uv_norm[k];
          j++;
        }
    }
    matched_loop_kf_uv.resize(j);
    matched_loop_kf_uv_norm.resize(j);
    matched_loop_kf_3dpt.resize(j);
    matched_query_kf_uv.resize(j);
    matched_query_kf_uv_norm.resize(j);
    
    cout<<"for loop kf "<<i<<": "<<matched_loop_kf_3dpt.size()<<" matches"<<endl;

    loop_kf->matched_point_2d_uv_map.insert({kf.time_stamp,matched_query_kf_uv});
    loop_kf->matched_point_2d_norm_map.insert({kf.time_stamp,matched_query_kf_uv_norm});
    loop_kf->point_2d_uv_map.insert({kf.time_stamp,matched_loop_kf_uv});
    loop_kf->point_2d_uv_norm_map.insert({kf.time_stamp,matched_loop_kf_uv_norm});
    loop_kf->point_3d_map.insert({kf.time_stamp,matched_loop_kf_3dpt});
    loop_kf->point_3d_linp_map.insert({kf.time_stamp,matched_loop_kf_3dpt});
    loop_kf->loop_img_timestamp_vec.push_back(kf.time_stamp);

  }

  //we finish all the matching works, and the match information into match_map.
  match_map.insert({kf.time_stamp,kf.loop_image_id_vec});
}



BriefExtractor::BriefExtractor(const std::string &pattern_file)
{
  // The DVision::BRIEF extractor computes a random pattern by default when
  // the object is created.
  // We load the pattern that we used to build the vocabulary, to make
  // the descriptors compatible with the predefined vocabulary

  // loads the pattern
  cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
  if(!fs.isOpened()) throw string("Could not open file ") + pattern_file;

  vector<int> x1, y1, x2, y2;
  fs["x1"] >> x1;
  fs["x2"] >> x2;
  fs["y1"] >> y1;
  fs["y2"] >> y2;

  m_brief.importPairs(x1, y1, x2, y2);
}

void BriefExtractor::operator() (const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
{
  m_brief.compute(im, keys, descriptors);
}
