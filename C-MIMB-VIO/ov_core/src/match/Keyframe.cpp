//
// Created by zzq on 2020/10/10.
//

#include "Keyframe.h"
using namespace ov_core;

template <typename Derived>
static void reduceVector(vector<Derived> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

Keyframe::Keyframe(double _time_stamp, size_t _index, cv::Mat& img,size_t camid,vector<cv::Point2f> &_point_2d_uv,
                   vector<cv::Point2f> &_point_2d_norm,vector<size_t> &_point_id,Eigen::VectorXd intrinsics): time_stamp(_time_stamp),index(_index)
{
                       cam_id=camid;
    _intrinsics=intrinsics;
    image=img.clone();
    point_2d_uv = _point_2d_uv;
    point_2d_norm = _point_2d_norm;
    point_id=_point_id;
    loop_info << 0, 0, 0, 0, 0, 0, 0, 0;
    has_loop=false;
    loop_index = -1;
  //  computeWindowBRIEFPoint();

}

Keyframe::Keyframe(double _time_stamp, int _index, std::size_t camid, cv::Mat &img, VectorXd intrinsics) {
    cam_id=camid;
    _intrinsics=intrinsics;
    image=img.clone();

    //TONY modify
    std::stringstream ss;
    ss << std::setprecision(19) << _time_stamp;
    std::string new_string_time = ss.str();
    new_string_time.at(0) = camid +1 +'0';
    double new_timestamp = stod(new_string_time);
    time_stamp = new_timestamp;
    index=_index;

}


// load previous Keyframe
Keyframe::Keyframe(double _time_stamp, int _index, Eigen::Matrix<double,7,1> &pos1, Eigen::Matrix<double,7,1> &pos2,
                   cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1 > &_loop_info,
                   vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm,vector<BRIEF::bitset> &_brief_descriptors,
                   Eigen::VectorXd intrinsics)
{
    time_stamp = _time_stamp;
    index = _index;
    cam_id=-1;
    _intrinsics=intrinsics;

    //for keyframe load from map, we assume VIO frame is World frame
    _Pose_KFinVIO= new PoseJPL();
    _Pose_KFinVIO->set_value(pos2);
    _Pose_KFinVIO->set_fej(pos2);
    _Pose_KFinWorld =new PoseJPL();
    _Pose_KFinWorld->set_value(pos2);
    _Pose_KFinWorld->set_fej(pos2);
    Eigen::Matrix<double,7,1> relative_pose;
    relative_pose<<0,0,0,1,0,0,0;
    _Pose_VIOtoWorld=new PoseJPL();
    _Pose_VIOtoWorld->set_value(relative_pose);
    _Pose_VIOtoWorld->set_value(relative_pose);

    image = _image.clone();
    
    if (_loop_index != -1)
        has_loop = true;
    else
        has_loop = false;
    loop_index = _loop_index;
    loop_info = _loop_info;
    has_fast_point = false;
    sequence = 0;
    keypoints = _keypoints;
    keypoints_norm = _keypoints_norm;

    brief_descriptors = _brief_descriptors;
}


void KF_BriefExtractor::operator() (const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const
{
    m_brief.compute(im, keys, descriptors);
}

void Keyframe::get_matches(Keyframe* loop_kf, vector<cv::Point2f>& matched_loop_kf_uv, vector<cv::Point2f>& matched_loop_kf_uv_norm,
                        vector<cv::Point3f>& matched_loop_kf_3dpt, vector<cv::Point2f>& matched_query_kf_uv, vector<cv::Point2f>& matched_query_kf_uv_norm)
{
  for(int i = 0; i < (int)brief_descriptors.size(); i++)
    {
        cv::Point2f pt(0.f, 0.f);
        cv::Point2f pt_norm(0.f, 0.f);
        cv::Point2f best_pt;
        int bestDist = 128;
        int bestIndex = -1;
        for(int j = 0; j < (int)loop_kf->brief_descriptors.size(); j++)
        {

            int dis = HammingDis(brief_descriptors[i], loop_kf->brief_descriptors[j]);
            if(dis < bestDist)
            {
                bestDist = dis;
                bestIndex = j;
            }
        }
        if (bestIndex != -1 && bestDist < 50)
        {
          matched_loop_kf_uv.push_back(loop_kf->keypoints[bestIndex].pt);
          matched_loop_kf_uv_norm.push_back(loop_kf->keypoints_norm[bestIndex].pt);
          cv::Point3f pt_3d = cv::Point3f(loop_kf->keypoints_3d[bestIndex](0),loop_kf->keypoints_3d[bestIndex](0),loop_kf->keypoints_3d[bestIndex](0));
          matched_loop_kf_3dpt.push_back(pt_3d);
          matched_query_kf_uv.push_back(keypoints[i].pt);
          matched_query_kf_uv_norm.push_back(keypoints_norm[i].pt);
        }
  
    }
}


bool Keyframe::searchInAera(const BRIEF::bitset window_descriptor,
                            const std::vector<BRIEF::bitset> &descriptors_old,
                            const std::vector<cv::KeyPoint> &keypoints_old,
                            const std::vector<cv::KeyPoint> &keypoints_old_norm,
                            cv::Point2f &best_match,
                            cv::Point2f &best_match_norm, size_t &id)
{
    cv::Point2f best_pt;
    int bestDist = 128;
    int bestIndex = -1;
    for(int i = 0; i < (int)descriptors_old.size(); i++)
    {

        int dis = HammingDis(window_descriptor, descriptors_old[i]);
        if(dis < bestDist)
        {
            bestDist = dis;
            bestIndex = i;
        }
    }
    if (bestIndex != -1 && bestDist < 50)
    {
        best_match = keypoints_old[bestIndex].pt;
        best_match_norm = keypoints_old_norm[bestIndex].pt;
        id=bestIndex;
        return true;
    }
    else
        return false;
}

void Keyframe::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
                                std::vector<cv::Point2f> &matched_2d_old_norm,
                                std::vector<uchar> &status,
                                const std::vector<BRIEF::bitset> &descriptors_old,
                                const std::vector<cv::KeyPoint> &keypoints_old,
                                const std::vector<cv::KeyPoint> &keypoints_old_norm,
                                std::vector<size_t> &matched_id_old)
{
    for(int i = 0; i < (int)window_brief_descriptors.size(); i++)
    {
        cv::Point2f pt(0.f, 0.f);
        cv::Point2f pt_norm(0.f, 0.f);
        size_t id=-1;
        if (searchInAera(window_brief_descriptors[i], descriptors_old, keypoints_old, keypoints_old_norm, pt, pt_norm,id))
        {
            status.push_back(1);
        }
        else
        {
            status.push_back(0);
        }

        matched_2d_old.push_back(pt);
        matched_2d_old_norm.push_back(pt_norm);
        matched_id_old.push_back(id);
    }

}


bool Keyframe::FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
                                      const std::vector<cv::Point2f> &matched_2d_old_norm,
                                      vector<uchar> &status,Eigen::VectorXd intrinsics_old_kf,Eigen::MatrixXd &Fund)
{
    int n = (int)matched_2d_cur_norm.size();
    for (int i = 0; i < n; i++)
        status.push_back(0);
    if (n >= 8)
    {

        cv::Mat F;
        cout<<"before findFundamentalMat"<<endl;
        F=cv::findFundamentalMat(matched_2d_cur_norm, matched_2d_old_norm, cv::FM_RANSAC, 3.0, 0.99, status);
        cout<<"after findFundamentalMat"<<endl;
        cv::cv2eigen(F,Fund);
        return true;
    }
    return false;
}

bool Keyframe::RecoverRelativePose(const std::vector<cv::Point2f> &matched_2d_cur_norm,
                                   const std::vector<cv::Point2f> &matched_2d_old_norm, Eigen::Matrix3d &K,
                                   vector<uchar> &status, Eigen::Matrix3d &R_cur_to_old,
                                   Eigen::Vector3d &p_cur_in_old)
{
    int n = (int)matched_2d_cur_norm.size();
    for (int i = 0; i < n; i++)
        status.push_back(0);
    if (n >= 8) {
        double fx_cur = K(0, 0);
        double fy_cur = K(1, 1);
        double cx_cur = K(0, 2);
        double cy_cur = K(1, 2);
        double fx_old = K(0, 0);
        double fy_old = K(1, 1);
        double cx_old = K(0, 2);
        double cy_old = K(1, 2);
        vector<cv::Point2f> tmp_cur(n), tmp_old(n);
        for (int i = 0; i < (int) matched_2d_cur_norm.size(); i++) {

            double tmp_x, tmp_y;
            tmp_x = fx_cur * matched_2d_cur_norm[i].x + cx_cur;
            tmp_y = fy_cur * matched_2d_cur_norm[i].y + cy_cur;
            tmp_cur[i] = cv::Point2f(tmp_x, tmp_y);


            tmp_x = fx_old * matched_2d_old_norm[i].x + cx_old;
            tmp_y = fy_old * matched_2d_old_norm[i].y + cy_old;
            tmp_old[i] = cv::Point2f(tmp_x, tmp_y);
        }
        cv::Mat K_cv;
        cv::eigen2cv(K,K_cv);
        cv::Mat ess;
        cout<<"before findEssentialMat"<<endl;
        status.clear();
        ess=cv::findEssentialMat(tmp_cur,tmp_old,K_cv,cv::RANSAC,0.99,1,status);
        cout<<"status size: "<<status.size()<<" tmp_cur size:"<<tmp_cur.size()<<endl;
        cout<<"after findEssentialMat"<<endl;
        cv::Mat R, t;
        status.clear();
        cv::recoverPose(ess, tmp_cur, tmp_old, K_cv, R, t, status);
        cout<<"status size: "<<status.size()<<" tmp_cur size:"<<tmp_cur.size()<<endl;
        cout<<"after recoverPose"<<endl;
        cv::cv2eigen(R, R_cur_to_old);
        cv::cv2eigen(t, p_cur_in_old);
        return true;
    }
    return false;
}

bool Keyframe::PnPRANSAC2d(const vector<cv::Point2f> &matched_2d_cur,
                       const std::vector<cv::Point3f> &matched_3d_kf,
                       Eigen::Vector3d &PnP_p_loopIncur, Eigen::Matrix3d &PnP_R_loopTocur,int thred,Eigen::Matrix3d R_kf_to_G)
{
    double fx = 1.0;
    double fy = 1.0;
    double cx = 0.0;
    double cy = 0.0;
    
    std::vector<int> cam_ids;
    
    opengv::points_t points;
    opengv::bearingVectors_t bearing_vectors;

    opengv::rotations_t cam_rotations;
    opengv::translations_t cam_translations;
    points.resize(matched_2d_cur.size());
    bearing_vectors.resize(matched_2d_cur.size());
    cam_ids.resize(matched_2d_cur.size());
    cam_rotations.resize(1);
    cam_translations.resize(1);

    cam_rotations[0] = Eigen::Matrix3d::Identity();
    cam_translations[0] = Eigen::Vector3d::Zero();
    
    cout<<"PnPRansac2d_in"<<endl;
    cout<<"match_3d_kf.size: "<<matched_3d_kf.size()<<endl;
    
    // project 2d uv to 3d image coordinate
    for (int i=0; i<matched_2d_cur.size(); i++){
        Eigen::Vector2d kp(matched_2d_cur.at(i).x, matched_2d_cur.at(i).y);
        kp[0] = (kp[0] - cx) / fx;
        kp[1] = (kp[1] - cy) / fy;
        bearing_vectors.at(i)[0] = kp[0];
        bearing_vectors.at(i)[1] = kp[1];
        bearing_vectors.at(i)[2] = 1;
        bearing_vectors[i].normalize();
        Eigen::Vector3d landmark_position(matched_3d_kf.at(i).x,matched_3d_kf.at(i).y,matched_3d_kf.at(i).z);
        points[i] = landmark_position;
        cam_ids.at(i) = 0;
    }

   bool random_seed_ = false;
   opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(
       bearing_vectors, cam_ids, points, cam_translations, cam_rotations); 
   opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;

//    std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
//     absposeproblem_ptr(
//         new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(adapter,
//         opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P,
//         random_seed_));
//     ransac.sac_model_ = absposeproblem_ptr;
//     ransac.threshold_ = thred;
//     //Hard code 
//     ransac.max_iterations_ = 5;
//     bool ransac_success = ransac.computeModel();
  
    double mypitch = 0;
    double mxroll = 0;
    Eigen::Vector3d mEula;
    Matrix3d R_loop_kf, R_relative, R_lidar2cam;
    R_lidar2cam<< 1.0, 0, 0, 0, -1.0, 0, 0, 0, -1;
    R_loop_kf = _Pose_KFinWorld->Rot();
    std::cout<<"p_loop_kif is "<<_Pose_KFinWorld->pos()<<std::endl;
    std::cout<<"R_loop_kf is "<<R_loop_kf<<std::endl;
    R_relative = R_kf_to_G.transpose() *  R_loop_kf;
    std::cout<<"R_relative is "<<R_relative<<std::endl;
    mEula = opengv::absolute_pose::rotationMatrixToEulerAngles(R_relative);
    mxroll = mEula(0);
    mypitch = mEula(1);
    std::cout<<"R_kf_to_G is "<<R_kf_to_G<<std::endl;
    std::cout<<"mxroll is "<<mxroll<<" mypitch is "<<mypitch<<std::endl;
    std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
    absposeproblem_ptr(
        new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(adapter,
        opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::MC2P,mypitch,mxroll,
        random_seed_));
    ransac.sac_model_ = absposeproblem_ptr;
    ransac.threshold_ = 0.05;
    //Hard code 
    ransac.max_iterations_ = 50;
    bool ransac_success = ransac.compute2EModel();
    Eigen::Matrix<double, 3, 4> final_model = ransac.model_coefficients_;

    if (ransac_success){

        PnP_R_loopTocur = final_model.block<3,3>(0,0).transpose();
        PnP_p_loopIncur = -PnP_R_loopTocur*final_model.col(3);
        
        std::cout<<"ransac.inliers_ size is "<<ransac.inliers_.size()<<std::endl;
        if(ransac.inliers_.size()>=thred)
        {
            std::vector<cv::Point2f> matched_2d_cur_new;
            std::vector<cv::Point3f> matched_3d_kf_new;

            cv::Mat r, rvec, t, D, tmp_r;
            tmp_r = (cv::Mat_<double>(3, 3) << PnP_R_loopTocur(0,0), PnP_R_loopTocur(0,1), PnP_R_loopTocur(0,2), \
             PnP_R_loopTocur(1,0), PnP_R_loopTocur(1,1), PnP_R_loopTocur(1,2), PnP_R_loopTocur(2,0), PnP_R_loopTocur(2,1), PnP_R_loopTocur(2,2));
            t = (cv::Mat_<double>(3, 1) << PnP_p_loopIncur.x() , PnP_p_loopIncur.y(), PnP_p_loopIncur.z());
            cv::Rodrigues(tmp_r,rvec);
            cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
            for (int i=0; i<ransac.inliers_.size(); i++)
            {
                matched_2d_cur_new.push_back(matched_2d_cur.at(ransac.inliers_.at(i))); 
                matched_3d_kf_new.push_back(matched_3d_kf.at(ransac.inliers_.at(i))); 
            }
            bool flag;
            solvePnPRefineLM(matched_3d_kf_new,matched_2d_cur_new,K,D,rvec,t,cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, FLT_EPSILON));
            cout<<"t: "<<t<<endl;
            cv::Rodrigues(rvec, r);
            cv::cv2eigen(r, PnP_R_loopTocur);
            cv::cv2eigen(t, PnP_p_loopIncur);
        }
        else
        {
            return false;
        }
        
        std::cout<<"PnP_p_loopIncur is "<<PnP_p_loopIncur.transpose()<<std::endl;
        std::cout<<"PnP_R_loopTocur is "<<PnP_R_loopTocur<<std::endl;
        return true;
    }else{
        cout<<"Error: Ransac failed"<<std::endl;
        return false;
    }
}


bool Keyframe::PnPRANSAC(const vector<cv::Point2f> &matched_2d_cur,
                         const std::vector<cv::Point3f> &matched_3d_kf,
                         Eigen::Vector3d &PnP_p_loopIncur, Eigen::Matrix3d &PnP_R_loopTocur,int thred)
{

    cv::Mat r, rvec, t, D, tmp_r;

    cv::Mat inliers;

    bool flag=false;
    cout<<"PnPRansac_in"<<endl;
    cout<<"match_3d_kf.size: "<<matched_3d_kf.size()<<endl;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
    
    if(matched_3d_kf.size()>=thred)
    {
        flag=solvePnP(matched_3d_kf,matched_2d_cur,K,D,rvec,t,false,cv::SOLVEPNP_EPNP);
        cout<<"PnPRansac_out with flag="<<flag<<endl;
        cout<<"t: "<<t<<endl;
    }
    else
    {
        return false;
    }
    
    
    VectorXd cam_d=_intrinsics;
    std::cout<<"_intrinsics is "<<_intrinsics.transpose()<<std::endl;
    if(flag)
    {
      cv::Rodrigues(rvec, r);
      Matrix3d R_pnp, R_w_c_old;
      cv::cv2eigen(r, PnP_R_loopTocur);
      cv::cv2eigen(t, PnP_p_loopIncur);
      cout<<"PnP_p_loopIncur: "<<PnP_p_loopIncur<<endl;
      cout<<"PnP_R_loopTocur: "<<std::endl<<PnP_R_loopTocur<<endl;
      std::vector<cv::Point2f> projectPoints;
      cv::projectPoints(matched_3d_kf,rvec,t,K,D,projectPoints);
      double error = 0.0;
      for (int i =0; i< projectPoints.size(); i++)
      {
        double diff_x = matched_2d_cur.at(i).x - projectPoints.at(i).x;
        double diff_y = matched_2d_cur.at(i).y - projectPoints.at(i).y;
        double tmp_error = sqrt(diff_x*diff_x + diff_y*diff_y);
        error += tmp_error;
      }
      std::cout<<"Mean error is "<< error/projectPoints.size()<<std::endl;
      for(int i=0;i<matched_3d_kf.size();i++)
      {
          Vector3d kf_3d(double(matched_3d_kf[i].x),double(matched_3d_kf[i].y),double(matched_3d_kf[i].z));
          Vector2d cur_2d(double(matched_2d_cur[i].x),double(matched_2d_cur[i].y));
          Vector3d cur_3d= PnP_R_loopTocur*kf_3d+PnP_p_loopIncur;
          Vector2d uv_norm=Vector2d::Zero();
          uv_norm<<cur_3d[0]/cur_3d[2],cur_3d[1]/cur_3d[2];
          Vector2d uv_dist=Vector2d::Zero();
          double r = std::sqrt(uv_norm(0)*uv_norm(0)+uv_norm(1)*uv_norm(1));
          double r_2 = r*r;
          double r_4 = r_2*r_2;
          double x1 = uv_norm(0)*(1+cam_d(4)*r_2+cam_d(5)*r_4)+2*cam_d(6)*uv_norm(0)*uv_norm(1)+cam_d(7)*(r_2+2*uv_norm(0)*uv_norm(0));
          double y1 = uv_norm(1)*(1+cam_d(4)*r_2+cam_d(5)*r_4)+cam_d(6)*(r_2+2*uv_norm(1)*uv_norm(1))+2*cam_d(7)*uv_norm(0)*uv_norm(1);
          uv_dist(0) = cam_d(0)*x1 + cam_d(2);
          uv_dist(1) = cam_d(1)*y1 + cam_d(3);

          uv_norm=Vector2d::Zero();
          uv_norm<<cur_2d[0],cur_2d[1];

          Vector2d uv_dist_kf=Vector2d::Zero();
          r = std::sqrt(uv_norm(0)*uv_norm(0)+uv_norm(1)*uv_norm(1));
          r_2 = r*r;
          r_4 = r_2*r_2;
          x1 = uv_norm(0)*(1+cam_d(4)*r_2+cam_d(5)*r_4)+2*cam_d(6)*uv_norm(0)*uv_norm(1)+cam_d(7)*(r_2+2*uv_norm(0)*uv_norm(0));
          y1 = uv_norm(1)*(1+cam_d(4)*r_2+cam_d(5)*r_4)+cam_d(6)*(r_2+2*uv_norm(1)*uv_norm(1))+2*cam_d(7)*uv_norm(0)*uv_norm(1);
          uv_dist_kf(0) = cam_d(0)*x1 + cam_d(2);
          uv_dist_kf(1) = cam_d(1)*y1 + cam_d(3);

      }
      return true;
    }
    else
    {
        return false;
    }
}


bool Keyframe::findConnection(Keyframe* old_kf)
{
//    TicToc tmp_t;
    vector<cv::Point2f> matched_2d_cur, matched_2d_old;
    vector<cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm;
//    vector<cv::Point3f> matched_3d;
    vector<size_t> matched_id_cur,matched_id_old;
    vector<uchar> status;

//    matched_3d = point_3d;
    matched_2d_cur = point_2d_uv;
    matched_2d_cur_norm = point_2d_norm;
    matched_id_cur = point_id;

//    TicToc t_match;
#if 0
    if (DEBUG_IMAGE)
	    {
	        cv::Mat gray_img, loop_match_img;
	        cv::Mat old_img = old_kf->image;
	        cv::hconcat(image, old_img, gray_img);
	        cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)point_2d_uv.size(); i++)
	        {
	            cv::Point2f cur_pt = point_2d_uv[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)old_kf->keypoints.size(); i++)
	        {
	            cv::Point2f old_pt = old_kf->keypoints[i].pt;
	            old_pt.x += COL;
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        ostringstream path;
	        path << "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "0raw_point.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	    }
#endif
    cout<<"before searchByBriedfdes"<<endl;
    searchByBRIEFDes(matched_2d_old, matched_2d_old_norm, status, old_kf->brief_descriptors, old_kf->keypoints, old_kf->keypoints_norm, matched_id_old);
    cout<<"after searchByBriedfdes"<<endl;
    reduceVector(matched_2d_cur, status);
    reduceVector(matched_2d_old, status);
    reduceVector(matched_2d_cur_norm, status);
    reduceVector(matched_2d_old_norm, status);
//    reduceVector(matched_3d, status);
    reduceVector(matched_id_cur, status);
    reduceVector(matched_id_old, status);
    cout<<"matched_2d_cur size:"<<matched_2d_cur.size()<<endl;
    cout<<"matched_2d_old size:"<<matched_2d_old.size()<<endl;
    cout<<"matched_2d_cur_norm size:"<<matched_2d_cur_norm.size()<<endl;
    cout<<"matched_2d_old_norm size:"<<matched_2d_old_norm.size()<<endl;
    cout<<"matched_id_cur size:"<<matched_id_cur.size()<<endl;
    cout<<"matched_id_old size:"<<matched_id_old.size()<<endl;

#if 0
    if (DEBUG_IMAGE)
	    {
			int gap = 10;
        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
            cv::Mat gray_img, loop_match_img;
            cv::Mat old_img = old_kf->image;
            cv::hconcat(image, gap_image, gap_image);
            cv::hconcat(gap_image, old_img, gray_img);
            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f cur_pt = matched_2d_cur[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)matched_2d_old.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x += (COL + gap);
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x +=  (COL + gap);
	            cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
	        }

	        ostringstream path, path1, path2;
	        path <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	        /*
	        path1 <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match_1.jpg";
	        cv::imwrite( path1.str().c_str(), image);
	        path2 <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "1descriptor_match_2.jpg";
	        cv::imwrite( path2.str().c_str(), old_img);
	        */

	    }
#endif
    status.clear();
    Eigen::MatrixXd Fund;
    Eigen::Matrix3d R_cur_to_old;
    Eigen::Vector3d p_cur_in_old;


    Eigen::VectorXd intrinsics_old_kf=old_kf->_intrinsics.block(0,0,8,1);
    cout<<"before FundamantalrixRANSAC"<<endl;
    if(FundmantalMatrixRANSAC(matched_2d_cur_norm, matched_2d_old_norm, status,intrinsics_old_kf,Fund))
    {
        reduceVector(matched_2d_cur, status);
        reduceVector(matched_2d_old, status);
        reduceVector(matched_2d_cur_norm, status);
        reduceVector(matched_2d_old_norm, status);
//    reduceVector(matched_3d, status);
        reduceVector(matched_id_cur, status);
        reduceVector(matched_id_old, status);

        //we now only support recovery the relative pose with same instrinsics
        assert(_intrinsics==intrinsics_old_kf);
        cout<<"after assert"<<endl;
        Eigen::Matrix3d K;
        K<<_intrinsics(0),0,_intrinsics(2),0,_intrinsics(1),_intrinsics(3),0,0,1;
        cout<<"get K:"<<K<<endl;
//        Eigen::Matrix3d Essential=K.transpose()*Fund*K;

        bool recover;
        cout<<"before RecoverRelativePose"<<endl;
        status.clear();
        recover=RecoverRelativePose(matched_2d_cur_norm,matched_2d_old_norm,K,status,R_cur_to_old,p_cur_in_old);
        if(recover)
        {
            reduceVector(matched_2d_cur, status);
            reduceVector(matched_2d_old, status);
            reduceVector(matched_2d_cur_norm, status);
            reduceVector(matched_2d_old_norm, status);
//    reduceVector(matched_3d, status);
            reduceVector(matched_id_cur, status);
            reduceVector(matched_id_old, status);
        }
        else
        {
            cout<<"unable to recover relative pose"<<endl;
            return false;
        }

    }


#if 0
    if (DEBUG_IMAGE)
	    {
			int gap = 10;
        	cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
            cv::Mat gray_img, loop_match_img;
            cv::Mat old_img = old_kf->image;
            cv::hconcat(image, gap_image, gap_image);
            cv::hconcat(gap_image, old_img, gray_img);
            cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
	        for(int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f cur_pt = matched_2d_cur[i];
	            cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for(int i = 0; i< (int)matched_2d_old.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x += (COL + gap);
	            cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
	        }
	        for (int i = 0; i< (int)matched_2d_cur.size(); i++)
	        {
	            cv::Point2f old_pt = matched_2d_old[i];
	            old_pt.x +=  (COL + gap) ;
	            cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
	        }

	        ostringstream path;
	        path <<  "/home/tony-ws1/raw_data/loop_image/"
	                << index << "-"
	                << old_kf->index << "-" << "2fundamental_match.jpg";
	        cv::imwrite( path.str().c_str(), loop_match_img);
	    }
#endif
#if 0
#endif
    cout<<"matched_2d_cur.size :"<<(int)matched_2d_cur.size()<<endl;
    if ((int)matched_2d_cur.size() > 20)
    {
        double relative_yaw=R_cur_to_old.eulerAngles(2,1,0)(0);
        if (abs(relative_yaw) < 30.0 && p_cur_in_old.norm() < 20.0)
        {
            cout<<"in if"<<endl;

            Eigen::Vector4d relative_q;
            relative_q=ov_core::rot_2_quat(R_cur_to_old); //in the form of JPL
            has_loop = true;
            loop_index = old_kf->index;
            //in loop_info, we also record the relative_q in the form of JPL (x,y,z,w)
            loop_info << p_cur_in_old.x(), p_cur_in_old.y(), p_cur_in_old.z(),
                    relative_q(0), relative_q(1), relative_q(2), relative_q(3),
                    relative_yaw;
            cout<<"before assert"<<endl;
            assert(matched_id_cur.size()==matched_id_old.size());
            cout<<"after assert"<<endl;

            _matched_id_cur=matched_id_cur;
            _matched_id_old=matched_id_old;
            return true;
        }
    }
    return false;
}


int Keyframe::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b)
{
    BRIEF::bitset xor_of_bitset = a ^ b;
    int dis = xor_of_bitset.count();
    return dis;
}

void Keyframe::getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = vio_T_w_i;
    _R_w_i = vio_R_w_i;
}

void Keyframe::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i)
{
    _T_w_i = T_w_i;
    _R_w_i = R_w_i;
}

void Keyframe::updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    T_w_i = _T_w_i;
    R_w_i = _R_w_i;
}

void Keyframe::updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
{
    vio_T_w_i = _T_w_i;
    vio_R_w_i = _R_w_i;
    T_w_i = vio_T_w_i;
    R_w_i = vio_R_w_i;
}

Eigen::Vector3d Keyframe::getLoopRelativeT()
{
    return Eigen::Vector3d(loop_info(0), loop_info(1), loop_info(2));
}

Eigen::Vector4d Keyframe::getLoopRelativeQ()
{
    return Eigen::Vector4d(loop_info(4), loop_info(5), loop_info(6), loop_info(3));
}

double Keyframe::getLoopRelativeYaw()
{
    return loop_info(7);
}

void Keyframe::updateLoop(Eigen::Matrix<double, 8, 1 > &_loop_info)
{
    if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
    {
        loop_info = _loop_info;
    }
}

KF_BriefExtractor::KF_BriefExtractor(const std::string &pattern_file)
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
