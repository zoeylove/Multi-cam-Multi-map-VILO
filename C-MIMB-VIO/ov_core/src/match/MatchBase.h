//
// Created by zzq on 2020/10/11.
//

#ifndef SRC_MATCHBASE_H
#define SRC_MATCHBASE_H

#include "match/KeyframeDatabase.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#include "ThirdParty/DBoW/TemplatedDatabase.h"
#include "ThirdParty/DBoW/TemplatedVocabulary.h"
#include "track/Grider_FAST.h"
#include "match/MatchBaseOptions.h"
#include <queue>
#include <fstream>
#include <sstream>
#include <string>
#include "types/PoseJPL.h"
#include <Eigen/Geometry>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <curl/curl.h>

#include<unistd.h>
#include <iconv.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <csignal>

using namespace ov_core;
using namespace DBoW2;



using namespace cv;
using json = nlohmann::json;

namespace ov_core{

    class MapFeatureType
    {
      public:
       enum Type
      {
        Brief,
        ORB,
        UNKNOWN
      };

      static inline Type from_string(const std::string& feat_type) {
            if(feat_type=="Brief") return Brief;
            if(feat_type=="ORB") return ORB;
            return UNKNOWN;
      }
    };
    
   

    class MatchBase {

    public:
    MatchBase(ov_core::MatchBaseOptions &options): kfdatabase(new KeyframeDatabase()){
      _options = options;

      cv::Matx33d tempK;
      tempK(0, 0) = _options.intrinsics(0);
      tempK(0, 1) = 0;
      tempK(0, 2) = _options.intrinsics(2);
      tempK(1, 0) = 0;
      tempK(1, 1) = _options.intrinsics(1);
      tempK(1, 2) = _options.intrinsics(3);
      tempK(2, 0) = 0;
      tempK(2, 1) = 0;
      tempK(2, 2) = 1;
      camera_k_OPENCV = tempK;
      // Distortion parameters
      cv::Vec4d tempD;
      tempD(0) = _options.intrinsics(4);
      tempD(1) = _options.intrinsics(5);
      tempD(2) = _options.intrinsics(6);
      tempD(3) = _options.intrinsics(7);
      camera_d_OPENCV = tempD;

      _is_fisheye = _options.is_fisheye;

      _last_keyframe = new Keyframe();

    }

    void feed_image(ov_core::Keyframe* img)
    {
      std::unique_lock<std::mutex> lck(img_buffer_mutex);
      img_buffer.push(img);

      return;
    }

  
    // 图片转Base64
    std::string encodeToBase64(const cv::Mat& img) {
        std::vector<uchar> buffer;
        cv::imencode(".jpg", img, buffer);
        return std::string(buffer.begin(), buffer.end());
    }

    int preNUm(unsigned char byte) {
        unsigned char mask = 0x80;
        int num = 0;
        for (int i = 0; i < 8; i++) {
            if ((byte & mask) == mask) {
                mask = mask >> 1;
                num++;
            } else {
                break;
            }
        }
        return num;
    }
    
    void SigHandle(int sig);
    
    void DetectAndMatch(const std::string& DetectAndMatch_URL, const std::string& DetectAndMatch_img_save_path);

    void keyframesCheck(ov_core::Keyframe* kf0,ov_core::Keyframe* kf1,ov_core::Keyframe* kf2,ov_core::Keyframe* kf3);

    virtual void loadVocabulary(string voc_file)=0;

    // virtual void loadPriorMap(string map_file)=0;
    virtual void loadPriorMap(string map_file,
                          std::vector<double> PriorMap_intrinsics_cam0,
                          std::vector<double> PriorMap_intrinsics_cam1,
                          std::vector<double> PriorMap_intrinsics_cam2,
                          std::vector<double> PriorMap_intrinsics_cam3,
                          double keyframe_cov_ori,
                          double keyframe_cov_pos) = 0;

    virtual void ExtractFeatureAndDescriptor(Keyframe& kf)=0;

    virtual bool DetectLoop(Keyframe& kf)=0;

    virtual void MatchingWithLoop(Keyframe& kf)=0;

    void Json_parse(Keyframe& kf0,Keyframe& kf1,Keyframe& kf2, json json_);

    cv::Point2f undistort_point_new(cv::Point2f pt_in, VectorXd tmp_intrinsics)
    {
      if (tmp_intrinsics(8))
      {
        cv::Mat mat(1, 2, CV_32F);
        mat.at<float>(0, 0) = pt_in.x;
        mat.at<float>(0, 1) = pt_in.y;
        mat = mat.reshape(2); // Nx1, 2-channel
        // Undistort it!
        cv::Matx33d tempK;
        tempK(0, 0) = tmp_intrinsics(0);
        tempK(0, 1) = 0;
        tempK(0, 2) = tmp_intrinsics(2);
        tempK(1, 0) = 0;
        tempK(1, 1) = tmp_intrinsics(1);
        tempK(1, 2) = tmp_intrinsics(3);
        tempK(2, 0) = 0;
        tempK(2, 1) = 0;
        tempK(2, 2) = 1;
        // Distortion parameters
        cv::Vec4d tempD;
        tempD(0) = tmp_intrinsics(4);
        tempD(1) = tmp_intrinsics(5);
        tempD(2) = tmp_intrinsics(6);
        tempD(3) = tmp_intrinsics(7);
        cv::fisheye::undistortPoints(mat, mat, tempK, tempD);
        // Construct our return vector
        cv::Point2f pt_out;
        mat = mat.reshape(1); // Nx2, 1-channel
        pt_out.x = mat.at<float>(0, 0);
        pt_out.y = mat.at<float>(0, 1);
        return pt_out;
      }
      else
      {
        cv::Mat mat(1, 2, CV_32F);
        mat.at<float>(0, 0) = pt_in.x;
        mat.at<float>(0, 1) = pt_in.y;
        mat = mat.reshape(2); // Nx1, 2-channel
        // Undistort it!
        cv::Matx33d tempK;
        tempK(0, 0) = tmp_intrinsics(0);
        tempK(0, 1) = 0;
        tempK(0, 2) = tmp_intrinsics(2);
        tempK(1, 0) = 0;
        tempK(1, 1) = tmp_intrinsics(1);
        tempK(1, 2) = tmp_intrinsics(3);
        tempK(2, 0) = 0;
        tempK(2, 1) = 0;
        tempK(2, 2) = 1;
        // Distortion parameters
        cv::Vec4d tempD;
        tempD(0) = tmp_intrinsics(4);
        tempD(1) = tmp_intrinsics(5);
        tempD(2) = tmp_intrinsics(6);
        tempD(3) = tmp_intrinsics(7);
        cv::undistortPoints(mat, mat, tempK, tempD);
        // Construct our return vector
        cv::Point2f pt_out;
        mat = mat.reshape(1); // Nx2, 1-channel
        pt_out.x = mat.at<float>(0, 0);
        pt_out.y = mat.at<float>(0, 1);
        return pt_out;
      }
    }
    cv::Point2f undistort_point_new(cv::Point2f pt_in, int cam_id)
    {
      if(_is_fisheye)
      {
        cv::Mat mat(1, 2, CV_32F);
        mat.at<float>(0, 0) = pt_in.x;
        mat.at<float>(0, 1) = pt_in.y;
        mat = mat.reshape(2); // Nx1, 2-channel
        // Undistort it!
        Eigen::MatrixXd tmp_intrinsics = _options.intrinsics_vec.at(cam_id);
        cv::Matx33d tempK;
        tempK(0, 0) = tmp_intrinsics(0);
        tempK(0, 1) = 0;
        tempK(0, 2) = tmp_intrinsics(2);
        tempK(1, 0) = 0;
        tempK(1, 1) = tmp_intrinsics(1);
        tempK(1, 2) = tmp_intrinsics(3);
        tempK(2, 0) = 0;
        tempK(2, 1) = 0;
        tempK(2, 2) = 1;
        // Distortion parameters
        cv::Vec4d tempD;
        tempD(0) = tmp_intrinsics(4);
        tempD(1) = tmp_intrinsics(5);
        tempD(2) = tmp_intrinsics(6);
        tempD(3) = tmp_intrinsics(7);
        cv::fisheye::undistortPoints(mat, mat, tempK, tempD);
        // Construct our return vector
        cv::Point2f pt_out;
        mat = mat.reshape(1); // Nx2, 1-channel
        pt_out.x = mat.at<float>(0, 0);
        pt_out.y = mat.at<float>(0, 1);
        return pt_out;
      }
      else
      {
        cv::Mat mat(1, 2, CV_32F);
        mat.at<float>(0, 0) = pt_in.x;
        mat.at<float>(0, 1) = pt_in.y;
        mat = mat.reshape(2); // Nx1, 2-channel
        // Undistort it!
        Eigen::MatrixXd tmp_intrinsics = _options.intrinsics_vec.at(cam_id);
        cv::Matx33d tempK;
        tempK(0, 0) = tmp_intrinsics(0);
        tempK(0, 1) = 0;
        tempK(0, 2) = tmp_intrinsics(2);
        tempK(1, 0) = 0;
        tempK(1, 1) = tmp_intrinsics(1);
        tempK(1, 2) = tmp_intrinsics(3);
        tempK(2, 0) = 0;
        tempK(2, 1) = 0;
        tempK(2, 2) = 1;
        // Distortion parameters
        cv::Vec4d tempD;
        tempD(0) = tmp_intrinsics(4);
        tempD(1) = tmp_intrinsics(5);
        tempD(2) = tmp_intrinsics(6);
        tempD(3) = tmp_intrinsics(7);
        cv::undistortPoints(mat, mat, tempK, tempD);
        // Construct our return vector
        cv::Point2f pt_out;
        mat = mat.reshape(1); // Nx2, 1-channel
        pt_out.x = mat.at<float>(0, 0);
        pt_out.y = mat.at<float>(0, 1);
        return pt_out;
      }
    }
    cv::Point2f undistort_point(cv::Point2f pt_in)
    {
      if(_is_fisheye)
      {
        cv::Mat mat(1, 2, CV_32F);
        mat.at<float>(0, 0) = pt_in.x;
        mat.at<float>(0, 1) = pt_in.y;
        mat = mat.reshape(2); // Nx1, 2-channel
        // Undistort it!
        cv::fisheye::undistortPoints(mat, mat, camera_k_OPENCV, camera_d_OPENCV);
        // Construct our return vector
        cv::Point2f pt_out;
        mat = mat.reshape(1); // Nx2, 1-channel
        pt_out.x = mat.at<float>(0, 0);
        pt_out.y = mat.at<float>(0, 1);
        return pt_out;
      }
      else
      {
        cv::Mat mat(1, 2, CV_32F);
        mat.at<float>(0, 0) = pt_in.x;
        mat.at<float>(0, 1) = pt_in.y;
        mat = mat.reshape(2); // Nx1, 2-channel
        // Undistort it!
        cv::undistortPoints(mat, mat, camera_k_OPENCV, camera_d_OPENCV);
        // Construct our return vector
        cv::Point2f pt_out;
        mat = mat.reshape(1); // Nx2, 1-channel
        pt_out.x = mat.at<float>(0, 0);
        pt_out.y = mat.at<float>(0, 1);
        return pt_out;
      }
    }

    KeyframeDatabase* get_interval_kfdatabase()
    {
      return kfdatabase;
    }



    bool finish_matching_thread(double ts)
    {
      std::unique_lock<std::mutex> lck(match_map_mutex);
      std::stringstream ss;
      ss << std::setprecision(19) << ts;
      std::string left_string_time = ss.str();
      std::string right_string_time = ss.str();
      left_string_time.at(0) = 3+'0';
      right_string_time.at(0) = 4+'0';
      double left_timestamp = stod(left_string_time);
      double right_timestamp = stod(right_string_time);

      if(match_map.find(ts)!=match_map.end())
      {
        if(kfdatabase->get_keyframe(ts)->checked)
        {
          cout<<"finish check"<<endl;
          return true;
        }
      }
      else if (match_map.find(left_timestamp)!=match_map.end())
      {
        if(kfdatabase->get_keyframe(left_timestamp)->checked)
        {
          cout<<"finish check"<<endl;
          return true;
        }
      }
      else if (match_map.find(right_timestamp)!=match_map.end())
      {
        if(kfdatabase->get_keyframe(right_timestamp)->checked)
        {
          cout<<"finish check"<<endl;
          return true;
        }
      }

      return false;
    }


    vector<int> get_matchkf_ids(double ts)
    {
      vector<int> res;
      res.clear();
      if(match_map.find(ts)!=match_map.end())
      {
        return match_map.at(ts);
      }
      else
      {
        return res;
      }
    }

    Keyframe* get_kf(double ts, bool remove = false)
    {
      return kfdatabase->get_keyframe(ts,remove);
    }

    vector<Keyframe*> get_matchkfs(double ts_tmp)
    {
      std::vector<double> tmp_timevector;
      tmp_timevector.push_back(ts_tmp);
      std::stringstream ss;
      ss << std::setprecision(19) << ts_tmp;
      std::string left_string_time = ss.str();
      std::string right_string_time = ss.str();
      left_string_time.at(0) = 3+'0';
      right_string_time.at(0) = 4+'0';
      double left_timestamp = stod(left_string_time);
      double right_timestamp = stod(right_string_time);
      tmp_timevector.push_back(left_timestamp);
      tmp_timevector.push_back(right_timestamp);
      
      vector<Keyframe*> res;
      res.clear();
      std::unique_lock<std::mutex> lck(match_map_mutex);
      for (auto ts : tmp_timevector)
      {
        if(match_map.find(ts)!=match_map.end())
        {
          std::cout<<"get_matchkfs ts is "<<to_string(ts)<<std::endl;
          vector<int> match_ids;
          match_ids = match_map.at(ts);
          for(int i=0; i<match_ids.size(); i++)
          {
            Keyframe* kf = kfdatabase->get_keyframe(match_ids[i]);
            assert(kf!=nullptr);
            res.push_back(kf);
          }
        }
      }
      return res;
    }
    
    MatchBaseOptions _options;
    // condition_variable sig_buffer;



    protected:
    
    std::mutex img_buffer_mutex; 
    std::mutex match_map_mutex;
    std::queue<ov_core::Keyframe*> img_buffer;
    // std::queue<std::shared_ptr<ov_core::Keyframe>> img_buffer;
    KeyframeDatabase *kfdatabase;
    //* current image ts v.s. matched map kf ids
    std::map<double,vector<int>> match_map;

    ov_core::Keyframe* _last_keyframe;

    
    /// Camera intrinsics in OpenCV format
    cv::Matx33d camera_k_OPENCV;
    /// Camera distortion in OpenCV format
    cv::Vec4d camera_d_OPENCV;
    ///
    bool _is_fisheye;

    boost::posix_time::ptime relinear_t1,relinear_t2;

    /// Mutex lock for detect and match
    std::mutex mtx_detect;

    };

}



#endif //SRC_MATCHBASE_H
