//
// Created by zzq on 2021/9/11.
//
#ifndef OV_CORE_MATCHBASE_OPTIONS_H
#define OV_CORE_MATCHBASE_OPTIONS_H

#include <string>
#include <vector>
#include <Eigen/Eigen>
using namespace std;
namespace ov_core {


    /**
     * @brief Struct which stores all our filter options
     */
    struct MatchBaseOptions {

      string voc_file = "voc_file";

      string map_feature_type = "Brief";

      string brief_pattern_filename = "brief_pattern_filename";

      string map_file = "map";

      string map_path = "path";

      bool show_image = false;

      bool use_prior_map = true;

      Eigen::MatrixXd intrinsics;

      bool is_fisheye = false;

      vector<Eigen::MatrixXd> intrinsics_vec;

      std::vector<double> PriorMap_intrinsics_cam0;
      std::vector<double> PriorMap_intrinsics_cam1;
      std::vector<double> PriorMap_intrinsics_cam2;
      std::vector<double> PriorMap_intrinsics_cam3;

      double keyframe_cov_ori;
      double keyframe_cov_pos;

      std::string DetectAndMatch_URL;
      std::string DetectAndMatch_img_save_path;
        
    };


}

#endif //OV_CORE_MATCHBASE_OPTIONS_H
