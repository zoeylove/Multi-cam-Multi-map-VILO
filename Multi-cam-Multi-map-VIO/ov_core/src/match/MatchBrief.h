//
// Created by zzq on 2020/10/11.
//

#ifndef SRC_MATCHBRIEF_H
#define SRC_MATCHBRIEF_H
#include "match/MatchBase.h"

namespace ov_core{

    class BriefExtractor
    {
      public:
      virtual void operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const;
      BriefExtractor(const std::string &pattern_file);
      BriefExtractor(){};
      DVision::BRIEF m_brief;
    };



    class MatchBrief: public MatchBase {

    public:
        MatchBrief(MatchBaseOptions &options): MatchBase(options)
        {
          // loadVocabulary(_options.voc_file);

          if(_options.use_prior_map)
          {
            loadPriorMap(_options.map_file, _options.PriorMap_intrinsics_cam0, _options.PriorMap_intrinsics_cam1, _options.PriorMap_intrinsics_cam2, _options.PriorMap_intrinsics_cam3, _options.keyframe_cov_ori, _options.keyframe_cov_pos);
          }
          
          // extractor = BriefExtractor(_options.brief_pattern_filename);
        }


    void loadVocabulary(string voc_file) override;

    void loadPriorMap(string map_file, std::vector<double> PriorMap_intrinsics_cam0, std::vector<double>PriorMap_intrinsics_cam1, std::vector<double> PriorMap_intrinsics_cam2, std::vector<double> PriorMap_intrinsics_cam3, double keyframe_cov_ori, double keyframe_cov_pos) override;

    void ExtractFeatureAndDescriptor(Keyframe& kf) override;

    bool DetectLoop(Keyframe& kf) override;

    void MatchingWithLoop(Keyframe& kf) override;

    void Json_parse(Keyframe& kf0,Keyframe& kf1,Keyframe& kf2, json json_);



    private:
    
    BriefDatabase _db;

    BriefVocabulary* voc;

    BriefExtractor extractor;

    };
}



#endif //SRC_MATCHBRIEF_H
