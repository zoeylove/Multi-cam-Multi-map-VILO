//
// Created by zzq on 2020/10/11.
//

#ifndef SRC_MATCHROBUST_H
#define SRC_MATCHROBUST_H
#include "match/MatchBase.h"

namespace ov_core {

    class MatchRobust : public MatchBase{

    public:

        MatchRobust(MatchBaseOptions &options): MatchBase(options)
        {}

         void loadVocabulary(string path) override
        {

        }

        //  void loadPriorMap(string map_path) override
         void loadPriorMap(string map_file, std::vector<double> PriorMap_intrinsics_cam0, std::vector<double>PriorMap_intrinsics_cam1, std::vector<double> PriorMap_intrinsics_cam2, std::vector<double> PriorMap_intrinsics_cam3, double keyframe_cov_ori, double keyframe_cov_pos) override
        {}

         void ExtractFeatureAndDescriptor(Keyframe& kf) override{};

         bool DetectLoop(Keyframe& kf) override{};

         void MatchingWithLoop(Keyframe& kf) override{};

    private:

       
    

    };
}

#endif //SRC_MATCHROBUST_H
