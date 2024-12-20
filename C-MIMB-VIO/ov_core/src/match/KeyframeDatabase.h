//
// Created by zzq on 2020/10/10.
//

#ifndef SRC_KEYFRAMEDATABASE_H
#define SRC_KEYFRAMEDATABASE_H

#include <vector>
#include <mutex>
#include <Eigen/Eigen>

#include "Keyframe.h"



namespace ov_core{



    class KeyframeDatabase{

    public:
        KeyframeDatabase() {
            keyframes_idlookup = std::unordered_map<double, Keyframe *>();
            keyframes_idts_lookup = std::unordered_map<int, double>();
        }

        /**
         * @brief Get a specified feature
         * @param id What feature we want to get
         * @param remove Set to true if you want to remove the feature from the database (you will need to handle the freeing of memory)
         * @return Either a feature object, or null if it is not in the database.
         */
        Keyframe *get_keyframe(double id, bool remove=false) {
            std::unique_lock<std::mutex> lck(mtx);
            std::cout<<"in get keyframe , time is "<< to_string(id)<<endl;

            if (keyframes_idlookup.find(id) != keyframes_idlookup.end()) {
                Keyframe* temp = keyframes_idlookup[id];
                std::cout<<"get keyframe time is "<<to_string(temp->time_stamp);
                if(remove)
                {
                  keyframes_idlookup.erase(id);
                  auto iter = keyframes_idts_lookup.begin();
                  while(iter!=keyframes_idts_lookup.end())
                  {
                    if(iter->second==id)
                    {
                      keyframes_idts_lookup.erase(iter->first);
                      break;
                    }

                    iter++;
                  }
                } 
                return temp;
            } else {
                return nullptr;
            }
        }

        Keyframe *get_keyframe(int id, bool remove=false)
        {
          std::unique_lock<std::mutex> lck(mtx);
          if(keyframes_idts_lookup.find(id) != keyframes_idts_lookup.end())
          {
            double ts = keyframes_idts_lookup[id];
            Keyframe* temp = keyframes_idlookup[ts];

            if(remove)
            {
              keyframes_idlookup.erase(ts);
              keyframes_idts_lookup.erase(id);
            } 
            return temp;
          }
          else
          {
            return nullptr;
          }

        }

        void cleanup() {
            // Loop through all features
            std::unique_lock<std::mutex> lck(mtx);
            for (auto it = keyframes_idlookup.begin(); it != keyframes_idlookup.end();) {
                // If delete flag is set, then delete it
                if ((*it).second->to_delete) {
                    delete (*it).second;
                    keyframes_idlookup.erase(it++);
                    double ts=(*it).first;
                    auto iter = keyframes_idts_lookup.begin();
                    while(iter!=keyframes_idts_lookup.end())
                    {
                      if(iter->second==ts)
                      {
                        keyframes_idts_lookup.erase(iter->first);
                        break;
                      }
                      iter++;
                    }
                } else {
                    it++;
                }
            }
        }

        size_t size() {
            std::unique_lock<std::mutex> lck(mtx);
            if(!keyframes_idts_lookup.empty())
            {
              assert(keyframes_idts_lookup.size()==keyframes_idlookup.size());
            }
            return keyframes_idlookup.size();
        }

        std::unordered_map<double, Keyframe *> get_internal_data() {
            std::unique_lock<std::mutex> lck(mtx);
              return keyframes_idlookup;
        }

        std::unordered_map<int,double> get_idts_data() 
        {
          std::unique_lock<std::mutex> lck(mtx);
          return keyframes_idts_lookup;
        }

        void update_kf_new(Keyframe* kf) {

            // Find this feature using the ID lookup
            std::unique_lock<std::mutex> lck(mtx);
            if (keyframes_idlookup.find(kf->time_stamp) != keyframes_idlookup.end()) {
                // Get our feature
                return;
            }

            // Debug info
            // Append this new feature into our database
            std::stringstream ss;
            ss << std::setprecision(19) << kf->time_stamp;
            std::string string_time = ss.str();
            string_time.at(0) = kf->cam_id +1 +'0';
            double new_kf_timestamp = stod(string_time);
            keyframes_idlookup.insert({new_kf_timestamp, kf});
            keyframes_idts_lookup.insert({kf->index, kf->time_stamp});
            id_count++;
        }

        void update_kf(Keyframe* kf) {

            // Find this feature using the ID lookup
            std::unique_lock<std::mutex> lck(mtx);
            if (keyframes_idlookup.find(kf->time_stamp) != keyframes_idlookup.end()) {

                return;
            }

            // Append this new feature into our database
            keyframes_idlookup.insert({kf->time_stamp, kf});
            if(kf->index!=-1)
            {
              cout<<"kf->index: "<<kf->index<<" id_count"<<id_count<<endl;
              assert(kf->index==id_count);
            }
            kf->index = id_count;
            keyframes_idts_lookup.insert({id_count,kf->time_stamp});
            id_count++;
            cout<<"keyframe_idlookup size:"<<keyframes_idlookup.size()<<endl;
        }

        double get_match_kf(double timestamp)
        {
            auto it=keyframes_idlookup.begin();
            while(it!=keyframes_idlookup.end())
            {
                if(it->second->loop_img_timestamp == timestamp)
                {
                    return it->first;
                    break;
                }
                it++;
            }
            return -1;
            
        }

        

        std::vector<double> get_match_kfs(double timestamp)
        {
            std::vector<double> res;
            auto it=keyframes_idlookup.begin();
            while(it!=keyframes_idlookup.end())
            {
                Keyframe* kf=it->second;
                for(int i=0;i<kf->loop_img_timestamp_vec.size();i++)
                {
                    if(abs(kf->loop_img_timestamp_vec[i] - timestamp) < 0.001)
                    {
                       res.push_back(it->first);
                       break;
                    }
                }
                it++;
            }
            return res;
        }



        double get_approx_ts(double timestamp)
        {
            double ts1=((timestamp*100)-3)/100.0;  //minus 0.03s
            double ts2=((timestamp*100)+3)/100.0;  //add 0.03s
            vector<double> ts;
            auto it=keyframes_idlookup.begin();
            while(it!=keyframes_idlookup.end())
            {
                Keyframe* kf=it->second;
                for(int i=0;i<kf->loop_img_timestamp_vec.size();i++)
                {
                    if(kf->loop_img_timestamp_vec[i]<ts2&&kf->loop_img_timestamp_vec[i]>ts1)
                    {
                        ts.push_back(kf->loop_img_timestamp_vec[i]);
                    }
                }
                it++;
            }
            if(ts.empty())
            {
               return -1.0;
            }


        }

        void set_map_kf_max_ts(double ts)
        {
          map_kf_max_ts = ts;
        }

        double get_map_kf_max_ts()
        {
          return map_kf_max_ts;
        }

        void set_map_kf_max_id(int id)
        {
          map_kf_max_id = id;
        }

        int get_map_kf_max_id()
        {
          return map_kf_max_id;
        }

        int get_kfdatabase_idcount()
        {
          return id_count;
        }



    protected:
        /// Mutex lock for our keyframes
        std::mutex mtx;
        
        int id_count=0;

        int map_kf_max_id = -1;
        double map_kf_max_ts = -1;
        /// Our lookup array that allow use to query based on ID(timestamp)
        std::unordered_map<double, Keyframe *> keyframes_idlookup;
        
        //* map between global id and keyframe timestamp
        std::unordered_map<int,double> keyframes_idts_lookup;
    };

}


#endif //SRC_KEYFRAMEDATABASE_H
