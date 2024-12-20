//
// Created by zzq on 2020/10/11.
//

#include "MatchBase.h"


static size_t WriteCallback(char* ptr, size_t size, size_t nmemb, std::string* data) {
    size_t totalBytes = size * nmemb;
    data->append(ptr, totalBytes);
    return totalBytes;
}



void MatchBase::DetectAndMatch(const std::string& DetectAndMatch_URL, const std::string& DetectAndMatch_img_save_path) 
{
  // 使用传递的参数
  std::cout << "DetectAndMatch_URL: " << DetectAndMatch_URL << std::endl;
  std::cout << "DetectAndMatch_img_save_path: " << DetectAndMatch_img_save_path << std::endl;
  
  Keyframe* kf1 = new Keyframe();
  Keyframe* kf2= new Keyframe();
  Keyframe* kf3 = new Keyframe();

  while(1)
  {

    {
      std::unique_lock<std::mutex> lck(img_buffer_mutex);
    if(!img_buffer.empty())
    {
      kf1 = img_buffer.front();
      img_buffer.pop();
    }
    if(!img_buffer.empty())
    {
      kf2 = img_buffer.front();
      img_buffer.pop();
    }
    if(!img_buffer.empty())
    {
      kf3 = img_buffer.front();
      img_buffer.pop();
    }
    }
    if(kf1->index!=-1)
    // if(false)
    {
      Mat img1 = kf1->image;
      Mat img2 = kf2->image;
      Mat img3 = kf3->image;

      Mat img2_new, img3_new;

      string new_name = DetectAndMatch_img_save_path + to_string(kf1->time_stamp) +".jpg";
      if(img1.empty() || img2.empty() || img3.empty()) {
          std::cout << "Could not open or find the images!" << std::endl;
          return;
      }

      // 初始化一个Mat来存储结果
      cv::Mat concat_result;

      if(img1.cols != img2.cols)
      {
        // 加载三张图片
        if(img1.empty() || img2.empty() || img3.empty())
        {
            std::cout << "Could not open or find the images!" << std::endl;
            return;
        }

        // 获取每张图片的宽度和高度
        int height1 = img1.rows;
        int height2 = img2.rows;
        int height3 = img3.rows;

        int width1 = img1.cols;
        int width2 = img2.cols;
        int width3 = img3.cols;

        // 计算结果图片的宽度和最大高度
        int maxWidth = 3 * std::max({width1,width2, width3});
        int maxHeight = std::max({height1, height2, height3});

        // 创建一个新的空白图像用于存放拼接后的图片
        concat_result = cv::Mat::zeros(maxHeight, maxWidth, CV_8UC1);
        img2_new = cv::Mat::zeros(maxHeight, std::max({width1,width2, width3}), CV_8UC1);
        img3_new = cv::Mat::zeros(maxHeight, std::max({width1,width2, width3}), CV_8UC1);

        img2.copyTo(img2_new(cv::Rect(0, 0, width2, height2)));
        img3.copyTo(img3_new(cv::Rect(0, 0, width3, height3)));

        // 直接将图片复制到结果图中，忽略高度差异（即图片顶部对齐）
        img1.copyTo(concat_result(cv::Rect(0, 0, width1, height1)));
        img3.copyTo(concat_result(cv::Rect(width1, 0, width2, height2)));
        img2.copyTo(concat_result(cv::Rect(2 * std::max({width1, width2, width3}), 0, width3, height3)));
        
      }else
      {
        // 创建一个Mat向量来存放要拼接的图像
        std::vector<cv::Mat> images;

        images.push_back(img1);
        images.push_back(img3);
        images.push_back(img2);

        // 使用hconcat函数进行横向拼接
        cv::hconcat(images, concat_result);
      }

      boost::posix_time::ptime relinear_t1=boost::posix_time::microsec_clock::local_time();   

      imwrite(new_name,concat_result);
      // Send request
      CURL *curl;
      CURLcode res;
      json result_json;
      std::string result_string;
      curl_global_init(CURL_GLOBAL_DEFAULT);
      curl = curl_easy_init();
      if (curl) {
          curl_easy_setopt(curl, CURLOPT_URL, DetectAndMatch_URL.c_str());
          curl_easy_setopt(curl, CURLOPT_POST, 1L);
          curl_mime *mime = curl_mime_init(curl);
          curl_mimepart *part = curl_mime_addpart(mime);
          curl_mime_name(part, "image"); // 表单字段名，根据云端API要求设置
          curl_mime_filedata(part, new_name.c_str()); // 图片路径
          curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
          // 执行请求
          curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
          curl_easy_setopt(curl, CURLOPT_WRITEDATA, &result_string); // 传递json对象的引用
          res = curl_easy_perform(curl);
          if(res != CURLE_OK) {
              std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
          }


          result_json = nlohmann::json::parse(result_string);
          Json_parse(*kf1,*kf2,*kf3,result_json);
          // 清理
          curl_mime_free(mime);
          curl_easy_cleanup(curl);
          // Parse response
          // result_json = json::parse(response_data);
      }
      boost::posix_time::ptime relinear_t2=boost::posix_time::microsec_clock::local_time();
      curl_global_cleanup();
      double time_ = (relinear_t2-relinear_t1).total_microseconds() * 1e-6;
      
      kf1->checked = true;
      kf2->checked = true;
      kf3->checked = true;
  }
}
}

void MatchBase::Json_parse(Keyframe& kf1,Keyframe& kf2,Keyframe& kf3, json json_)
{
  std::vector<Keyframe> kfs;
  kfs.push_back(kf1);
  kfs.push_back(kf3);
  kfs.push_back(kf2);

  std::ofstream outputFile;
  outputFile.open("/tmp/json.txt", ios::app);
    if (!outputFile.is_open()) {
        std::cerr << "Error opening output file!" << std::endl;
        return ;
    }

    // 以美化（pretty print）格式写入JSON到TXT文件
  outputFile << std::setw(4) << json_ << std::endl; 

  outputFile.close();

  for(int i = 0; i < json_.size(); i ++)
  // for(int i = 0; i < 1; i ++)
  {
    json match_info = json_[i];
    if (match_info["success"] < 0.5)
      continue;
    vector<cv::Point2f> matched_loop_kf_uv, matched_loop_kf_uv_norm;
    vector<cv::Point3f> matched_loop_kf_3dpt;
    vector<cv::Point2f> matched_query_kf_uv, matched_query_kf_uv_norm;
    int query_camid = match_info["query_camid"];
    int map_imgid = match_info["map_imgid"];
    std::string map_imgname = match_info["map_imgname"];
    Keyframe* loop_kf = kfdatabase->get_keyframe(map_imgid); 

    for (const auto& pointArray : match_info.at("query_p2d")) {
        if (pointArray.size() == 2) { // 确保数组包含两个元素
            double x = pointArray[0].get<double>();
            double y = pointArray[1].get<double>();
            matched_query_kf_uv.emplace_back(x, y);
        } else {
            std::cerr << "Wrong point size " << std::endl;
        }
    }
    for (const auto& pointArray : match_info.at("map_p2d")) {
        if (pointArray.size() == 2) { // 确保数组包含两个元素
            double x = pointArray[0].get<double>();
            double y = pointArray[1].get<double>();
            cv::Point2f tmp_point(x,y);
            cv::Point2f norm_point = undistort_point_new(tmp_point,loop_kf->_intrinsics);
            matched_loop_kf_uv_norm.emplace_back(norm_point);
            matched_loop_kf_uv.emplace_back(x, y);
        } else {
            std::cerr << "Wrong point size " << std::endl;
        }
    }
    for (const auto& pointArray : match_info.at("map_p3d")) {
        if (pointArray.size() == 3) { // 确保数组包含三个元素
            double x = pointArray[0].get<double>();
            double y = pointArray[1].get<double>();
            double z = pointArray[2].get<double>();
            matched_loop_kf_3dpt.emplace_back(x, y, z);
        } else {
            std::cerr << "Wrong point size " << std::endl;
        }
    }


    for (int i =0; i< matched_query_kf_uv.size();i++)
    {
      cv::Point2f query_kf_uv_norm = undistort_point_new(matched_query_kf_uv.at(i),query_camid);
      matched_query_kf_uv_norm.push_back(query_kf_uv_norm);
    }
    assert(matched_loop_kf_3dpt.size()==matched_loop_kf_uv.size());
    assert(matched_loop_kf_3dpt.size()==matched_loop_kf_uv_norm.size());
    assert(matched_query_kf_uv_norm.size()==matched_query_kf_uv.size());
    assert(matched_query_kf_uv_norm.size()==matched_loop_kf_3dpt.size());
    std::stringstream ss;
    ss << std::setprecision(19) << kfs[i].time_stamp;
    std::string new_string_time = ss.str();
    new_string_time.at(0) = query_camid+1+'0';
    double new_time = stod(new_string_time);
    loop_kf->matched_id_map.insert({new_time,query_camid});
    loop_kf->matched_point_2d_uv_map.insert({new_time,matched_query_kf_uv});
    loop_kf->matched_point_2d_norm_map.insert({new_time,matched_query_kf_uv_norm});
    loop_kf->point_2d_uv_map.insert({new_time,matched_loop_kf_uv});
    loop_kf->point_2d_uv_norm_map.insert({new_time,matched_loop_kf_uv_norm});
    loop_kf->point_3d_map.insert({new_time,matched_loop_kf_3dpt});
    loop_kf->point_3d_linp_map.insert({new_time,matched_loop_kf_3dpt});
    loop_kf->loop_img_timestamp_vec.push_back(new_time);
    loop_kf->get_loopkfs_time = new_time;

    //TODO
    loop_kf->matched_cam_id=query_camid; //which camera kf matched querry cam id 

    //we finish all the matching works, and the match information into match_map.
    kfs[i].loop_image_id_vec.clear();
    kfs[i].loop_image_id_vec.push_back(map_imgid);
    match_map.insert({new_time, kfs[i].loop_image_id_vec});  
  }
}

void MatchBase::keyframesCheck(ov_core::Keyframe* kf0,ov_core::Keyframe* kf1,ov_core::Keyframe* kf2,ov_core::Keyframe* kf3)
{
  
  img_buffer_mutex.lock();
  if(_last_keyframe->index==-1)
  {
    Keyframe* clone_kf1 = new Keyframe();
    Keyframe* clone_kf2 = new Keyframe();
    Keyframe* clone_kf3 = new Keyframe();

    clone_kf1->index = kfdatabase->get_kfdatabase_idcount();
    clone_kf1->time_stamp = kf0->time_stamp;
    clone_kf1->cam_id = kf0->cam_id;
    clone_kf1->image = kf0->image;
    clone_kf1->_intrinsics = kf0->_intrinsics;
    clone_kf1->_Pose_KFinVIO = kf0->_Pose_KFinVIO;
    clone_kf1->selected = true;
    kf0->selected = true;
    img_buffer.push(clone_kf1);
    kfdatabase->update_kf_new(clone_kf1);

    clone_kf2->index = kfdatabase->get_kfdatabase_idcount();
    clone_kf2->time_stamp = kf2->time_stamp;
    clone_kf2->cam_id = kf2->cam_id;
    clone_kf2->image = kf2->image;
    clone_kf2->_intrinsics = kf2->_intrinsics;
    clone_kf2->_Pose_KFinVIO = kf2->_Pose_KFinVIO;
    clone_kf2->selected = true;
    kf2->selected = true;
    img_buffer.push(clone_kf2);
    kfdatabase->update_kf_new(clone_kf2);

    clone_kf3->index = kfdatabase->get_kfdatabase_idcount();
    clone_kf3->time_stamp = kf3->time_stamp;
    clone_kf3->cam_id = kf3->cam_id;
    clone_kf3->image = kf3->image;
    clone_kf3->_intrinsics = kf3->_intrinsics;
    clone_kf3->_Pose_KFinVIO = kf3->_Pose_KFinVIO;
    clone_kf3->selected = true;
    kf3->selected = true;
    img_buffer.push(clone_kf3);
    kfdatabase->update_kf_new(clone_kf3);

    _last_keyframe = clone_kf1;
    cout<<"first keyframe"<<endl;
  }
  else
  {
    //TODO Temp modify
    bool cond1 = (kf0->time_stamp-_last_keyframe->time_stamp)>0.5;
    if(cond1)
    {
      Keyframe* clone_kf0 = new Keyframe();
      Keyframe* clone_kf2 = new Keyframe();
      Keyframe* clone_kf3 = new Keyframe();
      
      clone_kf0->index = kfdatabase->get_kfdatabase_idcount();
      clone_kf0->time_stamp = kf0->time_stamp;
      clone_kf0->cam_id = kf0->cam_id;
      clone_kf0->image = kf0->image;
      clone_kf0->_intrinsics = kf0->_intrinsics;
      clone_kf0->_Pose_KFinVIO = kf0->_Pose_KFinVIO;
      clone_kf0->selected = true;
      kf0->selected = true;
      img_buffer.push(clone_kf0);
      kfdatabase->update_kf_new(clone_kf0);

      clone_kf2->index = kfdatabase->get_kfdatabase_idcount();
      clone_kf2->time_stamp = kf2->time_stamp;
      clone_kf2->cam_id = kf2->cam_id;
      clone_kf2->image = kf2->image;
      clone_kf2->_intrinsics = kf2->_intrinsics;
      clone_kf2->_Pose_KFinVIO = kf2->_Pose_KFinVIO;
      clone_kf2->selected = true;
      kf2->selected = true;
      img_buffer.push(clone_kf2);
      kfdatabase->update_kf_new(clone_kf2);

      clone_kf3->index = kfdatabase->get_kfdatabase_idcount();
      clone_kf3->time_stamp = kf3->time_stamp;
      clone_kf3->cam_id = kf3->cam_id;
      clone_kf3->image = kf3->image;
      clone_kf3->_intrinsics = kf3->_intrinsics;
      clone_kf3->_Pose_KFinVIO = kf3->_Pose_KFinVIO;
      clone_kf3->selected = true;
      kf3->selected = true;
      img_buffer.push(clone_kf3);
      kfdatabase->update_kf_new(clone_kf3);
      
      _last_keyframe = clone_kf0;
      // clone_kf->checked = true;
    }
  }
  img_buffer_mutex.unlock();
  

  
}