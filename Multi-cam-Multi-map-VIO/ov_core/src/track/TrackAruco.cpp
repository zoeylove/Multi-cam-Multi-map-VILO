/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "TrackAruco.h"


using namespace ov_core;


void TrackAruco::feed_monocular(double timestamp, cv::Mat &imgin, size_t cam_id) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Lock this data feed for this camera
    std::unique_lock<std::mutex> lck(mtx_feeds.at(cam_id));

    // Histogram equalize
    cv::Mat img;
    cv::equalizeHist(imgin, img);

    // Clear the old data from the last timestep
    ids_aruco[cam_id].clear();
    corners[cam_id].clear();
    rejects[cam_id].clear();

    // If we are downsizing, then downsize
    cv::Mat img0;
    if(do_downsizing) {
        cv::pyrDown(img,img0,cv::Size(img.cols/2,img.rows/2));
    } else {
        img0 = img;
    }


    //===================================================================================
    //===================================================================================

    // Perform extraction
    // cv::aruco::detectMarkers(img0,aruco_dict,corners[cam_id],ids_aruco[cam_id],aruco_params,rejects[cam_id]);
    rT2 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    //===================================================================================

    // If we downsized, scale all our u,v measurements by a factor of two
    // Note: we do this so we can use these results for visulization later
    // Note: and so that the uv added is in the same image size
    if(do_downsizing) {
        for(size_t i=0; i<corners[cam_id].size(); i++) {
            for(size_t j=0; j<corners[cam_id].at(i).size(); j++) {
                corners[cam_id].at(i).at(j).x *= 2;
                corners[cam_id].at(i).at(j).y *= 2;
            }
        }
        for(size_t i=0; i<rejects[cam_id].size(); i++) {
            for(size_t j=0; j<rejects[cam_id].at(i).size(); j++) {
                rejects[cam_id].at(i).at(j).x *= 2;
                rejects[cam_id].at(i).at(j).y *= 2;
            }
        }
    }


    //===================================================================================
    //===================================================================================


    // ID vectors, of all currently tracked IDs
    std::vector<size_t> ids_new;

    // Append to our feature database this new information
    for(size_t i=0; i<ids_aruco[cam_id].size(); i++) {
        // Skip if ID is greater then our max
        if(ids_aruco[cam_id].at(i) > max_tag_id)
            continue;
        // Assert we have 4 points (we will only use one of them)
        assert(corners[cam_id].at(i).size()==4);
        // Try to undistort the point
        cv::Point2f npt_l = undistort_point(corners[cam_id].at(i).at(0), cam_id);
        // Append to the ids vector and database
        ids_new.push_back((size_t)ids_aruco[cam_id].at(i));
        database->update_feature((size_t)ids_aruco[cam_id].at(i), timestamp, cam_id,
                                 corners[cam_id].at(i).at(0).x, corners[cam_id].at(i).at(0).y,
                                 npt_l.x, npt_l.y);
    }


    // Move forward in time
    img_last[cam_id] = img.clone();
    ids_last[cam_id] = ids_new;
    rT3 =  boost::posix_time::microsec_clock::local_time();

}


void TrackAruco::feed_stereo(double timestamp, cv::Mat &img_leftin, cv::Mat &img_rightin, size_t cam_id_left, size_t cam_id_right) {

    // Start timing
    rT1 =  boost::posix_time::microsec_clock::local_time();

    // Lock this data feed for this camera
    std::unique_lock<std::mutex> lck1(mtx_feeds.at(cam_id_left));
    std::unique_lock<std::mutex> lck2(mtx_feeds.at(cam_id_right));

    // Histogram equalize
    cv::Mat img_left, img_right;
    cv::equalizeHist(img_leftin, img_left);
    cv::equalizeHist(img_rightin, img_right);

    // Clear the old data from the last timestep
    ids_aruco[cam_id_left].clear();
    ids_aruco[cam_id_right].clear();
    corners[cam_id_left].clear();
    corners[cam_id_right].clear();
    rejects[cam_id_left].clear();
    rejects[cam_id_right].clear();

    // If we are downsizing, then downsize
    cv::Mat img0, img1;
    if(do_downsizing) {
        cv::pyrDown(img_left,img0,cv::Size(img_left.cols/2,img_left.rows/2));
        cv::pyrDown(img_right,img1,cv::Size(img_right.cols/2,img_right.rows/2));
    } else {
        img0 = img_left;
        img1 = img_right;
    }


    //===================================================================================
    //===================================================================================

    // Perform extraction (doing this in parallel is actually slower on my machine -pgeneva)
    // cv::aruco::detectMarkers(img0,aruco_dict,corners[cam_id_left],ids_aruco[cam_id_left],aruco_params,rejects[cam_id_left]);
    // cv::aruco::detectMarkers(img1,aruco_dict,corners[cam_id_right],ids_aruco[cam_id_right],aruco_params,rejects[cam_id_right]);
    rT2 =  boost::posix_time::microsec_clock::local_time();


    //===================================================================================
    //===================================================================================

    // If we downsized, scale all our u,v measurements by a factor of two
    // Note: we do this so we can use these results for visulization later
    // Note: and so that the uv added is in the same image size
    if(do_downsizing) {
        for(size_t i=0; i<corners[cam_id_left].size(); i++) {
            for(size_t j=0; j<corners[cam_id_left].at(i).size(); j++) {
                corners[cam_id_left].at(i).at(j).x *= 2;
                corners[cam_id_left].at(i).at(j).y *= 2;
            }
        }
        for(size_t i=0; i<corners[cam_id_right].size(); i++) {
            for(size_t j=0; j<corners[cam_id_right].at(i).size(); j++) {
                corners[cam_id_right].at(i).at(j).x *= 2;
                corners[cam_id_right].at(i).at(j).y *= 2;
            }
        }
        for(size_t i=0; i<rejects[cam_id_left].size(); i++) {
            for(size_t j=0; j<rejects[cam_id_left].at(i).size(); j++) {
                rejects[cam_id_left].at(i).at(j).x *= 2;
                rejects[cam_id_left].at(i).at(j).y *= 2;
            }
        }
        for(size_t i=0; i<rejects[cam_id_right].size(); i++) {
            for(size_t j=0; j<rejects[cam_id_right].at(i).size(); j++) {
                rejects[cam_id_right].at(i).at(j).x *= 2;
                rejects[cam_id_right].at(i).at(j).y *= 2;
            }
        }
    }


    //===================================================================================
    //===================================================================================


    // ID vectors, of all currently tracked IDs
    std::vector<size_t> ids_left_new, ids_right_new;

    // Append to our feature database this new information
    for(size_t i=0; i<ids_aruco[cam_id_left].size(); i++) {
        // Skip if ID is greater then our max
        if(ids_aruco[cam_id_left].at(i) > max_tag_id)
            continue;
        // Assert we have 4 points (we will only use one of them)
        assert(corners[cam_id_left].at(i).size()==4);
        // Try to undistort the point (could fail undistortion!)
        cv::Point2f npt_l = undistort_point(corners[cam_id_left].at(i).at(0), cam_id_left);
        // Append to the ids vector and database
        ids_left_new.push_back((size_t)ids_aruco[cam_id_left].at(i));
        database->update_feature((size_t)ids_aruco[cam_id_left].at(i), timestamp, cam_id_left,
                                 corners[cam_id_left].at(i).at(0).x, corners[cam_id_left].at(i).at(0).y,
                                 npt_l.x, npt_l.y);
    }
    for(size_t i=0; i<ids_aruco[cam_id_right].size(); i++) {
        // Skip if ID is greater then our max
        if(ids_aruco[cam_id_right].at(i) > max_tag_id)
            continue;
        // Assert we have 4 points (we will only use one of them)
        assert(corners[cam_id_right].at(i).size()==4);
        // Try to undistort the point (could fail undistortion!)
        cv::Point2f npt_l = undistort_point(corners[cam_id_right].at(i).at(0), cam_id_right);
        // Append to the ids vector and database
        ids_right_new.push_back((size_t)ids_aruco[cam_id_right].at(i));
        database->update_feature((size_t)ids_aruco[cam_id_right].at(i), timestamp, cam_id_right,
                                 corners[cam_id_right].at(i).at(0).x, corners[cam_id_right].at(i).at(0).y,
                                 npt_l.x, npt_l.y);
    }


    // Move forward in time
    img_last[cam_id_left] = img_left.clone();
    img_last[cam_id_right] = img_right.clone();
    ids_last[cam_id_left] = ids_left_new;
    ids_last[cam_id_right] = ids_right_new;
    rT3 =  boost::posix_time::microsec_clock::local_time();

}


void TrackAruco::display_active(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2) {

    // Cache the images to prevent other threads from editing while we viz (which can be slow)
    std::map<size_t, cv::Mat> img_last_cache;
    for(auto const& pair : img_last) {
        img_last_cache.insert({pair.first,pair.second.clone()});
    }

    // Get the largest width and height
    int max_width = -1;
    int max_height = -1;
    for(auto const& pair : img_last_cache) {
        if(max_width < pair.second.cols) max_width = pair.second.cols;
        if(max_height < pair.second.rows) max_height = pair.second.rows;
    }

    // If the image is "small" thus we shoudl use smaller display codes
    bool is_small = (std::min(max_width,max_height) < 400);

    // If the image is "new" then draw the images from scratch
    // Otherwise, we grab the subset of the main image and draw on top of it
    bool image_new = ((int)img_last_cache.size()*max_width != img_out.cols || max_height != img_out.rows);

    // If new, then resize the current image
    if(image_new) img_out = cv::Mat(max_height,(int)img_last_cache.size()*max_width,CV_8UC3,cv::Scalar(0,0,0));

    // Loop through each image, and draw
    int index_cam = 0;
    for(auto const& pair : img_last_cache) {
        // Lock this image
        std::unique_lock<std::mutex> lck(mtx_feeds.at(pair.first));
        // select the subset of the image
        cv::Mat img_temp;
        if(image_new) cv::cvtColor(img_last_cache[pair.first], img_temp, CV_GRAY2RGB);
        else img_temp = img_out(cv::Rect(max_width*index_cam,0,max_width,max_height));
        // draw...
        // if(!ids_aruco[pair.first].empty()) cv::aruco::drawDetectedMarkers(img_temp, corners[pair.first], ids_aruco[pair.first]);
        // if(!rejects[pair.first].empty()) cv::aruco::drawDetectedMarkers(img_temp, rejects[pair.first], cv::noArray(), cv::Scalar(100,0,255));
        // Draw what camera this is
        auto txtpt = (is_small)? cv::Point(10,30) : cv::Point(30,60);
        cv::putText(img_temp, "CAM:"+std::to_string((int)pair.first), txtpt, cv::FONT_HERSHEY_COMPLEX_SMALL, (is_small) ? 1.5 : 3.0, cv::Scalar(0,255,0), 3);
        // Replace the output image
        img_temp.copyTo(img_out(cv::Rect(max_width*index_cam,0,img_last_cache[pair.first].cols,img_last_cache[pair.first].rows)));
        index_cam++;
    }

}
