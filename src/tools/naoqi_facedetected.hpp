/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef NAOQI_FACEDETECTED_HPP
#define NAOQI_FACEDETECTED_HPP

#include <string>
#include <vector>

namespace naoqi{

namespace tools {

/**
 * @brief The struct describing an facedetected retrieved by ALFaceDetection
 *        This specification can be found here:
 *        http://doc.aldebaran.com/2-1/naoqi/peopleperception/alfacedetection.html#alfacedetection
 */
struct NaoqiEyePoints{
  float eye_center_x;
  float eye_center_y;
  float nose_side_limit_x;
  float nose_side_limit_y;
  float ear_side_limit_x;
  float ear_side_limit_y;
};
struct NaoqiNosePoints{
  float bottom_center_limit_x;
  float bottom_center_limit_y;
  float bottom_left_limit_x;
  float bottom_left_limit_y;
  float bottom_right_limit_x;
  float bottom_right_limit_y;
};
struct NaoqiMouthPoints{
  float left_limit_x;
  float left_limit_y;
  float right_limit_x;
  float right_limit_y;
  float top_limit_x;
  float top_limit_y;
};
struct NaoqiExtraInfo{
  int face_id;
  float score_reco;
  std::string face_label;
  struct NaoqiEyePoints left_eye_points;
  struct NaoqiEyePoints right_eye_points;
  struct NaoqiNosePoints nose_points;
  struct NaoqiMouthPoints mouth_points;
};
struct NaoqiShapeInfo{
  float alpha;
  float beta;
  float sizeX;
  float sizeY;
};
struct NaoqiFaceInfo{
  struct NaoqiShapeInfo shape_info;
  std::vector<struct NaoqiExtraInfo> extra_info;
};
struct NaoqiTimeStamp{
  int timestamp_s;
  int timestamp_us;
};


struct NaoqiFaceDetected{
  //TimeStamp,
  struct NaoqiTimeStamp timestamp;
  //[ FaceInfo[N], Time_Filtered_Reco_Info ]
  std::vector<struct NaoqiFaceInfo> face_info;
  //CameraPose_InTorsoFrame,
  float camera_pose_in_torso_frame[6];
  //CameraPose_InRobotFrame,
  float camera_pose_in_robot_frame[6];
  //Camera_Id
  int camera_id;
};

}

}

#endif // NAOQI_FACEDETECTED_HPP
