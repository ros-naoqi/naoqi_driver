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

#include "from_any_value.hpp"

namespace naoqi {

namespace tools {

NaoqiImage fromAnyValueToNaoqiImage(qi::AnyValue& value){
  qi::AnyReferenceVector anyref;
  NaoqiImage result;
  std::ostringstream ss;
  try{
    anyref = value.asListValuePtr();
  }
  catch(std::runtime_error& e)
  {
    ss << "Could not transform AnyValue into list: " << e.what();
    throw std::runtime_error(ss.str());
  }
  qi::AnyReference ref;

  /** Width **/
  ref = anyref[0].content();
  if(ref.kind() == qi::TypeKind_Int)
  {
    result.width = ref.asInt32();
  }
  else
  {
    ss << "Could not retrieve width";
    throw std::runtime_error(ss.str());
  }

  /** Heigth **/
  ref = anyref[1].content();
  if(ref.kind() == qi::TypeKind_Int)
  {
    result.height = ref.asInt32();
  }
  else
  {
    ss << "Could not retrieve height";
    throw std::runtime_error(ss.str());
  }

  /** Layers **/
  ref = anyref[2].content();
  if(ref.kind() == qi::TypeKind_Int)
  {
    result.number_of_layers = ref.asInt32();
  }
  else
  {
    ss << "Could not retrieve number of layers";
    throw std::runtime_error(ss.str());
  }

  /** colorspace **/
  ref = anyref[3].content();
  if(ref.kind() == qi::TypeKind_Int)
  {
    result.colorspace = ref.asInt32();
  }
  else
  {
    ss << "Could not retrieve colorspace";
    throw std::runtime_error(ss.str());
  }

  /** timestamp_s **/
  ref = anyref[4].content();
  if(ref.kind() == qi::TypeKind_Int)
  {
    result.timestamp_s = ref.asInt32();
  }
  else
  {
    ss << "Could not retrieve timestamp_s";
    throw std::runtime_error(ss.str());
  }

  /** timestamp_us **/
  ref = anyref[5].content();
  if(ref.kind() == qi::TypeKind_Int)
  {
    result.timestamp_us = ref.asInt32();
  }
  else
  {
    ss << "Could not retrieve timestamp_us";
    throw std::runtime_error(ss.str());
  }

  /** buffer **/
  ref = anyref[6].content();
  if(ref.kind() == qi::TypeKind_Raw)
  {
    result.buffer = (void*)ref.asRaw().first;
  }
  else
  {
    ss << "Could not retrieve buffer";
    throw std::runtime_error(ss.str());
  }

  /** cam_id **/
  ref = anyref[7].content();
  if(ref.kind() == qi::TypeKind_Int)
  {
    result.cam_id = ref.asInt32();
  }
  else
  {
    ss << "Could not retrieve cam_id";
    throw std::runtime_error(ss.str());
  }

  /** fov_left **/
  ref = anyref[8].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.fov_left = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve fov_left";
    throw std::runtime_error(ss.str());
  }

  /** fov_top **/
  ref = anyref[9].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.fov_top = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve fov_top";
    throw std::runtime_error(ss.str());
  }

  /** fov_right **/
  ref = anyref[10].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.fov_right = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve fov_right";
    throw std::runtime_error(ss.str());
  }

  /** fov_bottom **/
  ref = anyref[11].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.fov_bottom = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve fov_bottom";
    throw std::runtime_error(ss.str());
  }
  return result;
}

NaoqiTimeStamp fromAnyValueToTimeStamp(qi::AnyReference& anyrefs, NaoqiTimeStamp& result){
  qi::AnyReference ref;
  std::ostringstream ss;

  /** timestamp_s **/
  ref = anyrefs[0].content();
  if(ref.kind() == qi::TypeKind_Int)
  {
    result.timestamp_s = ref.asInt32();
  }
  else
  {
    ss << "Could not retrieve timestamp_s";
    throw std::runtime_error(ss.str());
  }

  /** timestamp_us **/
  ref = anyrefs[1].content();
  if(ref.kind() == qi::TypeKind_Int)
  {
    result.timestamp_us = ref.asInt32();
  }
  else
  {
    ss << "Could not retrieve timestamp_us";
    throw std::runtime_error(ss.str());
  }

  return result;
}

NaoqiEyePoints fromAnyValueToEyePoints(qi::AnyReference& anyrefs, NaoqiEyePoints& result){
  qi::AnyReference ref;
  std::ostringstream ss;

  // eye_center
  ref = anyrefs[0].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.eye_center_x = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve eye_center_x";
    throw std::runtime_error(ss.str());
  }

  ref = anyrefs[1].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.eye_center_y = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve eye_center_y";
    throw std::runtime_error(ss.str());
  }

  // nose_side_limit
  ref = anyrefs[2].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.nose_side_limit_x = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve nose_side_limit_x";
    throw std::runtime_error(ss.str());
  }

  ref = anyrefs[3].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.nose_side_limit_y = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve nose_side_limit_y";
    throw std::runtime_error(ss.str());
  }

  // ear_side_limit
  ref = anyrefs[4].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.ear_side_limit_x = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve ear_side_limit_x";
    throw std::runtime_error(ss.str());
  }

  ref = anyrefs[5].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.ear_side_limit_y = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve ear_side_limit_y";
    throw std::runtime_error(ss.str());
  }

  return result;
}

NaoqiNosePoints fromAnyValueToNosePoints(qi::AnyReference& anyrefs, NaoqiNosePoints& result){
  qi::AnyReference ref;
  std::ostringstream ss;

  // bottom_center_limit
  ref = anyrefs[0].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.bottom_center_limit_x = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve bottom_center_limit_x";
    throw std::runtime_error(ss.str());
  }

  ref = anyrefs[1].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.bottom_center_limit_y = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve bottom_center_limit_y";
    throw std::runtime_error(ss.str());
  }

  // bottom_left_limit
  ref = anyrefs[2].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.bottom_left_limit_x = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve bottom_left_limit_x";
    throw std::runtime_error(ss.str());
  }

  ref = anyrefs[3].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.bottom_left_limit_y = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve bottom_left_limit_y";
    throw std::runtime_error(ss.str());
  }

  // bottom_right_limit
  ref = anyrefs[4].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.bottom_right_limit_x = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve bottom_right_limit_x";
    throw std::runtime_error(ss.str());
  }

  ref = anyrefs[5].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.bottom_right_limit_y = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve bottom_right_limit_y";
    throw std::runtime_error(ss.str());
  }

  return result;
}

NaoqiMouthPoints fromAnyValueToMouthPoints(qi::AnyReference& anyrefs, NaoqiMouthPoints& result){
  qi::AnyReference ref;
  std::ostringstream ss;

  // left_limit
  ref = anyrefs[0].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.left_limit_x = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve left_limit_x";
    throw std::runtime_error(ss.str());
  }

  ref = anyrefs[1].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.left_limit_y = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve left_limit_y";
    throw std::runtime_error(ss.str());
  }

  // right_limit
  ref = anyrefs[0].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.right_limit_x = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve right_limit_x";
    throw std::runtime_error(ss.str());
  }

  ref = anyrefs[1].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.right_limit_y = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve right_limit_y";
    throw std::runtime_error(ss.str());
  }

  // top_limit
  ref = anyrefs[0].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.top_limit_x = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve top_limit_x";
    throw std::runtime_error(ss.str());
  }

  ref = anyrefs[1].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.top_limit_y = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve top_limit_y";
    throw std::runtime_error(ss.str());
  }

  return result;
}

NaoqiShapeInfo fromAnyValueToShapeInfo(qi::AnyReference& anyrefs, NaoqiShapeInfo& result){
  qi::AnyReference ref;
  std::ostringstream ss;
  //0
  //alpha
  ref = anyrefs[1].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.alpha = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve alpha";
    throw std::runtime_error(ss.str());
  }

  //beta
  ref = anyrefs[2].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.beta = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve  beta";
    throw std::runtime_error(ss.str());
  }

  //sizeX
  ref = anyrefs[3].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.sizeX = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve sizeX";
    throw std::runtime_error(ss.str());
  }

  //sizeY
  ref = anyrefs[4].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.sizeY = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve sizeY";
    throw std::runtime_error(ss.str());
  }
  return result;
}

NaoqiExtraInfo fromAnyValueToExtraInfo(qi::AnyReference& anyrefs, NaoqiExtraInfo& result){
  qi::AnyReference ref;
  std::ostringstream ss;

  //faceID
  ref = anyrefs[0].content();
  if(ref.kind() == qi::TypeKind_Int)
  {
    result.face_id = ref.asInt32();
  }
  else
  {
    ss << "Could not retrieve faceID";
    throw std::runtime_error(ss.str());
  }

  //scoreReco
  ref = anyrefs[1].content();
  if(ref.kind() == qi::TypeKind_Float)
  {
    result.score_reco = ref.asFloat();
  }
  else
  {
    ss << "Could not retrieve  beta";
    throw std::runtime_error(ss.str());
  }

  //faceLabel
  ref = anyrefs[2].content();
  if(ref.kind() == qi::TypeKind_String)
  {
    result.face_label = ref.asString();
  }
  else
  {
    ss << "Could not retrieve faceLabel";
    throw std::runtime_error(ss.str());
  }

  /* leftEyePoints */
  ref = anyrefs[3].content();
  result.left_eye_points = fromAnyValueToEyePoints(ref, result.left_eye_points);

  /* rightEyePoints */
  ref = anyrefs[4].content();
  result.right_eye_points = fromAnyValueToEyePoints(ref, result.right_eye_points);

  /* nosePoints */
  ref = anyrefs[7].content();
  result.nose_points = fromAnyValueToNosePoints(ref, result.nose_points);

  /* mouthPoints */
  ref = anyrefs[8].content();
  result.mouth_points = fromAnyValueToMouthPoints(ref, result.mouth_points);

  return result;
}

NaoqiFaceInfo fromAnyValueToFaceInfo(qi::AnyReference& anyrefs, NaoqiFaceInfo& result){
  qi::AnyReference ref;
  std::ostringstream ss;

  /* ShapeInfo */
  ref = anyrefs[0].content();
  result.shape_info = fromAnyValueToShapeInfo(ref, result.shape_info);

  /* ExtraInfo */
  ref = anyrefs[1].content();
  NaoqiExtraInfo extra_info;
  result.extra_info.push_back(fromAnyValueToExtraInfo(ref, extra_info));

  return result;
}

NaoqiFaceDetected fromAnyValueToNaoqiFaceDetected(qi::AnyValue& value){
  qi::AnyReferenceVector anyref;
  NaoqiFaceDetected result;
  std::ostringstream ss;
  try{
    anyref = value.asListValuePtr();
  }
  catch(std::runtime_error& e)
  {
    ss << "Could not transform AnyValue into list: " << e.what();
    throw std::runtime_error(ss.str());
  }
  qi::AnyReference ref;

  if ( anyref.size() != 5 ) {
    return result;
  }
  /** TimeStamp_ **/
  ref = anyref[0].content();
  if(ref.kind() == qi::TypeKind_List)
  {
    result.timestamp = fromAnyValueToTimeStamp(ref, result.timestamp);
  }
  else
  {
    ss << "Could not retrieve timestamp";
    throw std::runtime_error(ss.str());
  }

  qi::AnyReferenceVector vec;
  vec = anyref[1].asListValuePtr(); // FaceInfo[N]
  for(int i = 0; i < vec.size()-1; i++) // N
  {
    qi::AnyReference ref2 = vec[i].content();
    struct NaoqiFaceInfo face_info;
    face_info = fromAnyValueToFaceInfo(ref2, face_info);
    result.face_info.push_back(face_info);
  }

  //CameraPose_InTorsoFrame,
  ref = anyref[2].content();
  for (int i = 0; i < 6; i++ ){
    if(ref[i].content().kind() == qi::TypeKind_Float)
    {
      result.camera_pose_in_torso_frame[i] = ref[i].content().asFloat();
    }
    else
    {
      ss << "Could not retrieve Position6D";
      throw std::runtime_error(ss.str());
    }
  }
  //CameraPose_InRobotFrame,
  ref = anyref[3].content();
  for (int i = 0; i < 6; i++ ){
    if(ref[i].content().kind() == qi::TypeKind_Float)
    {
      result.camera_pose_in_robot_frame[i] = ref[i].content().asFloat();
    }
    else
    {
      ss << "Could not retrieve Position6D";
      throw std::runtime_error(ss.str());
    }
  }
  //Camera_Id
  ref = anyref[4].content();
  if(ref.kind() == qi::TypeKind_Int)
  {
    result.camera_id = ref.asInt32();
  }
  else
  {
    ss << "Could not retrieve timestamp_us";
    throw std::runtime_error(ss.str());
  }

  return result;
}

std::vector<float> fromAnyValueToFloatVector(qi::AnyValue& value, std::vector<float>& result){
  qi::AnyReferenceVector anyrefs = value.asListValuePtr();

  for(int i=0; i<anyrefs.size();i++)
  {
    try
    {
      result.push_back(anyrefs[i].content().toFloat());
    }
    catch(std::runtime_error& e)
    {
      result.push_back(-1.0);
      std::cout << e.what() << "=> set to -1" << std::endl;
    }
  }
  return result;
}

std::vector<std::string> fromAnyValueToStringVector(qi::AnyValue& value, std::vector<std::string>& result){
  qi::AnyReferenceVector anyrefs = value.asListValuePtr();

  for(int i=0; i<anyrefs.size();i++)
  {
    try
    {
      result.push_back(anyrefs[i].content().toString());
    }
    catch(std::runtime_error& e)
    {
      result.push_back("Not available");
      std::cout << e.what() << " => set to 'Not available'" << std::endl;
    }
  }
  return result;
}

}
}
