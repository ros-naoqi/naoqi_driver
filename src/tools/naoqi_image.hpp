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

#ifndef NAOQI_IMAGE_HPP
#define NAOQI_IMAGE_HPP

namespace naoqi{

namespace tools {

/**
 * @brief The struct describing an image retrieved by ALVideoDevice.
 *        This specification can be found here:
 *        http://doc.aldebaran.com/2-1/naoqi/vision/alvideodevice-tuto.html
 */
struct NaoqiImage{
  int width;
  int height;
  int number_of_layers;
  int colorspace;
  int timestamp_s;
  int timestamp_us;
  void* buffer;
  int cam_id;
  float fov_left;
  float fov_top;
  float fov_right;
  float fov_bottom;
};

}

}

#endif // NAOQI_IMAGE_HPP
