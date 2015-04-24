/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "from_any_value.hpp"

namespace alros {

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
