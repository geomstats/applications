/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: John Hsu */

#ifndef USDF_STATE_H
#define USDF_STATE_H

#include <string>
#include <vector>
#include <map>

#include <urdf/boost_replacement/shared_ptr.h>

#include "urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h"
#include "urdf/urdfdom_headers/urdf_model/include/urdf_model/pose.h"

namespace urdf{
// light class
class Light
{
public:
  Light() {this->clear();}
  // name
  std::string name;
  // type of light
  std::string type;
  // position of this light
  Vector3 position;
  // point in world that the light points to (for directional lights)
  Vector3 lookat;

  void clear() {
    name.clear();
    type.clear();
    position.clear();
    lookat.clear();
  }
};

// camera class
class Camera
{
public:
  Camera() {this->clear();}
  // name
  std::string name;
  // position of this camera
  Vector3 position;
  // point in world that the camera points to
  Vector3 lookat;
  // up direction in the world
  Vector3 vertical;

  void clear() {
    name.clear();
    position.clear();
    lookat.clear();
    vertical.clear();
  }
};

// static object class
class StaticObject
{
public:
  StaticObject() {this->clear();}
  // name
  std::string name;

  /// position and orientation in world
  Pose origin;

  /// visual element
  my_shared_ptr<Visual> visual;

  /// collision element
  my_shared_ptr<Collision> collision;

  /// if more than one collision element is specified, all collision elements are placed in this array (the collision member points to the first element of the array)
  std::vector<my_shared_ptr<Collision> > collision_array;

  /// if more than one visual element is specified, all visual elements are placed in this array (the visual member points to the first element of the array)
  std::vector<my_shared_ptr<Visual> > visual_array;

  void clear() {
    name.clear();
    origin.clear();
    visual.reset(0);
    collision.reset(0);
    visual_array.clear();
    collision_array.clear();
  }
};

// graphics class
class Graphics
{
public:
  Graphics() {this->clear();}

  // vector of lights in the world
  std::map<std::string, my_shared_ptr<Light>> lights;
  my_shared_ptr<Light> getLight(const std::string& name)
  {
    my_shared_ptr<Light> ptr;
    if (lights.find(name) == lights.end()) {
      ptr.reset(0);
    }
    else {
      ptr = lights.find(name)->second;
    }
    return ptr;
  };

  // vector of static meshes in this world
  std::map<std::string, my_shared_ptr<StaticObject>> static_objects;
  my_shared_ptr<StaticObject> getStaticObject(const std::string& name)
  {
    my_shared_ptr<StaticObject> ptr;
    if (static_objects.find(name) == static_objects.end()) {
      ptr.reset(0);
    }
    else {
      ptr = static_objects.find(name)->second;
    }
    return ptr;
  };

  // vector of cameras to be used in this world
  std::map<std::string, my_shared_ptr<Camera>> cameras;
  my_shared_ptr<Camera> getCamera(const std::string& name)
  {
    my_shared_ptr<Camera> ptr;
    if (cameras.find(name) == cameras.end()) {
      ptr.reset(0);
    }
    else {
      ptr = cameras.find(name)->second;
    }
    return ptr;
  };

  void clear() {
    lights.clear();
    static_objects.clear();
    cameras.clear();    
  }
};

// robot class
class Robot
{
public:
  Robot() {this->clear();}

  std::string name;
  std::string model_working_dir;
  std::string model_filename;
  std::string model_name;
  Pose origin;

  void clear() {
    name.clear();
    model_working_dir.clear();
    model_filename.clear();
    model_name.clear();
    origin.clear();
  }
};

class World
{
public:
  World() { this->clear(); };

  /// world name must be unique
  std::string name_;

  // map of parsed robots in this world
  std::map<std::string, my_shared_ptr<Robot>> models_;

  // get Robot
  my_shared_ptr<Robot> getRobot(const std::string& name)
  {
    my_shared_ptr<Robot> ptr;
    if (models_.find(name) == models_.end()) {
      ptr.reset(0);
    }
    else {
      ptr = models_.find(name)->second;
    }
    return ptr;
  };

  // gravity vector in this world
  Vector3 gravity_; //TODO: shift to using Eigen

  // graphics specs for this world
  Graphics graphics_;

  void clear()
  {
    name_.clear();
    models_.clear();
    gravity_.clear();
    graphics_.clear();
  };
};
}

#endif

