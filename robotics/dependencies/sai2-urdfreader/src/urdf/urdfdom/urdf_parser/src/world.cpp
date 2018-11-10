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

/* Author: Wim Meeussen */


#include <urdf/urdfdom_headers/urdf_world/include/urdf_world/world.h>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h>
#include "urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h"

#include <tinyxml2.h>
#include "urdf/boost_replacement/printf_console.h"

namespace urdf{

// function to parse pose. defined in pose.cpp
bool parsePose(Pose &pose, tinyxml2::XMLElement* xml);

// function to parse a static object in this world. defined in graphics.cpp
void parseStaticObject(my_shared_ptr<StaticObject>& object, tinyxml2::XMLElement* xml);

// function to parse a light in this world. defined in graphics.cpp
void parseLight(my_shared_ptr<Light>& light, tinyxml2::XMLElement* xml);

// function to parse a camera in this world. defined in graphics.cpp
void parseCamera(my_shared_ptr<Camera>& camera, tinyxml2::XMLElement* xml);

// function to parse robot spec
static void parseRobot(my_shared_ptr<Robot>& robot, tinyxml2::XMLElement* robot_xml) {
	// get name
	const char *name = robot_xml->Attribute("name");
	assert(name);
	robot->name = std::string(name);
	assert(!robot->name.empty());

	// get model path
	tinyxml2::XMLElement* model_xml = robot_xml->FirstChildElement("model");
	assert(model_xml);
	const char *model_filename = model_xml->Attribute("path");
	assert(model_filename);
	robot->model_filename = std::string(model_filename);
	assert(!robot->model_filename.empty());
	const char *model_name = model_xml->Attribute("name");
	assert(model_name);
	robot->model_name = std::string(model_name);
	assert(!robot->model_name.empty());

	// get model working directory
	const char *model_working_dir = model_xml->Attribute("dir");
	if (model_working_dir) {
		robot->model_working_dir = std::string(model_working_dir);
	}

	// get origin
	tinyxml2::XMLElement* pose_xml = robot_xml->FirstChildElement("origin");
	bool success = parsePose(robot->origin, pose_xml);
	assert(success);
}

my_shared_ptr<World> parseURDFWorld(const std::string &xml_string)
{
	my_shared_ptr<World> world(new World);
	world->clear();

	tinyxml2::XMLDocument xml_doc;
	xml_doc.Parse(xml_string.c_str());
	if (xml_doc.Error())
	{
		logError(xml_doc.GetErrorStr1());
	    assert(0);
	}

	// parse world attributes
	// NOTE: we currently support only one world specification per file
	tinyxml2::XMLElement *world_xml = xml_doc.FirstChildElement("world");
	assert(world_xml);

	// get world name
	const char *name = world_xml->Attribute("name");
	assert(name);
	world->name_ = std::string(name);
	assert(!world->name_.empty());

	// parse gravity specification if any
	const char* gravity_str = world_xml->Attribute("gravity");
	if (gravity_str) {
		world->gravity_.init(gravity_str);
	}

	// get all robot elements
	for (tinyxml2::XMLElement* robot_xml = world_xml->FirstChildElement("robot"); robot_xml; robot_xml = robot_xml->NextSiblingElement("robot"))
	{
		my_shared_ptr<Robot> robot(new Robot);
		parseRobot(robot, robot_xml);
		// ensure no duplication of robot names
		assert(!world->getRobot(robot->name));
		world->models_.insert(make_pair(robot->name, robot));
		logDebug("urdfdom: successfully parsed a new robot in the world '%s'", robot->name.c_str());
	}

	// get all static mesh elements
	for (tinyxml2::XMLElement* object_xml = world_xml->FirstChildElement("static_object"); object_xml; object_xml = object_xml->NextSiblingElement("static_object"))
	{
		my_shared_ptr<StaticObject> object(new StaticObject);
		parseStaticObject(object, object_xml);
		// ensure no duplication of object names
		assert(!world->graphics_.getStaticObject(object->name));
		world->graphics_.static_objects.insert(make_pair(object->name, object));
		logDebug("urdfdom: successfully parsed a new static object in the world '%s'", object->name.c_str());
	}

	// get all graphics elements: lights
	for (tinyxml2::XMLElement* light_xml = world_xml->FirstChildElement("light"); light_xml; light_xml = light_xml->NextSiblingElement("light"))
	{
		my_shared_ptr<Light> light(new Light);
		parseLight(light, light_xml);
		// ensure no duplication of light names
		assert(!world->graphics_.getLight(light->name));
		world->graphics_.lights.insert(make_pair(light->name, light));
		logDebug("urdfdom: successfully parsed a new light in the world '%s'", light->name.c_str());
	}

	// get all graphics elements: cameras
	for (tinyxml2::XMLElement* camera_xml = world_xml->FirstChildElement("camera"); camera_xml; camera_xml = camera_xml->NextSiblingElement("camera"))
	{
		my_shared_ptr<Camera> camera(new Camera);
		parseCamera(camera, camera_xml);
		// ensure no duplication of camera names
		assert(!world->graphics_.getCamera(camera->name));
		world->graphics_.cameras.insert(make_pair(camera->name, camera));
		logDebug("urdfdom: successfully parsed a new camera in the world '%s'", camera->name.c_str());
	}

	return world;
}

bool exportWorld(World &world, tinyxml2::XMLElement* xml)
{
  // to be implemented
  // exportModels(*world.models, world_xml);
  return true;
}

}
