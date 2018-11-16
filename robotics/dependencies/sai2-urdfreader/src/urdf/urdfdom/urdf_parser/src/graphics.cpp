/**
 * \file graphics.cpp
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#include <urdf/urdfdom_headers/urdf_world/include/urdf_world/world.h>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/link.h>
#include "urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h"

#include <tinyxml2.h>
#include "urdf/boost_replacement/printf_console.h"

namespace urdf{

// function to parse pose. defined in pose.cpp
bool parsePose(Pose &pose, tinyxml2::XMLElement* xml);

// function to parse collisions. defined in link.cpp
bool parseCollision(Collision &col, tinyxml2::XMLElement* config);

// function to parse visuals. defined in link.cpp
bool parseVisual(Visual &vis, tinyxml2::XMLElement *config);

void parseLight(my_shared_ptr<Light>& light, tinyxml2::XMLElement* xml) {
	// get name
	const char *name = xml->Attribute("name");
	assert(name);
	light->name = std::string(name);
	assert(!light->name.empty());

	// get type (optional)
	const char *type = xml->Attribute("type");
	if (type) {
		light->type = std::string(type);
	}

	// get position (optional)
	tinyxml2::XMLElement* position_xml = xml->FirstChildElement("position");
	if (position_xml) {
		const char *position = position_xml->Attribute("xyz");
		light->position.init(position);
	}

	// get lookat (optional)
	tinyxml2::XMLElement* lookat_xml = xml->FirstChildElement("lookat");
	if (lookat_xml) {
		const char *lookat = lookat_xml->Attribute("xyz");
		light->lookat.init(lookat);
	}
}

void parseCamera(my_shared_ptr<Camera>& camera, tinyxml2::XMLElement* xml) {
	// get name
	const char *name = xml->Attribute("name");
	assert(name);
	camera->name = std::string(name);
	assert(!camera->name.empty());

	// get position
	tinyxml2::XMLElement* position_xml = xml->FirstChildElement("position");
	assert(position_xml);
	const char *position = position_xml->Attribute("xyz");
	camera->position.init(position);

	// get lookat
	tinyxml2::XMLElement* lookat_xml = xml->FirstChildElement("lookat");
	assert(lookat_xml);
	const char *lookat = lookat_xml->Attribute("xyz");
	camera->lookat.init(lookat);

	// get vertical direction
	tinyxml2::XMLElement* vertical_xml = xml->FirstChildElement("vertical");
	assert(vertical_xml);
	const char *vertical = vertical_xml->Attribute("xyz");
	camera->vertical.init(vertical);
}

void parseStaticObject(my_shared_ptr<StaticObject>& object, tinyxml2::XMLElement* xml) {
	object->clear();

	// parse name
	const char *name = xml->Attribute("name");
	assert(name);
	object->name = std::string(name);

	// parse pose
	tinyxml2::XMLElement* pose_xml = xml->FirstChildElement("origin");
	bool success = parsePose(object->origin, pose_xml);
	assert(success);

	// Multiple Visuals (optional)
	for (tinyxml2::XMLElement* vis_xml = xml->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
	{
		my_shared_ptr<Visual> vis;
		vis.reset(new Visual());
		bool success = parseVisual(*vis, vis_xml);
		assert(success);
		object->visual_array.push_back(vis);
	}

	// Visual (optional)
	// Assign the first visual to the .visual ptr, if it exists
	if (!object->visual_array.empty()) {
		object->visual = object->visual_array[0];
	}

	// Multiple Collisions (optional)
	for (tinyxml2::XMLElement* col_xml = xml->FirstChildElement("collision"); col_xml; col_xml = col_xml->NextSiblingElement("collision"))
	{
		my_shared_ptr<Collision> col;
		col.reset(new Collision());
		bool success = parseCollision(*col, col_xml);
		assert(success);
		object->collision_array.push_back(col);
	}

	// Collision (optional) 
	// Assign the first collision to the .collision ptr, if it exists
	if (!object->collision_array.empty()) {
		object->collision = object->collision_array[0];
	}
}

}
