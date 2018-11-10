// 02-robot-parse: example of how the parser loads the world

#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

const string world_fname = "resources/world.urdf";

int main (int argc, char ** argv) {
	cout << "Loading world file: " << world_fname << endl;

	// load file
	ifstream world_file (world_fname);
	if (!world_file) {
		cerr << "Error opening file '" << world_fname << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string world_xml_string;
	world_file.seekg(0, ios::end);
	world_xml_string.reserve(world_file.tellg());
	world_file.seekg(0, ios::beg);
	world_xml_string.assign((istreambuf_iterator<char>(world_file)), istreambuf_iterator<char>());

	world_file.close();

	// parse xml string
	auto world_ptr = urdf::parseURDFWorld(world_xml_string);

	// print world properties
	cout << "World name: " << world_ptr->name_ << "\n";
	cout << "Gravity: " << world_ptr->gravity_.x << ", " << world_ptr->gravity_.y << ", " << world_ptr->gravity_.z << "\n";
	uint model_ctr = 0;
	cout << "# Models: " << world_ptr->models_.size() << "\n";
	for (auto model_itr: world_ptr->models_) {
		cout << "- Model " << model_ctr << ": " << model_itr.first << "\n";
		model_ctr++;
	}
	uint light_ctr = 0;
	cout << "# Lights: " << world_ptr->graphics_.lights.size() << "\n";
	for (auto lights_itr: world_ptr->graphics_.lights) {
		cout << "- Light " << light_ctr << ": " << lights_itr.first << "\n";
		light_ctr++;
	}
	uint camera_ctr = 0;
	cout << "# Cameras: " << world_ptr->graphics_.cameras.size() << "\n";
	for (auto camera_itr: world_ptr->graphics_.cameras) {
		cout << "- Light " << camera_ctr << ": " << camera_itr.first << "\n";
		camera_ctr++;
	}
	uint sobject_ctr = 0;
	cout << "# Static objects: " << world_ptr->graphics_.static_objects.size() << "\n";
	for (auto sobject_itr: world_ptr->graphics_.static_objects) {
		cout << "- Static object " << sobject_ctr << ": " << sobject_itr.first << "\n";
		sobject_ctr++;
	}
	cout << flush;
	return 0;
}