// 01-robot-parse: example of how the parser loads the model

#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

const string robot_fname = "resources/pbot.urdf";

int main (int argc, char ** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	// load file
	ifstream model_file (robot_fname);
	if (!model_file) {
		cerr << "Error opening file '" << robot_fname << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, ios::beg);
	model_xml_string.assign((istreambuf_iterator<char>(model_file)), istreambuf_iterator<char>());

	model_file.close();

	// parse xml string
	auto model_ptr = urdf::parseURDF(model_xml_string);

	// print model properties
	cout << "Robot name: " << model_ptr->getName() << "\n";
	cout << "Robot root name: " << model_ptr->getRoot()->name << "\n";
	cout << "Robot # links: " << model_ptr->m_numLinks << "\n";
	uint link_ctr = 0;
	for (auto link_itr: model_ptr->links_) {
		cout << " - Link " << link_ctr << ": " << link_itr.first << "\n";
		link_ctr++;
	}
	cout << "Robot # joints: " << model_ptr->m_numJoints << "\n";
	uint joint_ctr = 0;
	for (auto joint_itr: model_ptr->joints_) {
		cout << " - Joint " << joint_ctr << ": " << joint_itr.first << "\n";
		joint_ctr++;
	}
	cout << endl;
	return 0;
}