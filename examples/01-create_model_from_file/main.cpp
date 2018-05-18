// 01-create_model_from_file: example of how to create a robot model from a urdf file

#include <iostream>
// #include <string>
#include <Sai2Model.h>

using namespace std;

const string robot_fname = "resources/pbot.urdf";

int main (int argc, char ** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	Sai2Model::Sai2Model robot = Sai2Model::Sai2Model(robot_fname, true);

	cout << "robot degrees of freedom : " << robot.dof() << endl;
	cout << "robot coordinates : " << robot._q.transpose() << endl;

	return 0;
}