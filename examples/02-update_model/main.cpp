// 02-update_model: 
// example of how to use the different kinematic and dynamic functions

#include <iostream>
#include <Sai2Model.h>

using namespace std;

const string robot_fname = "resources/rpbot.urdf";

int main (int argc, char ** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	Sai2Model::Sai2Model* robot = new Sai2Model::Sai2Model(robot_fname, false);
	int dof = robot->dof();

	const string ee_link = "link1";
	const Eigen::Vector3d ee_pos_in_link = Eigen::Vector3d(0.0, 0.0, 1.0);

	// position and orientation of the frame attached to the second joint
	Eigen::Vector3d position;
	Eigen::Vector3d velocity;
	Eigen::Matrix3d rotation;
	Eigen::MatrixXd J(6,dof);
	Eigen::VectorXd gravity(dof);

	// position and orientation of the end effector
	robot->position(position, ee_link, ee_pos_in_link);
	robot->linearVelocity(velocity, ee_link, ee_pos_in_link);
	robot->rotation(rotation, ee_link);
	// jacobian at the end effector (1m from second joint)
	robot->J_0(J,ee_link,ee_pos_in_link);
	// gravity and coriolis/centrifugal forces
	robot->gravityVector(gravity);

	cout << "------------------------------------------------------" << endl;
	cout << "               Initial configuration                  " << endl;
	cout << "------------------------------------------------------" << endl;
	cout << endl;
	cout << "robot coordinates : " << robot->_q.transpose() << endl;
	cout << "position at end effector : " << position.transpose() << endl;
	cout << "velocity at end effector : " << velocity.transpose() << endl;
	cout << "orientation at end effector : \n" << rotation << endl;
	cout << "jacobian at the end effector \n" << J << endl;
	cout << "Mass matrix :\n" << robot->_M << endl;
	cout << "joint gravity : " << gravity.transpose() << endl;
	cout << endl;

	// modify joint positions and velocities
	robot->_q << -0.5*M_PI, 1;
	robot->_dq << 0, 1;
	robot->position(position, ee_link, Eigen::Vector3d(0.0, 0.0, 0.0));
	robot->linearVelocity(velocity, ee_link, ee_pos_in_link);
	robot->rotation(rotation, ee_link);
	robot->J_0(J,ee_link,ee_pos_in_link);
	robot->gravityVector(gravity);

	cout << endl;
	cout << "------------------------------------------------------" << endl;
	cout << "  we modify the joint positions to 90 degrees and 1m  " << endl;
	cout << " nothing will change before we call updateKinematics()" << endl;
	cout << "------------------------------------------------------" << endl;
	cout << endl;
	cout << "robot coordinates : " << robot->_q.transpose() << endl;
	cout << "position at end effector : " << position.transpose() << endl;
	cout << "velocity at end effector : " << velocity.transpose() << endl;
	cout << "orientation at end effector : \n" << rotation << endl;
	cout << "Mass matrix :\n" << robot->_M << endl;
	cout << "jacobian at the end effector \n" << J << endl;
	cout << "joint gravity : " << gravity.transpose() << endl;
	cout << endl;

	// update kinematics
	robot->updateKinematics();
	robot->position(position, ee_link, Eigen::Vector3d(0.0, 0.0, 0.0));
	robot->linearVelocity(velocity, ee_link, ee_pos_in_link);
	robot->rotation(rotation, ee_link);
	robot->J_0(J,ee_link,ee_pos_in_link);
	robot->gravityVector(gravity);

	cout << endl;
	cout << "------------------------------------------------------" << endl;
	cout << "               call to updateKinematics()             " << endl;
	cout << "      Everything updated except the mass matrix       " << endl;
	cout << "------------------------------------------------------" << endl;
	cout << endl;
	cout << "robot coordinates : " << robot->_q.transpose() << endl;
	cout << "position at end effector : " << position.transpose() << endl;
	cout << "velocity at end effector : " << velocity.transpose() << endl;
	cout << "orientation at end effector : \n" << rotation << endl;
	cout << "Mass matrix :\n" << robot->_M << endl;
	cout << "jacobian at the end effector \n" << J << endl;
	cout << "joint gravity : " << gravity.transpose() << endl;
	cout << endl;

	// update dynamics
	robot->updateDynamics();

	cout << endl;
	cout << "------------------------------------------------------" << endl;
	cout << "               call to updateDynamics()               " << endl;
	cout << "   The mass matrix (and its inverse) is recomputed    " << endl;
	cout << "------------------------------------------------------" << endl;
	cout << endl;
	cout << "robot coordinates : " << robot->_q.transpose() << endl;
	cout << "Mass matrix :\n" << robot->_M << endl;
	cout << endl;

	// update model : everything at the same time
	// come back to initial position
	robot->_q << 0,0;
	robot->_dq << 0,0;
	robot->updateModel();

	robot->position(position, ee_link, Eigen::Vector3d(0.0, 0.0, 0.0));
	robot->linearVelocity(velocity, ee_link, ee_pos_in_link);
	robot->rotation(rotation, ee_link);
	robot->J_0(J,ee_link,ee_pos_in_link);
	robot->gravityVector(gravity);

	cout << endl;
	cout << "------------------------------------------------------" << endl;
	cout << "               back to initial position               " << endl;
	cout << " call updateModel() to update kinematics and dynamics " << endl;
	cout << "------------------------------------------------------" << endl;
	cout << endl;
	cout << "robot coordinates : " << robot->_q.transpose() << endl;
	cout << "position at end effector : " << position.transpose() << endl;
	cout << "velocity at end effector : " << velocity.transpose() << endl;
	cout << "orientation at end effector : \n" << rotation << endl;
	cout << "Mass matrix :\n" << robot->_M << endl;
	cout << "jacobian at the end effector \n" << J << endl;
	cout << "joint gravity : " << gravity.transpose() << endl;
	cout << endl;

	return 0;
}