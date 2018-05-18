#ifndef SAI2_URDF_TO_RBDLMODEL_H
#define SAI2_URDF_TO_RBDLMODEL_H

#include <rbdl/rbdl_config.h>
#include "urdf_parser/urdf_parser.h"


namespace RigidBodyDynamics {

struct Model;

	RBDL_DLLAPI bool URDFReadFromFile (const char* filename, Model* model, std::map<std::string,int>& joint_names_map, bool floating_base, bool verbose = false, Eigen::Vector3d world_gravity = Eigen::Vector3d(0.0,0.0,-9.81));
	RBDL_DLLAPI bool URDFReadFromString (const char* model_xml_string, Model* model, std::map<std::string,int>& joint_names_map, bool floating_base, bool verbose = false, Eigen::Vector3d world_gravity = Eigen::Vector3d(0.0,0.0,-9.81));


}

/* _RBDL_URDFREADER_H */
#endif
