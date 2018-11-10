/*
 * Sai2Model.h
 *
 *  Created on: Dec 14, 2016
 *      Author: XXX
 */

#ifndef SAI2MODEL_H_
#define SAI2MODEL_H_

#include <rbdl/Model.h>

namespace Sai2Model
{

enum ContactNature {PointContact, SurfaceContact};

class Sai2Model
{
public:
    // Sai2Model ();
    Sai2Model (const std::string path_to_model_file,
               bool verbose=true,
               const Eigen::Vector3d world_gravity=Eigen::Vector3d(0.0,0.0,-9.81),
               const Eigen::Affine3d position_in_world=Eigen::Affine3d::Identity());
    ~Sai2Model ();


    /**
     * @brief update the kinematics.
     */
    void updateKinematics();

    /**
     * @brief update the dynamics.
     */
    void updateDynamics();

    /**
     * @brief update the kinematics and dynamics. Effectively calls the two previous functions
     */
    void updateModel();

    /**
     * @brief      returns the number of degrees of freedom of the robot
     */
    int dof();

    /**
     * @brief Gives the joint gravity torques vector of the last updated configuration using the model world gravity
     * @param g Vector to which the joint gravity torques will be written
     */
    void gravityVector(Eigen::VectorXd& g);


    /**
     * @brief Gives the joint gravity torques vector of the last updated configuration suing a custom world gravity vector
     * @param g Vector to which the joint gravity torques will be written
     * @param gravity the 3d gravity vector of the world in base frame
     */
    void gravityVector(Eigen::VectorXd& g,
                               const Eigen::Vector3d& gravity);

    /**
     * @brief Gives the joint coriolis and centrifugal forces of the last updated configuration
     * @param b Vector to which the joint coriolis and centrifugal forces will be written
     */
    void coriolisForce(Eigen::VectorXd& b);

    /**
     * @brief Computes the modified Newton-Euler Algorithm as described in
     * '''
     * De Luca, A., & Ferrajoli, L. (2009, May).
     * A modified Newton-Euler method for dynamic computations in robot fault detection and control.
     * In Robotics and Automation, 2009. ICRA'09. IEEE International Conference on (pp. 3359-3364). IEEE.
     * '''
     * @tau                       return vector
     * @param consider_gravity    consider or not the acceleration due to gravity at the base
     * @param q                   joint positions
     * @param dq                  joint velocity
     * @param dqa                 auxiliary joint velocity
     * @param ddq                 joint acceleration
     */
    void modifiedNewtonEuler(Eigen::VectorXd& tau,
                                const bool consider_gravity,
                                const Eigen::VectorXd& q,
                                const Eigen::VectorXd& dq,
                                const Eigen::VectorXd& dqa,
                                const Eigen::VectorXd& ddq);

    void factorizedChristoffelMatrix(Eigen::MatrixXd& C);

    /**
     * @brief Full jacobian for link, relative to base (id=0) in the form [Jv; Jw]
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     */
    void J_0(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link);

    /**
     * @brief Full jacobian for link, relative to base (id=0) in the form [Jw; Jv]
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     * @param q Joint positions
     */
    void J(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link);


    /**
     * @brief Velocity jacobian for point on link, relative to base (id=0)
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     */
    void Jv(Eigen::MatrixXd& J,
                    const std::string& link_name,
                    const Eigen::Vector3d& pos_in_link);


    /**
     * @brief Angular velocity jacobian for link, relative to base (id=0)
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     */
    void Jw(Eigen::MatrixXd& J,
                    const std::string& link_name);


    /**
     * @brief transformation from base to link, in base coordinates.
     * @param T Transformation matrix to which the result is computed
     * @param link_name name of the link where to compute the transformation matrix
     */
    void transform(Eigen::Affine3d& T,
                           const std::string& link_name);

    /**
     * @brief transformation from base to link at the given position, in base coordinates.
     * @param T Transformation matrix to which the result is computed
     * @param link_name name of the link where to compute the transformation matrix
     */
    void transform(Eigen::Affine3d& T,
                           const std::string& link_name,
                           const Eigen::Vector3d& pos_in_body);

    /**
     * @brief transformation from world origin to link, in world coordinates.
     * @param T Transformation matrix to which the result is computed
     * @param link_name name of the link where to compute the transformation matrix
     */
    void transformInWorld(Eigen::Affine3d& T,
                           const std::string& link_name);

    /**
     * @brief transformation from world origin to link at the given position, in world coordinates.
     * @param T Transformation matrix to which the result is computed
     * @param link_name name of the link where to compute the transformation matrix
     */
    void transformInWorld(Eigen::Affine3d& T,
                           const std::string& link_name,
                           const Eigen::Vector3d& pos_in_body);

    /**
     * @brief Position from base to point in link, in base coordinates
     * @param pos Vector of position to which the result is written
     * @param link_name name of the link in which is the point where to compute the position
     * @param pos_in_link the position of the point in the link, in local link frame
     */
    void position(Eigen::Vector3d& pos,
                          const std::string& link_name,
                          const Eigen::Vector3d& pos_in_link);

    /**
     * @brief Position from world origin to point in link, in world coordinates
     * @param pos Vector of position to which the result is written
     * @param link_name name of the link in which is the point where to compute the position
     * @param pos_in_link the position of the point in the link, in local link frame
     */
    void positionInWorld(Eigen::Vector3d& pos,
                          const std::string& link_name,
                          const Eigen::Vector3d& pos_in_link);

    /**
     * @brief Velocity of point in link, in base coordinates
     * @param vel Vector of velocities to which the result is written
     * @param link_name name of the link in which is the point where to compute the velocity
     * @param pos_in_link the position of the point in the link, in local link frame
     */
    void linearVelocity(Eigen::Vector3d& vel,
                          const std::string& link_name,
                          const Eigen::Vector3d& pos_in_link);

    /**
     * @brief Acceleration of point in link, in base coordinates
     * @param accel Vector of accelerations to which the result is written
     * @param link_name name of the link in which is the point where to compute the acceleration
     * @param pos_in_link the position of the point in the link, in local link frame
     */
    void linearAcceleration(Eigen::Vector3d& accel,
                              const std::string& link_name,
                              const Eigen::Vector3d& pos_in_link);

    /**
     * @brief Rotation of a link with respect to base frame
     * @param rot Rotation matrix to which the result is written
     * @param link_name name of the link for which to compute the rotation
     */
    void rotation(Eigen::Matrix3d& rot,
                          const std::string& link_name);

    /**
     * @brief Rotation of a link with respect to the world origin
     * @param rot Rotation matrix to which the result is written
     * @param link_name name of the link for which to compute the rotation
     */
    void rotationInWorld(Eigen::Matrix3d& rot,
                          const std::string& link_name);

    /**
     * @brief Angular velocity of a link with respect to base frame
     * @param avel Vector to which the result is written
     * @param link_name name of the link for which to compute the angular velocity
     */
    void angularVelocity(Eigen::Vector3d& avel,
                                 const std::string& link_name);

    /**
     * @brief Angular acceleration of a link with respect to base frame
     * @param aaccel Vector to which the result is written
     * @param link_name name of the link for which to compute the angular acceleration
     */
    void angularAcceleration(Eigen::Vector3d& aaccel,
                                     const std::string& link_name);

    /**
     * @brief Gives the link id for a given name with the right indexing for rbdl
     * @param link_name name of the link
     */
    unsigned int linkId(const std::string& link_name);

    /**
     * @brief Gives the joint id for a given name with the right indexing for rbdl
     * @param joint_name name of the joint
     */
    int jointId(const std::string& joint_name);

    /**
     * @brief Gives the mass properties of a given link
     * @param mass the returned mass value
     * @param center_of_mass the position of the center of mass in the body's frame
     * @param inertia the inertia of the given link
     * @param link_name the name of the considered link
     */
    void getLinkMass(double& mass,
                     Eigen::Vector3d& center_of_mass,
                     Eigen::Matrix3d& inertia,
                     const std::string& link_name);

    /**
     * @brief Gives the mass properties of a given link
     * @param mass the returned mass value
     * @param center_of_mass the position of the center of mass in the body's frame
     * @param link_name the name of the considered link
     */
    void getLinkMass(double& mass,
                     Eigen::Vector3d& center_of_mass,
                     const std::string& link_name);



    /**
     * @brief Computes the operational space matrix corresponding to a given Jacobian
     * @param Lambda Matrix on which the operational space mass matrix will be written
     * @param task_jacobian The jacobian of the task for which we want the op space mass matrix
     */
    void taskInertiaMatrix(Eigen::MatrixXd& Lambda,
                           const Eigen::MatrixXd& task_jacobian);

    /**
     * @brief Computes the operational space matrix robust to singularities
     * @param Lambda Matrix on which the operational space mass matrix will be written
     * @param task_jacobian The jacobian of the task for which we want the op space mass matrix
     */
    void taskInertiaMatrixWithPseudoInv(Eigen::MatrixXd& Lambda,
                           const Eigen::MatrixXd& task_jacobian);

    /**
     * @brief      Computes the dynamically consistent inverse of the jacobian for a given task. Recomputes the task inertia at each call
     *
     * @param      Jbar           Matrix to which the dynamically consistent inverse will be written
     * @param[in]  task_jacobian  The task jacobian
     */
    void dynConsistentInverseJacobian(Eigen::MatrixXd& Jbar,
                                      const Eigen::MatrixXd& task_jacobian);


    /**
     * @brief      Computes the nullspace matrix for the highest priority task. Recomputes the dynamically consistent inverse and the task mass matrix at each call
     *
     * @param      N              Matrix to which the nullspace matrix will be written
     * @param[in]  task_jacobian  The task jacobian
     */
    void nullspaceMatrix(Eigen::MatrixXd& N,
                             const Eigen::MatrixXd& task_jacobian);

    /**
     * @brief      Computes the nullspace matrix of the task, consistent with the previous nullspace
     *             Recomputes the dynamically consistent inverse and the task mass matrix at each call
     *
     * @param      N              Matrix to which the nullspace matrix will be written
     * @param[in]  task_jacobian  The task jacobian
     * @param[in]  N_prec         The previous nullspace matrix
     */
    void nullspaceMatrix(Eigen::MatrixXd& N,
                             const Eigen::MatrixXd& task_jacobian,
                             const Eigen::MatrixXd& N_prec);

    /**
     * @brief      Computes the operational spce matrices (task inertia, dynamically consistent inverse of the jacobian and nullspace) for a given task,
     *             for the first task. More efficient than calling the three individual functions.
     *
     * @param      Lambda         Matrix to which the operational space mass matrix will be written
     * @param      Jbar           Matrix to which the dynamically consistent inverse of the jacobian will be written
     * @param      N              Matrix to which the nullspace matrix will be written
     * @param[in]  task_jacobian  Task jacobian
     */
    void operationalSpaceMatrices(Eigen::MatrixXd& Lambda, Eigen::MatrixXd& Jbar, Eigen::MatrixXd& N,
                                    const Eigen::MatrixXd& task_jacobian);

    /**
     * @brief      Computes the operational spce matrices (task inertia, dynamically consistent inverse of the jacobian and nullspace) for a given task,
     *             In the nullspace of the previous task. More efficient than calling the three individual functions.
     *
     * @param      Lambda         Matrix to which the operational space mass matrix will be written
     * @param      Jbar           Matrix to which the dynamically consistent inverse of the jacobian will be written
     * @param      N              Matrix to which the nullspace matrix will be written
     * @param[in]  task_jacobian  Task jacobian
     * @param[in]  N_prec         Previous nullspace matrix
     */
    void operationalSpaceMatrices(Eigen::MatrixXd& Lambda, Eigen::MatrixXd& Jbar, Eigen::MatrixXd& N,
                                    const Eigen::MatrixXd& task_jacobian,
                                    const Eigen::MatrixXd& N_prec);


    /**
     * @brief Computes the grasp matrix in the cases where there are
     * 2, 3 or 4 contact points.
     * the external forces and moments are assumed to be in world frame
     * for 2 contact points, the output quantities are given in local frame, and the description of the local frame is given by R
     * for 3 and 4 contacts, the output quantities are given in world frame
     * the convention for the output is the following order : support forces, support moments, internal tensions, internal moments
     * the internal tensions are given in the order 1-2, 1-3, 2-3 in the 3 contact case
     * and 1-2, 1-3, 1-4, 2-3, 2-4, 3-4 in the 4 contact case.
     * @param G  :  The grasp matrix that is going to be populated
     * @param R : the rotation matrix between the world frame and the frame attached to the object (useful when 2 contacts only)
     * @param link_names  :  a vector of the names of the links where the contact occur
     * @param pos_in_links  :  a vector of the position of the contact in each link
     * @param contact_natures  :  a vector containing the nature of each contact (we only consider point contact and surface contact)
     * @param center_point  :  The position (in world frame) of the point on which we resolve the resultant forces and moments
     */
    void GraspMatrix(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     const std::vector<std::string> link_names,
                     const std::vector<Eigen::Vector3d> pos_in_links,
                     const std::vector<ContactNature> contact_natures,
                     const Eigen::Vector3d center_point);

    /**
     * @brief Computes the grasp matrix in the cases where there are
     * 2, 3 or 4 contact points.
     * @param G  :  The grasp matrix that is going to be populated
     * @param R : the rotation matrix between the world frame and the frame attached to the object (useful when 2 contacts only)
     * @param geopetric_center  :  The position (in world frame) of the geometric center (found and returned by the function) on which we resolve the resultant forces and moments
     * @param link_names  :  a vector of the names of the links where the contact occur
     * @param pos_in_links  :  a vector of the position of the contact in each link
     * @param contact_natures  :  a vector containing the nature of each contact (we only consider point contact and surface contact)
     */
    void GraspMatrixAtGeometricCenter(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center,
                     const std::vector<std::string> link_names,
                     const std::vector<Eigen::Vector3d> pos_in_links,
                     const std::vector<ContactNature> contact_natures);


    /// \brief internal rbdl model
    RigidBodyDynamics::Model* _rbdl_model;


    /// \brief Joint positions
    Eigen::VectorXd _q;

    /// \brief Joint velocities
    Eigen::VectorXd _dq;

    /// \brief Joint accelerations
    Eigen::VectorXd _ddq;

    /// \brief Mass Matrix
    Eigen::MatrixXd _M;

    /// \brief Inverse of the mass matrix
    Eigen::MatrixXd _M_inv;

public:
    /// \brief compute the cross product operator of a 3d vector
    static Eigen::Matrix3d CrossProductOperator(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d v_hat;
        v_hat << 0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(0), 0;
        return v_hat;
    }

    /// \brief number of Dof of robot
    int _dof;

    /// \brief Transform from world coordinates to robot base coordinates
    Eigen::Affine3d _base_position_in_world;

protected:
    /// \brief map from joint names to joint id
    std::map<std::string,int> _joint_names_map;

};

 /**
 * @brief Gives orientation error from rotation matrices
 * @param delta_phi Vector on which the orientation error will be written
 * @param desired_orientation desired orientation rotation matrix
 * @param current_orientation current orientation matrix
 */
void orientationError(Eigen::Vector3d& delta_phi,
                      const Eigen::Matrix3d& desired_orientation,
                      const Eigen::Matrix3d& current_orientation);


/**
 * @brief Gives orientation error from quaternions
 * @param delta_phi Vector on which the orientation error will be written
 * @param desired_orientation desired orientation quaternion
 * @param current_orientation current orientation quaternion
 */
void orientationError(Eigen::Vector3d& delta_phi,
                      const Eigen::Quaterniond& desired_orientation,
                      const Eigen::Quaterniond& current_orientation);


} /* namespace Model */

#endif /* RBDLMODEL_H_ */
