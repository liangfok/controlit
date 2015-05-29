/*
 * Copyright (C) 2015 The University of Texas at Austin and the
 * Institute of Human Machine Cognition. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 2.1 of
 * the License, or (at your option) any later version. See
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#ifndef __CONTROLIT_CORE_CONTROL_MODEL_HPP__
#define __CONTROLIT_CORE_CONTROL_MODEL_HPP__

#include <string>
#include <memory>
#include <map>
#include <vector>

#include <rbdl/rbdl.h>

#include <controlit/RobotState.hpp>
#include <controlit/ConstraintSet.hpp>
#include <controlit/VirtualLinkageModel.hpp>
#include <controlit/utility/ControlItParameters.hpp>

// For checking validity of 'A' matrix after it is updated
#include <controlit/utility/ContainerUtility.hpp>

#include "ros/ros.h"

#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;

namespace controlit {

/*!
 * \brief Forward declarations of classes needed by the model
 */
class ConstraintSet; //internal (transmission) and external (contact) constraints

/*!
 * The control model is a container that encapsulates both the robot model
 * and behavioral objectives.  Its main elements consists of a Robot
 * Model, Compound Task, Constraint Set, and Virtual Linkage Model.  The
 * Control Model is designed to contain all state information
 * needed by the controller to compute the next desired joint torque commands.
 */
class ControlModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Typedef for the map from link names to movable parent (RBDL) joint names
  typedef std::map<std::string, std::string> LinkNameToJointNameMap_t;

  /*!
   * The constructor.  Note that init(...) should be called
   * immediately after constructing the ControlModel.
   */
  ControlModel();

  //! Destructor
  virtual ~ControlModel() {}
  /*!
   * A factory method for creating ControlModels from information contained
   * on the ROS parameter server.
   *
   * \param[in] nh The ROS node handle to use when creating the control model.
   * \param[in] latestRobotState A pointer to the latest robot state.
   * \param[out] params A pointer to the WBC parameters.  This is necessary to save the
   * joint limit information when the URDF is parsed.  It can be a nullptr, in which case the joint
   * limit information will not be saved.
   * \return A pointer to the newly created ControlModel.
   */
  static ControlModel* createModel(ros::NodeHandle & nh, RobotState * latestRobotState,
    controlit::utility::ControlItParameters * params);

  /*!
   * A factory method for creating ControlModels from urdf and yaml descriptions.
   *
   * \param[in] urdfDescription The URDF description of the robot.
   * \param[in] yamlConfig The YAML specification of the constraint set.
   * \param[in] latestRobotState A pointer to the latest robot state.
   * \param[out] params A pointer to the WBC parameters.  This is necessary to save the
   * joint limit information when the URDF is parsed.
   * \return A pointer to the newly created ControlModel.
   */
  static ControlModel* createModel(std::string const & urdfDescription,
    std::string const & yamlConfig, RobotState * latestRobotState,
    controlit::utility::ControlItParameters * params);

  /*!
   * Initializes this control model.  This method should only be called once.
   * Ownership of the pointer parameters are transferred to this object.
   *
   * \param[in] model A pointer to the RBDL model.  Ownership of this pointer is passed to this class.
   * \param[in] robotState A pointer to the latest robot state.
   * \param[in] linkNameToJointNameMap A pointer to the link name to joint name map.
   * \param[in] constraints A pointer to the constraint set.
   * \param[in] params The ControlIt! parameters.
   * \return true if successful.
   */
  bool init(
    RigidBodyDynamics::Model * model,
    RobotState * robotState,
    LinkNameToJointNameMap_t * linkNameToJointNameMap,
    ConstraintSet * constraints,
    controlit::utility::ControlItParameters * params);

  /*!
   * Initializes this control model.  This can be called any number of times
   * after init(RigidBodyDynamics::Model * model,
   * LinkNameToJointNameMap_t * linkNameToJointNameMap, ConstraintSet * constraints)
   * is called.
   *
   * \return true if successful.
   */
  bool reinit();

  //! brief Initializes the model used with SysId and SysId itself
  // void initSysId(const std::string & urdfDescription);

  /*!
   * Sets the AMask_ matrix based on
   * groups of joints which are coupled in A_ matrix
   */
  bool setAMask(const std::vector<std::vector<std::string>> & jointGroups);

  /*!
   * Sets the gravMask_ vector.
   * any joint named in the input vector should have a zero in
   */
  bool setGravMask(const std::vector<std::string> & noGravJoints);

  /*!
   * Sets the constraint set used for gravity compensation.
   *
   * \param[in] cs The constraint set that should be used for
   * gravity compensation.
   */
  // void setGravityCompConstraintSet(ConstraintSet * cs);

  /*!
   * Returns true if an alternate constraint set should be used for
   * gravity compensation.
   *
   * \return whether to use an alternate constraint set for
   * gravity compensation.
   */
  // bool useAlternateConstraintSetForGravityComp();

  /*!
   * An accessor to the constraint set that's to be used for
   * gravity compensation.
   *
   * \return A pointer to the constraint set for gravity
   * compensation.
   */
  // ConstraintSet * getGravityCompConstraintSet();


  /*!
   * An accessor to the constraint set.
   *
   * \return A pointer to the constraint set for gravity
   * compensation.
   */
  ConstraintSet * getConstraintSet() { return constraints_.get(); }

  /*!
   * Gets the robot model.
   *
   * \return A reference to the robot model.
   */
  RigidBodyDynamics::Model& rbdlModel();

  /*!
   * Gets the robot model.
   *
   * \return A const reference to the robot model.
   */
  RigidBodyDynamics::Model const& rbdlModel() const;

  //! Convienence function to grab the link name to joint name map
  LinkNameToJointNameMap_t& linkNameToJointNameMap();
  LinkNameToJointNameMap_t const& linkNameToJointNameMap() const;

  //! Get a reference to a whole-body external constraint description
  ConstraintSet& constraints();
  ConstraintSet const& constraints() const;

  //! Get a reference to the virtual linkage model
  VirtualLinkageModel& virtualLinkageModel();
  VirtualLinkageModel const& virtualLinkageModel() const;

  /*!
   * Updates this control model based on the latest joint and base states.
   *
   * Prior to calling this method, the following must be done (order is important):
   *  1. The member variable 'latestRobotState' needs to be updated.
   *  2. The method updateJointState() needs to be called.
   */
  void update();

  /*!
   * Updates the state of the robot's joints (both real and virtual).
   * This uses the information contained within member variable 'latestJointState'.
   */
  void updateJointState();

  /*!
   * Directly sets all of the joints including the virtual links.
   * This is used for debugging purposes.
   */
  void setFullJointState(Vector const& q, Vector const& qd, Vector const& qdd);

  /*!
   * Returns the number of actuatable degrees of freedom in the robot.
   * \note This will only return a valid value after init() or reinit() is called.
   *
   * \return the number of actuatable degrees of freedom in the robot.
   */
  int getNActuableDOFs() const {return NActuableDOFs_;}

  /*!
   * Returns the total number of joints in the robot (virtual and actual).
   */
  int getNumDOFs() const {return numDOFs; }

  /*!
   * Returns the total number of real joints in the robot (actual).
   */
  int getNumRealDOFs() const {return NRealDOFs_; }

  /*!
   * Returns the number of virtual DOFs, which is equal to the
   * total number of DOFs - the number of actuable DOFs.
   */
  int getNumVirtualDOFs() const { return getNumDOFs() - getNumRealDOFs(); }

  /*!
   * Returns the joint positions (both virtual and real).
   *
   * \return A reference to a vector containing the positions of
   * *all* the joints in the robot (both real and virtual).
   */
  inline const Vector & getQ() const {return Q_;}

  /*!
   * Returns the positions of just the actuable joints.
   *
   * \note The cost of this method includes a memory allocation and copy operation!
   * \return A vector containing the joint positions of the actuable joints.
   */
  inline const Vector getActuableQ() const { return constraints_->getU() * Q_; }

  /*!
   * Returns the joint velocities (both virtual and real).
   *
   * \return A reference to a vector containing the velocities of
   * *all* the joints in the robot (both real and virtual).
   */
  inline const Vector & getQd() const {return Qd_;}

  /*!
   * Returns the velocites of just the actuable joints.
   *
   * \note The cost of this method includes a memory allocation and copy operation!
   * \return A vector containing the velocities of the actuable joints.
   */
  inline const Vector getActuableQd() const { return constraints_->getU() * Qd_; }

  /*!
   * Returns the joint accelerations (both virtual and real).
   *
   * \return A reference to a vector containing the accelerations of
   * *all* the joints in the robot (both real and virtual).
   */
  inline const Vector & getQdd() const {return Qdd_;}

  /*!
   * Returns the accelerations of just the actuable joints.
   *
   * \note The cost of this method includes a memory allocation and copy operation!
   * \return A vector containing the accelerations of the actuable joints.
   */
  inline const Vector getActuableQdd() const { return Qdd_.tail(getNActuableDOFs()); }

  /*!
   * Gets A, the joint space inertia matrix.
   * Its dimesions are: # DOFs x # DOFs.
   *
   * \return The joint space inertia matrix.
   */
  inline Matrix const & getA() const {return A_;}

  /*!
   * Gets Ainv, the inverse of A, the joint space inertia matrix.
   * Its dimesions are: # DOFs x # DOFs.
   *
   * \return The inverse of the joint space inertia matrix.
   */
  inline Matrix const & getAinv() const {return Ainv_;}

  /*!
   * Returns the joint space gravity vector, 'grav'.  This is
   * the amount of force due to gravity that each joint feels.
   * The length of this vector includes both real and virtual joints
   * and is equal to the number of DOFs in the RBDL model.
   *
   * \return A reference to the gravity vector.
   */
  inline Vector const & getGrav() const {return grav_;}

  /*!
   * Obtains the Frame ID from the name of the body or its joint
   * (also allowing for "world").
   *
   * \param[in] name The name of the body or its joint.
   * \param[out] frameID The ID of the frame.  This is where the result is stored.
   * If name is "world", -1 is returned.
   * \return Whether the frame ID was successfully obtained.
   */
  bool getFrameID(std::string & name, int & frameID) const;

  /*!
   * Obtains the body ID from the name of the body or its joint
   *
   * \param[in] name The name of the body or its joint.
   * \param[out] bodyID The ID of the body.  This is where the result is stored.
   * \return Whether the body ID was successfully obtained.
   */
  bool getBodyID(const std::string & name, unsigned int & bodyID) const;

  /*!
   * Obtains the command vector's index that contains the state of a
   * particular joint.
   *
   * \param[in] name The name of the joint.
   * \param[out] index The index of the position in the command vector
   * that contains the specified joint's state.
   */
  bool getJointIndex(const std::string & name, unsigned int & index) const;

  /*!
   * Prints the state of the robot.  This is used for debugging purposes.
   */
  void printState();

  /*!
   * Creates a ROS publisher for each joint and saves it in variable
   * jointStatePublishers.  This is used for visualizing the joint
   * positions in rViz.
   */
  // void setupJointStatePubs();

  /*!
   * Computes the COM of each link in the world frame.
   *
   * \param linkCOMs A reference to where the COMs will be stored.
   * It is a matrix of size 3 x number_of_links.
   * \return Whether the method successfully executed.
   */
  bool getLinkCOMs(Matrix & linkCOMs);

  /*!
   * Dumps the contents of the LinkNameToJointNameMap as a string.
   *
   * \param os The output stream to which to dump the text.
   */
  void dumpLinkNameToJointNameMap(std::ostream& os) const;

  /*!
   * Returns a string containing a list of actuable joints.
   */
  std::string getActuatedJointNames() const;

  /*!
   * Returns a vector containing a list of real (aka non-virtual) joints.
   * Their order matches that of the RBDL model.
   *
   * \return The names of the real joints.
   */
  const std::vector<std::string> & getRealJointNamesVector() const;

  /*!
   * Returns a vector containing a list of actuable joints.
   * Their order matches that of the RBDL model.
   *
   * \return The names of the actuated joints.
   */
  const std::vector<std::string> & getActuatedJointNamesVector() const;

  /*!
   * Obtains the name of the link to which the virtual DOFs connect.
   *
   * \return The name of the base link.
   */
  const std::string getBaseLinkName() const;

  /*!
   * Obtains two strings: one containing a list of link names,
   * and another containing a list of joint names.  Note that
   * this includes both actuated and unactuated joints!
   *
   * \param linkNames The list of links.
   * \param jointNames The list of joints.
   * \return Whether the method was successfully executed.
   */
  bool getLinkAndJointNames(std::string & linkNames,
    std::string & jointNames) const;

  /*!
   * An accessor to the latest joint state.
   * \note{This state may be newer than the state that was used to update
   * the RBDL model, gravity vector, mass matrix, constraint set,
   * and virtual linkage model.}
   *
   * \return A pointer to the latest robot state.
   */
  const RobotState * getLatestJointState() const { return latestRobotState; }

  /*!
   * An accessor to the latest full joint state of the robot.  This includes
   * both the virtual and real joints.  It is likely to be newer than the
   * joints states used to update the RBDL model.
   *
   * \param Q[out] the joint position.
   * \param Qd[out] the joint velocity.
   */
  void getLatestFullState(Vector & Q, Vector & Qd);

  /*!
   * Returns the age of this ControlModel.  The age is the amount of
   * time that has passed since the joint states were last updated.
   *
   * \return The number of seconds that have elapsed since the joint
   * states were updated in this ControlModel.
   */
  double getAge();

  /*!
   * Gets the timestamp of this control model.  This is the time
   * when the joint states used in this control model were updated.
   *
   * \return the timetsamp of this control model.
   */
  ros::Time & getTimeStamp();

  /*!
   * Sets the name of this control model.  This is useful when there are
   * multiple control models being used, e.g., in RTControlModel.
   *
   * \param name The name of this control model.
   */
  void setName(std::string name) { this->name = name; }

  /*!
   * Returns the name of this control model.
   */
  const std::string & getName() { return name; }

  /*!
   * Update with sys id
   */
  // bool checkForSysIdUpdates();

  /*!
   * Returns whether this control model is stale.
   */
  bool isStale() { return isStale_; }

  /*!
   * Sets the stale flag to be true.
   */
  void setStale() { isStale_ = true; }

private:
  /*!
   * Whether the init(...) method was called.
   */
  bool initialized_;

  /*!
   * Whether this control model is stale.  It starts stale and becomes
   * unstale when update() is called.  It becomes stale again when
   * the controller is stopped.
   */
  bool isStale_;

  /*!
   * A pointer to the latest robot state.  Note that this is potentially
   * newer than the values in Q_, Qd_, and Qdd_.
   */
  RobotState * latestRobotState;

  /*!
   * The number of actuable degrees of freedom.
   */
  int NActuableDOFs_;

  /*!
   * The number of real (actuated and unactated) degrees of freedom.
   */
  int NRealDOFs_;

  /*!
   * The total number of DOFs in the robot (real and virtual).
   */
  int numDOFs;

  /*!
   * The (virtual + actual) joint state of the robot.
   */
  Vector Q_;

  /*!
   * The (virtual + actual) joint velocity of the robot.
   */
  Vector Qd_;

  /*!
   * The (virtual + actual) joint acceleration of the robot.
   */
  Vector Qdd_;

  /*!
   * The joint state inertia matrix.
   */
  Matrix A_;

  /*!
   * The joint state inertia matrix, unmolested.
   */
  Matrix UnmolestedA_;

  /*!
   * The inverse of the joint state inertia matrix.
   */
  Matrix Ainv_;

  /*!
   * The inverse of the joint state inertia matrix, unmolested.
   */
  Matrix UnmolestedAinv_;

  /*!
   * A "mask" on the A_ matrix to decouple kinematic chains (default no decoupling ==> AMask_ = ones(size(A)))
   */
  Matrix AMask_;

  /*!
   * The gravity vector.
   */
  Vector grav_;

  /*!
   * A "mask" on the grav_ vector to turn gravity compensation off for some joints but not for others
   */
  Vector gravMask_;

  /*!
   * The constraint set.
   */
  std::unique_ptr<ConstraintSet> constraints_;

  /*!
   * The RBDL model.
   */
  std::unique_ptr<RigidBodyDynamics::Model> rbdlModel_;

  /*!
   * The virtual linkage model.
   */
  std::unique_ptr<VirtualLinkageModel> virtualLinkageModel_;

  /*!
   * Maps the link name (a string) to a joint name (a string).
   */
  std::unique_ptr<LinkNameToJointNameMap_t> linkNameToJointNameMap_;

  /*!
   *  For visualizing joint state information.
   */
  std::map<std::string, ros::Publisher> jointStatePublishers;

  /*!
   * Ensures the matrices used in magicSauce remain valid.
   */
  controlit::utility::ContainerUtility containerUtility;

  /*!
   * The time when the joint states were most recently updated.
   */
  high_resolution_clock::time_point jointStateTimeStamp;

  /*!
   * The same as jointStateTimeStamp except stored as a ros::Time object.
   */
  ros::Time jointStateROSTimeStamp;

  /*!
   * The name of this control model.  This is useful when testing
   * the multi-threaded RTControlModel which has multiple ControlModel
   * objects within it.
   */
  std::string name;

  /*!
   * The constraint set that's used for gravity compensation.
   */
  // std::unique_ptr<ConstraintSet> gravityCompConstraintSet;

  /*!
   * The names of the actuated joints.
   */
  std::vector<std::string> actuatedJointNames;

  /*!
   * The names of the real joints.
   */
  std::vector<std::string> realJointNames;

  /*!
   * A pointer to the parameters.
   */
  controlit::utility::ControlItParameters * params;

  /*!
   * The rotor inertia values that are reflected onto the joint inertia
   * matrix.  This includes the affects of the gear ratios.
   */
  std::vector<double> reflectedRotorInertias;
};

} // namespace controlit

#endif  // __CONTROLIT_CORE_CONTROL_MODEL_HPP__
