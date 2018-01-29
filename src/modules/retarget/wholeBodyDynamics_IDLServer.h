// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_wholeBodyDynamics_IDLServer
#define YARP_THRIFT_GENERATOR_wholeBodyDynamics_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class wholeBodyDynamics_IDLServer;


/**
 * wholeBodyDynamics_IDLServer
 * Interface.
 */
class wholeBodyDynamics_IDLServer : public yarp::os::Wire {
public:
  wholeBodyDynamics_IDLServer();
  
  /**
   * Quit the module.
   * @return true/false on success/failure
   */
  virtual bool quit();
  /**
   * Reset the odometry world to be (initially) a frame specified in the robot model,
   * and specify a link that is assumed to be fixed in the odometry.
   * @param initial_world_frame the frame of the robot model that is assume to be initially
   *        coincident with the world/inertial frame.
   * @param new_fixed_link the name of the link that should be initially fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  virtual bool resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link);
  /**
   * Change the link that is considered fixed by the odometry.
   * @param new_fixed_link the name of the new link that should be considered fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  virtual bool changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link);
  /**
   * Use a fixed frame (tipically root_link, l_sole or r_sole)
   * as the source of kinematic information. The assumption
   * is that the specified frame will remain fixed until
   * the kinematic source is changing, and the gravity
   * on this link is specified by the fixedFrameGravity (tipically
   * set to (0,0,-9.81) .
   */
  virtual bool useFixedFrameAsKinematicSource(const std::string& fixedFrame);
  /**
   * Set if to use or not the joint velocities in estimation.
   */
  virtual bool setUseOfJointVelocities(const bool enable);
  /**
   * Set if to use or not the joint velocities in estimation.
   */
  virtual bool setUseOfJointAccelerations(const bool enable);
  /**
   * Get the current settings in the form of a string.
   * @return the current settings as a human readable string.
   */
  virtual std::string getCurrentSettingsString();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
