#ifndef TRANSMISSION_INTERFACE_JOINT_STATE_INTERFACE_H
#define TRANSMISSION_INTERFACE_JOINT_STATE_INTERFACE_H

#include <hardware_interface/hardware_interface.h>
#include <string>
#include <map>
#include <vector>
#include <utility>  // for std::make_pair

namespace transmission_interface
{

/// A handle used to read the state of a single joint
class StateHandle
{
public:
  StateHandle() {};
  StateHandle(const std::string& name, double* pos, double* vel, double* eff)
    : position(pos), velocity(vel), effort(eff), name_(name)
  {}

  std::string getName() const {return name_;}
  double getPosition() const {return *position;}
  double getVelocity() const {return *velocity;}
  double getEffort()   const {return *effort;}

  double* position;
  double* velocity;
  double* effort;

private:
  std::string name_;
};


/** \brief Hardware interface to support reading the state of an array of joints
 * 
 * This \ref HardwareInterface supports reading the state of an array of named
 * joints, each of which has some position, velocity, and effort (force or
 * torque).
 *
 */
class StateInterface: public hardware_interface::HardwareInterface
{
public:
  /// Get the vector of joint names registered to this interface.
  std::vector<std::string> getNames() const
  {
    std::vector<std::string> out;
    out.reserve(handle_map_.size());
    for( HandleMap::const_iterator it = handle_map_.begin(); it != handle_map_.end(); ++it)
    {
      out.push_back(it->first);
    }
    return out;
  }

  /** \brief Register a new joint with this interface.
   *
   * \param name The name of the new joint
   * \param pos A pointer to the storage for this joint's position 
   * \param vel A pointer to the storage for this joint's velocity
   * \param eff A pointer to the storage for this joint's effort (force or torque)
   *
   */
  void registerHandle(const std::string& name, double* pos, double* vel, double* eff)
  {
    StateHandle handle(name, pos, vel, eff);
    HandleMap::iterator it = handle_map_.find(name);
    if (it == handle_map_.end())
      handle_map_.insert(std::make_pair(name, handle));
    else
      it->second = handle;
  }

  /** \brief Get a \ref StateHandle for accessing a joint's state
   *
   * \param name The name of the joint
   *
   */
  StateHandle getStateHandle(const std::string& name) const
  {
    HandleMap::const_iterator it = handle_map_.find(name);

    if (it == handle_map_.end())
      throw hardware_interface::HardwareInterfaceException("Could not find joint [" + name + "] in StateInterface");

    return it->second;
  }

private:
  typedef std::map<std::string, StateHandle> HandleMap;
  HandleMap handle_map_;

};

}

#endif
