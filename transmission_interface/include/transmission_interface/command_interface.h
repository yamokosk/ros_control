#ifndef TRANSMISSION_INTERFACE_COMMAND_INTERFACE_H
#define TRANSMISSION_INTERFACE_COMMAND_INTERFACE_H

#include <transmission_interface/state_interface.h>


namespace transmission_interface
{

/** \brief A handle used to read and command a single joint
 */
class CommandHandle : public StateHandle
{
public:
  CommandHandle() {};
  CommandHandle(const StateHandle& js, double* cmd)
    : StateHandle(js), command(cmd)
  {}

  void setCommand(double cmd) {*command = cmd;};

  double* command;
};


/** \brief Hardware interface to support commanding an array of joints
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named joints. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single double, they are not necessarily
 * effort commands. To specify a meaning to this command, see the derived
 * classes like \ref EffortJointInterface etc.
 *
 */
class CommandInterface : public hardware_interface::HardwareInterface
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
   * \param cmd A pointer to the storage for this joint's output command
   */
  void registerHandle(const StateHandle& js, double* cmd)
  {
    CommandHandle handle(js, cmd);
    HandleMap::iterator it = handle_map_.find(js.getName());
    if (it == handle_map_.end())
      handle_map_.insert(std::make_pair(js.getName(), handle));
    else
      it->second = handle;
  }

  /** \brief Get a \ref CommandHandle for accessing a joint's state and setting
   * its output command.
   *
   * When a \ref CommandHandle is acquired, this interface will claim the joint
   * as a resource.
   *
   * \param name The name of the joint
   *
   * \returns A \ref CommandHandle corresponding to the joint identified by \c name
   *
   */
  CommandHandle getCommandHandle(const std::string& name)
  {
    HandleMap::const_iterator it = handle_map_.find(name);

    if (it == handle_map_.end())
      throw hardware_interface::HardwareInterfaceException("Could not find joint [" + name + "] in CommandInterface");

    HardwareInterface::claim(name);
    return it->second;
  }

protected:
  typedef std::map<std::string, CommandHandle> HandleMap;
  HandleMap handle_map_;
};

/// \ref CommandInterface for commanding effort-based joints
class EffortInterface : public CommandInterface
{

};

/// \ref CommandInterface for commanding velocity-based joints
class VelocityInterface : public CommandInterface
{

};

/// \ref CommandInterface for commanding position-based joints
class PositionInterface : public CommandInterface
{

};


}

#endif
