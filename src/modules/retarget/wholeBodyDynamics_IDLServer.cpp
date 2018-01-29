// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include "wholeBodyDynamics_IDLServer.h"
#include <yarp/os/idl/WireTypes.h>

class wholeBodyDynamics_IDLServer_quit : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_resetSimpleLeggedOdometry : public yarp::os::Portable {
public:
  std::string initial_world_frame;
  std::string initial_fixed_link;
  bool _return;
  void init(const std::string& initial_world_frame, const std::string& initial_fixed_link);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_changeFixedLinkSimpleLeggedOdometry : public yarp::os::Portable {
public:
  std::string new_fixed_link;
  bool _return;
  void init(const std::string& new_fixed_link);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};


class wholeBodyDynamics_IDLServer_useFixedFrameAsKinematicSource : public yarp::os::Portable {
public:
  std::string fixedFrame;
  bool _return;
  void init(const std::string& fixedFrame);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_setUseOfJointVelocities : public yarp::os::Portable {
public:
  bool enable;
  bool _return;
  void init(const bool enable);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_setUseOfJointAccelerations : public yarp::os::Portable {
public:
  bool enable;
  bool _return;
  void init(const bool enable);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_getCurrentSettingsString : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};



bool wholeBodyDynamics_IDLServer_quit::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("quit",1,1)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_quit::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_quit::init() {
  _return = false;
}

bool wholeBodyDynamics_IDLServer_resetSimpleLeggedOdometry::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("resetSimpleLeggedOdometry",1,1)) return false;
  if (!writer.writeString(initial_world_frame)) return false;
  if (!writer.writeString(initial_fixed_link)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_resetSimpleLeggedOdometry::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_resetSimpleLeggedOdometry::init(const std::string& initial_world_frame, const std::string& initial_fixed_link) {
  _return = false;
  this->initial_world_frame = initial_world_frame;
  this->initial_fixed_link = initial_fixed_link;
}

bool wholeBodyDynamics_IDLServer_changeFixedLinkSimpleLeggedOdometry::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("changeFixedLinkSimpleLeggedOdometry",1,1)) return false;
  if (!writer.writeString(new_fixed_link)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_changeFixedLinkSimpleLeggedOdometry::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_changeFixedLinkSimpleLeggedOdometry::init(const std::string& new_fixed_link) {
  _return = false;
  this->new_fixed_link = new_fixed_link;
}

bool wholeBodyDynamics_IDLServer_useFixedFrameAsKinematicSource::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("useFixedFrameAsKinematicSource",1,1)) return false;
  if (!writer.writeString(fixedFrame)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_useFixedFrameAsKinematicSource::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_useFixedFrameAsKinematicSource::init(const std::string& fixedFrame) {
  _return = false;
  this->fixedFrame = fixedFrame;
}

bool wholeBodyDynamics_IDLServer_setUseOfJointVelocities::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setUseOfJointVelocities",1,1)) return false;
  if (!writer.writeBool(enable)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_setUseOfJointVelocities::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_setUseOfJointVelocities::init(const bool enable) {
  _return = false;
  this->enable = enable;
}

bool wholeBodyDynamics_IDLServer_setUseOfJointAccelerations::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setUseOfJointAccelerations",1,1)) return false;
  if (!writer.writeBool(enable)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_setUseOfJointAccelerations::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_setUseOfJointAccelerations::init(const bool enable) {
  _return = false;
  this->enable = enable;
}

bool wholeBodyDynamics_IDLServer_getCurrentSettingsString::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getCurrentSettingsString",1,1)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_getCurrentSettingsString::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_getCurrentSettingsString::init() {
  _return = "";
}

wholeBodyDynamics_IDLServer::wholeBodyDynamics_IDLServer() {
  yarp().setOwner(*this);
}

bool wholeBodyDynamics_IDLServer::quit() {
  bool _return = false;
  wholeBodyDynamics_IDLServer_quit helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_resetSimpleLeggedOdometry helper;
  helper.init(initial_world_frame,initial_fixed_link);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_changeFixedLinkSimpleLeggedOdometry helper;
  helper.init(new_fixed_link);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool wholeBodyDynamics_IDLServer::useFixedFrameAsKinematicSource(const std::string& fixedFrame) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_useFixedFrameAsKinematicSource helper;
  helper.init(fixedFrame);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::useFixedFrameAsKinematicSource(const std::string& fixedFrame)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::setUseOfJointVelocities(const bool enable) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_setUseOfJointVelocities helper;
  helper.init(enable);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::setUseOfJointVelocities(const bool enable)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::setUseOfJointAccelerations(const bool enable) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_setUseOfJointAccelerations helper;
  helper.init(enable);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::setUseOfJointAccelerations(const bool enable)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string wholeBodyDynamics_IDLServer::getCurrentSettingsString() {
  std::string _return = "";
  wholeBodyDynamics_IDLServer_getCurrentSettingsString helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string wholeBodyDynamics_IDLServer::getCurrentSettingsString()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool wholeBodyDynamics_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "quit") {
      bool _return;
      _return = quit();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "resetSimpleLeggedOdometry") {
      std::string initial_world_frame;
      std::string initial_fixed_link;
      if (!reader.readString(initial_world_frame)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(initial_fixed_link)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = resetSimpleLeggedOdometry(initial_world_frame,initial_fixed_link);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "changeFixedLinkSimpleLeggedOdometry") {
      std::string new_fixed_link;
      if (!reader.readString(new_fixed_link)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = changeFixedLinkSimpleLeggedOdometry(new_fixed_link);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    
    if (tag == "useFixedFrameAsKinematicSource") {
      std::string fixedFrame;
      if (!reader.readString(fixedFrame)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = useFixedFrameAsKinematicSource(fixedFrame);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setUseOfJointVelocities") {
      bool enable;
      if (!reader.readBool(enable)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setUseOfJointVelocities(enable);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setUseOfJointAccelerations") {
      bool enable;
      if (!reader.readBool(enable)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setUseOfJointAccelerations(enable);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getCurrentSettingsString") {
      std::string _return;
      _return = getCurrentSettingsString();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> wholeBodyDynamics_IDLServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("quit");
    helpString.push_back("resetSimpleLeggedOdometry");
    helpString.push_back("changeFixedLinkSimpleLeggedOdometry");
    helpString.push_back("useFixedFrameAsKinematicSource");
    helpString.push_back("setUseOfJointVelocities");
    helpString.push_back("setUseOfJointAccelerations");
    helpString.push_back("getCurrentSettingsString");
    helpString.push_back("help");
  }
  else {
    if (functionName=="quit") {
      helpString.push_back("bool quit() ");
      helpString.push_back("Quit the module. ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="resetSimpleLeggedOdometry") {
      helpString.push_back("bool resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link) ");
      helpString.push_back("Reset the odometry world to be (initially) a frame specified in the robot model, ");
      helpString.push_back("and specify a link that is assumed to be fixed in the odometry. ");
      helpString.push_back("@param initial_world_frame the frame of the robot model that is assume to be initially ");
      helpString.push_back("       coincident with the world/inertial frame. ");
      helpString.push_back("@param new_fixed_link the name of the link that should be initially fixed ");
      helpString.push_back("@return true/false on success/failure (typically if the frame/link names are wrong) ");
    }
    if (functionName=="changeFixedLinkSimpleLeggedOdometry") {
      helpString.push_back("bool changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link) ");
      helpString.push_back("Change the link that is considered fixed by the odometry. ");
      helpString.push_back("@param new_fixed_link the name of the new link that should be considered fixed ");
      helpString.push_back("@return true/false on success/failure (typically if the frame/link names are wrong) ");
    }
    
    if (functionName=="useFixedFrameAsKinematicSource") {
      helpString.push_back("bool useFixedFrameAsKinematicSource(const std::string& fixedFrame) ");
      helpString.push_back("Use a fixed frame (tipically root_link, l_sole or r_sole) ");
      helpString.push_back("as the source of kinematic information. The assumption ");
      helpString.push_back("is that the specified frame will remain fixed until ");
      helpString.push_back("the kinematic source is changing, and the gravity ");
      helpString.push_back("on this link is specified by the fixedFrameGravity (tipically ");
      helpString.push_back("set to (0,0,-9.81) . ");
    }
    if (functionName=="setUseOfJointVelocities") {
      helpString.push_back("bool setUseOfJointVelocities(const bool enable) ");
      helpString.push_back("Set if to use or not the joint velocities in estimation. ");
    }
    if (functionName=="setUseOfJointAccelerations") {
      helpString.push_back("bool setUseOfJointAccelerations(const bool enable) ");
      helpString.push_back("Set if to use or not the joint velocities in estimation. ");
    }
    if (functionName=="getCurrentSettingsString") {
      helpString.push_back("std::string getCurrentSettingsString() ");
      helpString.push_back("Get the current settings in the form of a string. ");
      helpString.push_back("@return the current settings as a human readable string. ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}


