// Auto-generated. Do not edit!

// (in-package cav_project.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ControlInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.steering_angle = null;
      this.desired_velocity = null;
      this.control_input = null;
    }
    else {
      if (initObj.hasOwnProperty('steering_angle')) {
        this.steering_angle = initObj.steering_angle
      }
      else {
        this.steering_angle = 0.0;
      }
      if (initObj.hasOwnProperty('desired_velocity')) {
        this.desired_velocity = initObj.desired_velocity
      }
      else {
        this.desired_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('control_input')) {
        this.control_input = initObj.control_input
      }
      else {
        this.control_input = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlInfo
    // Serialize message field [steering_angle]
    bufferOffset = _serializer.float64(obj.steering_angle, buffer, bufferOffset);
    // Serialize message field [desired_velocity]
    bufferOffset = _serializer.float64(obj.desired_velocity, buffer, bufferOffset);
    // Serialize message field [control_input]
    bufferOffset = _serializer.float64(obj.control_input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlInfo
    let len;
    let data = new ControlInfo(null);
    // Deserialize message field [steering_angle]
    data.steering_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [desired_velocity]
    data.desired_velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [control_input]
    data.control_input = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cav_project/ControlInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a24bd3f8a8eca54ba890a9158619a2e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 steering_angle
    float64 desired_velocity
    float64 control_input
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlInfo(null);
    if (msg.steering_angle !== undefined) {
      resolved.steering_angle = msg.steering_angle;
    }
    else {
      resolved.steering_angle = 0.0
    }

    if (msg.desired_velocity !== undefined) {
      resolved.desired_velocity = msg.desired_velocity;
    }
    else {
      resolved.desired_velocity = 0.0
    }

    if (msg.control_input !== undefined) {
      resolved.control_input = msg.control_input;
    }
    else {
      resolved.control_input = 0.0
    }

    return resolved;
    }
};

module.exports = ControlInfo;
