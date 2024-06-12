// Auto-generated. Do not edit!

// (in-package cav_project.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class limo_info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vel = null;
    }
    else {
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = new std_msgs.msg.Float64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type limo_info
    // Serialize message field [vel]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type limo_info
    let len;
    let data = new limo_info(null);
    // Deserialize message field [vel]
    data.vel = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cav_project/limo_info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b2990a44cec4fde7af6f58f2727169f0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64 vel
    
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new limo_info(null);
    if (msg.vel !== undefined) {
      resolved.vel = std_msgs.msg.Float64.Resolve(msg.vel)
    }
    else {
      resolved.vel = new std_msgs.msg.Float64()
    }

    return resolved;
    }
};

module.exports = limo_info;
