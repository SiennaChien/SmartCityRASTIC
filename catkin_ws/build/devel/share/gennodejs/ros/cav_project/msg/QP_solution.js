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

class QP_solution {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.u = null;
    }
    else {
      if (initObj.hasOwnProperty('u')) {
        this.u = initObj.u
      }
      else {
        this.u = new std_msgs.msg.Float64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QP_solution
    // Serialize message field [u]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.u, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QP_solution
    let len;
    let data = new QP_solution(null);
    // Deserialize message field [u]
    data.u = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cav_project/QP_solution';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a9be02629cec3211ddab55e654369a74';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64 u
    
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
    const resolved = new QP_solution(null);
    if (msg.u !== undefined) {
      resolved.u = std_msgs.msg.Float64.Resolve(msg.u)
    }
    else {
      resolved.u = new std_msgs.msg.Float64()
    }

    return resolved;
    }
};

module.exports = QP_solution;
