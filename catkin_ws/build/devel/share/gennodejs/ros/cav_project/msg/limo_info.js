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
      this.ID = null;
      this.x = null;
      this.y = null;
      this.vel = null;
      this.path = null;
      this.d1 = null;
      this.d2 = null;
      this.origin_dist = null;
    }
    else {
      if (initObj.hasOwnProperty('ID')) {
        this.ID = initObj.ID
      }
      else {
        this.ID = new std_msgs.msg.Int32();
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('d1')) {
        this.d1 = initObj.d1
      }
      else {
        this.d1 = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('d2')) {
        this.d2 = initObj.d2
      }
      else {
        this.d2 = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('origin_dist')) {
        this.origin_dist = initObj.origin_dist
      }
      else {
        this.origin_dist = new std_msgs.msg.Float64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type limo_info
    // Serialize message field [ID]
    bufferOffset = std_msgs.msg.Int32.serialize(obj.ID, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.y, buffer, bufferOffset);
    // Serialize message field [vel]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.vel, buffer, bufferOffset);
    // Serialize message field [path]
    bufferOffset = std_msgs.msg.String.serialize(obj.path, buffer, bufferOffset);
    // Serialize message field [d1]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.d1, buffer, bufferOffset);
    // Serialize message field [d2]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.d2, buffer, bufferOffset);
    // Serialize message field [origin_dist]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.origin_dist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type limo_info
    let len;
    let data = new limo_info(null);
    // Deserialize message field [ID]
    data.ID = std_msgs.msg.Int32.deserialize(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [vel]
    data.vel = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [path]
    data.path = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [d1]
    data.d1 = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [d2]
    data.d2 = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [origin_dist]
    data.origin_dist = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.path);
    return length + 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cav_project/limo_info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '120a9aa2d6a27cbacb07fac971279075';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Int32 ID
    std_msgs/Float64 x
    std_msgs/Float64 y
    std_msgs/Float64 vel
    std_msgs/String path
    std_msgs/Float64 d1
    std_msgs/Float64 d2
    std_msgs/Float64 origin_dist
    
    ================================================================================
    MSG: std_msgs/Int32
    int32 data
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new limo_info(null);
    if (msg.ID !== undefined) {
      resolved.ID = std_msgs.msg.Int32.Resolve(msg.ID)
    }
    else {
      resolved.ID = new std_msgs.msg.Int32()
    }

    if (msg.x !== undefined) {
      resolved.x = std_msgs.msg.Float64.Resolve(msg.x)
    }
    else {
      resolved.x = new std_msgs.msg.Float64()
    }

    if (msg.y !== undefined) {
      resolved.y = std_msgs.msg.Float64.Resolve(msg.y)
    }
    else {
      resolved.y = new std_msgs.msg.Float64()
    }

    if (msg.vel !== undefined) {
      resolved.vel = std_msgs.msg.Float64.Resolve(msg.vel)
    }
    else {
      resolved.vel = new std_msgs.msg.Float64()
    }

    if (msg.path !== undefined) {
      resolved.path = std_msgs.msg.String.Resolve(msg.path)
    }
    else {
      resolved.path = new std_msgs.msg.String()
    }

    if (msg.d1 !== undefined) {
      resolved.d1 = std_msgs.msg.Float64.Resolve(msg.d1)
    }
    else {
      resolved.d1 = new std_msgs.msg.Float64()
    }

    if (msg.d2 !== undefined) {
      resolved.d2 = std_msgs.msg.Float64.Resolve(msg.d2)
    }
    else {
      resolved.d2 = new std_msgs.msg.Float64()
    }

    if (msg.origin_dist !== undefined) {
      resolved.origin_dist = std_msgs.msg.Float64.Resolve(msg.origin_dist)
    }
    else {
      resolved.origin_dist = new std_msgs.msg.Float64()
    }

    return resolved;
    }
};

module.exports = limo_info;
