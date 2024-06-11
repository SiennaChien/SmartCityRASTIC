// Auto-generated. Do not edit!

// (in-package cav_project.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let limo_info = require('./limo_info.js');

//-----------------------------------------------------------

class limo_info_array {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.limo_infos = null;
    }
    else {
      if (initObj.hasOwnProperty('limo_infos')) {
        this.limo_infos = initObj.limo_infos
      }
      else {
        this.limo_infos = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type limo_info_array
    // Serialize message field [limo_infos]
    // Serialize the length for message field [limo_infos]
    bufferOffset = _serializer.uint32(obj.limo_infos.length, buffer, bufferOffset);
    obj.limo_infos.forEach((val) => {
      bufferOffset = limo_info.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type limo_info_array
    let len;
    let data = new limo_info_array(null);
    // Deserialize message field [limo_infos]
    // Deserialize array length for message field [limo_infos]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.limo_infos = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.limo_infos[i] = limo_info.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.limo_infos.forEach((val) => {
      length += limo_info.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cav_project/limo_info_array';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cc8c33c215547bc4fe7561cb7d91633e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    cav_project/limo_info[] limo_infos
    
    ================================================================================
    MSG: cav_project/limo_info
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
    const resolved = new limo_info_array(null);
    if (msg.limo_infos !== undefined) {
      resolved.limo_infos = new Array(msg.limo_infos.length);
      for (let i = 0; i < resolved.limo_infos.length; ++i) {
        resolved.limo_infos[i] = limo_info.Resolve(msg.limo_infos[i]);
      }
    }
    else {
      resolved.limo_infos = []
    }

    return resolved;
    }
};

module.exports = limo_info_array;
