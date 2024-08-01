// Auto-generated. Do not edit!

// (in-package cav_project.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let limo_state = require('./limo_state.js');

//-----------------------------------------------------------

class limo_state_matrix {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.limos = null;
    }
    else {
      if (initObj.hasOwnProperty('limos')) {
        this.limos = initObj.limos
      }
      else {
        this.limos = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type limo_state_matrix
    // Serialize message field [limos]
    // Serialize the length for message field [limos]
    bufferOffset = _serializer.uint32(obj.limos.length, buffer, bufferOffset);
    obj.limos.forEach((val) => {
      bufferOffset = limo_state.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type limo_state_matrix
    let len;
    let data = new limo_state_matrix(null);
    // Deserialize message field [limos]
    // Deserialize array length for message field [limos]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.limos = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.limos[i] = limo_state.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.limos.forEach((val) => {
      length += limo_state.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cav_project/limo_state_matrix';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '16904eb3a77a18c79afd9f6ff63a6e61';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    limo_state[] limos
    
    ================================================================================
    MSG: cav_project/limo_state
    string limoID
    float64 vel
    float64 d0
    float64 d1
    float64 v1
    float64 d2
    float64 v2
    float64 l2
    float64 d3
    float64 v3
    float64 l3
    float64 vd
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new limo_state_matrix(null);
    if (msg.limos !== undefined) {
      resolved.limos = new Array(msg.limos.length);
      for (let i = 0; i < resolved.limos.length; ++i) {
        resolved.limos[i] = limo_state.Resolve(msg.limos[i]);
      }
    }
    else {
      resolved.limos = []
    }

    return resolved;
    }
};

module.exports = limo_state_matrix;
