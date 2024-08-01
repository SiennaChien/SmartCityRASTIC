// Auto-generated. Do not edit!

// (in-package cav_project_2.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class limo_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.limoID = null;
      this.vel = null;
      this.d0 = null;
      this.d1 = null;
      this.v1 = null;
      this.d2 = null;
      this.v2 = null;
      this.vd = null;
    }
    else {
      if (initObj.hasOwnProperty('limoID')) {
        this.limoID = initObj.limoID
      }
      else {
        this.limoID = '';
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = 0.0;
      }
      if (initObj.hasOwnProperty('d0')) {
        this.d0 = initObj.d0
      }
      else {
        this.d0 = 0.0;
      }
      if (initObj.hasOwnProperty('d1')) {
        this.d1 = initObj.d1
      }
      else {
        this.d1 = 0.0;
      }
      if (initObj.hasOwnProperty('v1')) {
        this.v1 = initObj.v1
      }
      else {
        this.v1 = 0.0;
      }
      if (initObj.hasOwnProperty('d2')) {
        this.d2 = initObj.d2
      }
      else {
        this.d2 = 0.0;
      }
      if (initObj.hasOwnProperty('v2')) {
        this.v2 = initObj.v2
      }
      else {
        this.v2 = 0.0;
      }
      if (initObj.hasOwnProperty('vd')) {
        this.vd = initObj.vd
      }
      else {
        this.vd = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type limo_state
    // Serialize message field [limoID]
    bufferOffset = _serializer.string(obj.limoID, buffer, bufferOffset);
    // Serialize message field [vel]
    bufferOffset = _serializer.float64(obj.vel, buffer, bufferOffset);
    // Serialize message field [d0]
    bufferOffset = _serializer.float64(obj.d0, buffer, bufferOffset);
    // Serialize message field [d1]
    bufferOffset = _serializer.float64(obj.d1, buffer, bufferOffset);
    // Serialize message field [v1]
    bufferOffset = _serializer.float64(obj.v1, buffer, bufferOffset);
    // Serialize message field [d2]
    bufferOffset = _serializer.float64(obj.d2, buffer, bufferOffset);
    // Serialize message field [v2]
    bufferOffset = _serializer.float64(obj.v2, buffer, bufferOffset);
    // Serialize message field [vd]
    bufferOffset = _serializer.float64(obj.vd, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type limo_state
    let len;
    let data = new limo_state(null);
    // Deserialize message field [limoID]
    data.limoID = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [vel]
    data.vel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d0]
    data.d0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d1]
    data.d1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v1]
    data.v1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d2]
    data.d2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [v2]
    data.v2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vd]
    data.vd = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.limoID);
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cav_project_2/limo_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a1fba4736fa7b1723499e2717d3f0429';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string limoID
    float64 vel
    float64 d0
    float64 d1
    float64 v1
    float64 d2
    float64 v2
    float64 vd
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new limo_state(null);
    if (msg.limoID !== undefined) {
      resolved.limoID = msg.limoID;
    }
    else {
      resolved.limoID = ''
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = 0.0
    }

    if (msg.d0 !== undefined) {
      resolved.d0 = msg.d0;
    }
    else {
      resolved.d0 = 0.0
    }

    if (msg.d1 !== undefined) {
      resolved.d1 = msg.d1;
    }
    else {
      resolved.d1 = 0.0
    }

    if (msg.v1 !== undefined) {
      resolved.v1 = msg.v1;
    }
    else {
      resolved.v1 = 0.0
    }

    if (msg.d2 !== undefined) {
      resolved.d2 = msg.d2;
    }
    else {
      resolved.d2 = 0.0
    }

    if (msg.v2 !== undefined) {
      resolved.v2 = msg.v2;
    }
    else {
      resolved.v2 = 0.0
    }

    if (msg.vd !== undefined) {
      resolved.vd = msg.vd;
    }
    else {
      resolved.vd = 0.0
    }

    return resolved;
    }
};

module.exports = limo_state;
