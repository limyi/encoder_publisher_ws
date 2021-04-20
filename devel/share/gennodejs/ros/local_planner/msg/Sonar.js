// Auto-generated. Do not edit!

// (in-package local_planner.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Sonar {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.back_l = null;
      this.back_r = null;
      this.left_b = null;
      this.left_m = null;
      this.left_f = null;
      this.front_l = null;
      this.front_r = null;
      this.right_f = null;
      this.right_m = null;
      this.right_b = null;
    }
    else {
      if (initObj.hasOwnProperty('back_l')) {
        this.back_l = initObj.back_l
      }
      else {
        this.back_l = 0.0;
      }
      if (initObj.hasOwnProperty('back_r')) {
        this.back_r = initObj.back_r
      }
      else {
        this.back_r = 0.0;
      }
      if (initObj.hasOwnProperty('left_b')) {
        this.left_b = initObj.left_b
      }
      else {
        this.left_b = 0.0;
      }
      if (initObj.hasOwnProperty('left_m')) {
        this.left_m = initObj.left_m
      }
      else {
        this.left_m = 0.0;
      }
      if (initObj.hasOwnProperty('left_f')) {
        this.left_f = initObj.left_f
      }
      else {
        this.left_f = 0.0;
      }
      if (initObj.hasOwnProperty('front_l')) {
        this.front_l = initObj.front_l
      }
      else {
        this.front_l = 0.0;
      }
      if (initObj.hasOwnProperty('front_r')) {
        this.front_r = initObj.front_r
      }
      else {
        this.front_r = 0.0;
      }
      if (initObj.hasOwnProperty('right_f')) {
        this.right_f = initObj.right_f
      }
      else {
        this.right_f = 0.0;
      }
      if (initObj.hasOwnProperty('right_m')) {
        this.right_m = initObj.right_m
      }
      else {
        this.right_m = 0.0;
      }
      if (initObj.hasOwnProperty('right_b')) {
        this.right_b = initObj.right_b
      }
      else {
        this.right_b = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Sonar
    // Serialize message field [back_l]
    bufferOffset = _serializer.float64(obj.back_l, buffer, bufferOffset);
    // Serialize message field [back_r]
    bufferOffset = _serializer.float64(obj.back_r, buffer, bufferOffset);
    // Serialize message field [left_b]
    bufferOffset = _serializer.float64(obj.left_b, buffer, bufferOffset);
    // Serialize message field [left_m]
    bufferOffset = _serializer.float64(obj.left_m, buffer, bufferOffset);
    // Serialize message field [left_f]
    bufferOffset = _serializer.float64(obj.left_f, buffer, bufferOffset);
    // Serialize message field [front_l]
    bufferOffset = _serializer.float64(obj.front_l, buffer, bufferOffset);
    // Serialize message field [front_r]
    bufferOffset = _serializer.float64(obj.front_r, buffer, bufferOffset);
    // Serialize message field [right_f]
    bufferOffset = _serializer.float64(obj.right_f, buffer, bufferOffset);
    // Serialize message field [right_m]
    bufferOffset = _serializer.float64(obj.right_m, buffer, bufferOffset);
    // Serialize message field [right_b]
    bufferOffset = _serializer.float64(obj.right_b, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Sonar
    let len;
    let data = new Sonar(null);
    // Deserialize message field [back_l]
    data.back_l = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [back_r]
    data.back_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_b]
    data.left_b = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_m]
    data.left_m = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_f]
    data.left_f = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [front_l]
    data.front_l = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [front_r]
    data.front_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_f]
    data.right_f = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_m]
    data.right_m = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_b]
    data.right_b = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 80;
  }

  static datatype() {
    // Returns string type for a message object
    return 'local_planner/Sonar';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f867769555dffd428a6a95bc8cdaae9c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 back_l
    float64 back_r
    
    float64 left_b
    float64 left_m
    float64 left_f
    
    float64 front_l
    float64 front_r
    
    float64 right_f
    float64 right_m
    float64 right_b
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Sonar(null);
    if (msg.back_l !== undefined) {
      resolved.back_l = msg.back_l;
    }
    else {
      resolved.back_l = 0.0
    }

    if (msg.back_r !== undefined) {
      resolved.back_r = msg.back_r;
    }
    else {
      resolved.back_r = 0.0
    }

    if (msg.left_b !== undefined) {
      resolved.left_b = msg.left_b;
    }
    else {
      resolved.left_b = 0.0
    }

    if (msg.left_m !== undefined) {
      resolved.left_m = msg.left_m;
    }
    else {
      resolved.left_m = 0.0
    }

    if (msg.left_f !== undefined) {
      resolved.left_f = msg.left_f;
    }
    else {
      resolved.left_f = 0.0
    }

    if (msg.front_l !== undefined) {
      resolved.front_l = msg.front_l;
    }
    else {
      resolved.front_l = 0.0
    }

    if (msg.front_r !== undefined) {
      resolved.front_r = msg.front_r;
    }
    else {
      resolved.front_r = 0.0
    }

    if (msg.right_f !== undefined) {
      resolved.right_f = msg.right_f;
    }
    else {
      resolved.right_f = 0.0
    }

    if (msg.right_m !== undefined) {
      resolved.right_m = msg.right_m;
    }
    else {
      resolved.right_m = 0.0
    }

    if (msg.right_b !== undefined) {
      resolved.right_b = msg.right_b;
    }
    else {
      resolved.right_b = 0.0
    }

    return resolved;
    }
};

module.exports = Sonar;
