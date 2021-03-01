// Auto-generated. Do not edit!

// (in-package roboclaw_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SpeedCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.m1_qpps = null;
      this.m2_qpps = null;
      this.accel = null;
      this.max_secs = null;
    }
    else {
      if (initObj.hasOwnProperty('m1_qpps')) {
        this.m1_qpps = initObj.m1_qpps
      }
      else {
        this.m1_qpps = 0;
      }
      if (initObj.hasOwnProperty('m2_qpps')) {
        this.m2_qpps = initObj.m2_qpps
      }
      else {
        this.m2_qpps = 0;
      }
      if (initObj.hasOwnProperty('accel')) {
        this.accel = initObj.accel
      }
      else {
        this.accel = 0;
      }
      if (initObj.hasOwnProperty('max_secs')) {
        this.max_secs = initObj.max_secs
      }
      else {
        this.max_secs = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SpeedCommand
    // Serialize message field [m1_qpps]
    bufferOffset = _serializer.int32(obj.m1_qpps, buffer, bufferOffset);
    // Serialize message field [m2_qpps]
    bufferOffset = _serializer.int32(obj.m2_qpps, buffer, bufferOffset);
    // Serialize message field [accel]
    bufferOffset = _serializer.uint32(obj.accel, buffer, bufferOffset);
    // Serialize message field [max_secs]
    bufferOffset = _serializer.uint32(obj.max_secs, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SpeedCommand
    let len;
    let data = new SpeedCommand(null);
    // Deserialize message field [m1_qpps]
    data.m1_qpps = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [m2_qpps]
    data.m2_qpps = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [accel]
    data.accel = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [max_secs]
    data.max_secs = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'roboclaw_driver/SpeedCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1217b4f680d01eb5008f99442806c05f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Motor speed in QPPS
    int32 m1_qpps
    int32 m2_qpps
    
    # Acceleration
    uint32 accel
    
    # Max seconds before automatically stopping
    uint32 max_secs
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SpeedCommand(null);
    if (msg.m1_qpps !== undefined) {
      resolved.m1_qpps = msg.m1_qpps;
    }
    else {
      resolved.m1_qpps = 0
    }

    if (msg.m2_qpps !== undefined) {
      resolved.m2_qpps = msg.m2_qpps;
    }
    else {
      resolved.m2_qpps = 0
    }

    if (msg.accel !== undefined) {
      resolved.accel = msg.accel;
    }
    else {
      resolved.accel = 0
    }

    if (msg.max_secs !== undefined) {
      resolved.max_secs = msg.max_secs;
    }
    else {
      resolved.max_secs = 0
    }

    return resolved;
    }
};

module.exports = SpeedCommand;
