// Auto-generated. Do not edit!

// (in-package roboclaw_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Stats {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.m1_enc_val = null;
      this.m2_enc_val = null;
      this.m1_enc_qpps = null;
      this.m2_enc_qpps = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('m1_enc_val')) {
        this.m1_enc_val = initObj.m1_enc_val
      }
      else {
        this.m1_enc_val = 0;
      }
      if (initObj.hasOwnProperty('m2_enc_val')) {
        this.m2_enc_val = initObj.m2_enc_val
      }
      else {
        this.m2_enc_val = 0;
      }
      if (initObj.hasOwnProperty('m1_enc_qpps')) {
        this.m1_enc_qpps = initObj.m1_enc_qpps
      }
      else {
        this.m1_enc_qpps = 0;
      }
      if (initObj.hasOwnProperty('m2_enc_qpps')) {
        this.m2_enc_qpps = initObj.m2_enc_qpps
      }
      else {
        this.m2_enc_qpps = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Stats
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [m1_enc_val]
    bufferOffset = _serializer.int32(obj.m1_enc_val, buffer, bufferOffset);
    // Serialize message field [m2_enc_val]
    bufferOffset = _serializer.int32(obj.m2_enc_val, buffer, bufferOffset);
    // Serialize message field [m1_enc_qpps]
    bufferOffset = _serializer.int32(obj.m1_enc_qpps, buffer, bufferOffset);
    // Serialize message field [m2_enc_qpps]
    bufferOffset = _serializer.int32(obj.m2_enc_qpps, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Stats
    let len;
    let data = new Stats(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [m1_enc_val]
    data.m1_enc_val = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [m2_enc_val]
    data.m2_enc_val = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [m1_enc_qpps]
    data.m1_enc_qpps = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [m2_enc_qpps]
    data.m2_enc_qpps = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'roboclaw_driver/Stats';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '210b25d19a34293aed84261a946d88a5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # std_msgs/Header 
    Header header
    
    # Motor 1 & 2 encoder values
    int32 m1_enc_val
    int32 m2_enc_val
    
    # Motor 1 & 2 speed values in QPPS (+/-)
    int32 m1_enc_qpps
    int32 m2_enc_qpps
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Stats(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.m1_enc_val !== undefined) {
      resolved.m1_enc_val = msg.m1_enc_val;
    }
    else {
      resolved.m1_enc_val = 0
    }

    if (msg.m2_enc_val !== undefined) {
      resolved.m2_enc_val = msg.m2_enc_val;
    }
    else {
      resolved.m2_enc_val = 0
    }

    if (msg.m1_enc_qpps !== undefined) {
      resolved.m1_enc_qpps = msg.m1_enc_qpps;
    }
    else {
      resolved.m1_enc_qpps = 0
    }

    if (msg.m2_enc_qpps !== undefined) {
      resolved.m2_enc_qpps = msg.m2_enc_qpps;
    }
    else {
      resolved.m2_enc_qpps = 0
    }

    return resolved;
    }
};

module.exports = Stats;
