// Auto-generated. Do not edit!

// (in-package panthera_locomotion.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ICRsearchRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.received_angle = null;
      this.turn_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('received_angle')) {
        this.received_angle = initObj.received_angle
      }
      else {
        this.received_angle = false;
      }
      if (initObj.hasOwnProperty('turn_angle')) {
        this.turn_angle = initObj.turn_angle
      }
      else {
        this.turn_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ICRsearchRequest
    // Serialize message field [received_angle]
    bufferOffset = _serializer.bool(obj.received_angle, buffer, bufferOffset);
    // Serialize message field [turn_angle]
    bufferOffset = _serializer.float64(obj.turn_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ICRsearchRequest
    let len;
    let data = new ICRsearchRequest(null);
    // Deserialize message field [received_angle]
    data.received_angle = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [turn_angle]
    data.turn_angle = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'panthera_locomotion/ICRsearchRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9b0f9d7b5541dc5d4d54f360e0b450a0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool received_angle
    float64 turn_angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ICRsearchRequest(null);
    if (msg.received_angle !== undefined) {
      resolved.received_angle = msg.received_angle;
    }
    else {
      resolved.received_angle = false
    }

    if (msg.turn_angle !== undefined) {
      resolved.turn_angle = msg.turn_angle;
    }
    else {
      resolved.turn_angle = 0.0
    }

    return resolved;
    }
};

class ICRsearchResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.feasibility = null;
      this.wheel_angles = null;
      this.wheel_speeds = null;
    }
    else {
      if (initObj.hasOwnProperty('feasibility')) {
        this.feasibility = initObj.feasibility
      }
      else {
        this.feasibility = false;
      }
      if (initObj.hasOwnProperty('wheel_angles')) {
        this.wheel_angles = initObj.wheel_angles
      }
      else {
        this.wheel_angles = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('wheel_speeds')) {
        this.wheel_speeds = initObj.wheel_speeds
      }
      else {
        this.wheel_speeds = new geometry_msgs.msg.Twist();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ICRsearchResponse
    // Serialize message field [feasibility]
    bufferOffset = _serializer.bool(obj.feasibility, buffer, bufferOffset);
    // Serialize message field [wheel_angles]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.wheel_angles, buffer, bufferOffset);
    // Serialize message field [wheel_speeds]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.wheel_speeds, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ICRsearchResponse
    let len;
    let data = new ICRsearchResponse(null);
    // Deserialize message field [feasibility]
    data.feasibility = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [wheel_angles]
    data.wheel_angles = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [wheel_speeds]
    data.wheel_speeds = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 97;
  }

  static datatype() {
    // Returns string type for a service object
    return 'panthera_locomotion/ICRsearchResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b155cda63f3fb3bd2896ba1fe0ba2d6b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool feasibility
    geometry_msgs/Twist wheel_angles
    geometry_msgs/Twist wheel_speeds
    
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ICRsearchResponse(null);
    if (msg.feasibility !== undefined) {
      resolved.feasibility = msg.feasibility;
    }
    else {
      resolved.feasibility = false
    }

    if (msg.wheel_angles !== undefined) {
      resolved.wheel_angles = geometry_msgs.msg.Twist.Resolve(msg.wheel_angles)
    }
    else {
      resolved.wheel_angles = new geometry_msgs.msg.Twist()
    }

    if (msg.wheel_speeds !== undefined) {
      resolved.wheel_speeds = geometry_msgs.msg.Twist.Resolve(msg.wheel_speeds)
    }
    else {
      resolved.wheel_speeds = new geometry_msgs.msg.Twist()
    }

    return resolved;
    }
};

module.exports = {
  Request: ICRsearchRequest,
  Response: ICRsearchResponse,
  md5sum() { return 'bcaf6636f1df5330595534eeebecd7f8'; },
  datatype() { return 'panthera_locomotion/ICRsearch'; }
};
