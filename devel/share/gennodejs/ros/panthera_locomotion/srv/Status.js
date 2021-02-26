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


//-----------------------------------------------------------

class StatusRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reconfig = null;
    }
    else {
      if (initObj.hasOwnProperty('reconfig')) {
        this.reconfig = initObj.reconfig
      }
      else {
        this.reconfig = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StatusRequest
    // Serialize message field [reconfig]
    bufferOffset = _serializer.bool(obj.reconfig, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StatusRequest
    let len;
    let data = new StatusRequest(null);
    // Deserialize message field [reconfig]
    data.reconfig = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'panthera_locomotion/StatusRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cdc35faafd94fe7d4c46256ad525db4d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool reconfig
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StatusRequest(null);
    if (msg.reconfig !== undefined) {
      resolved.reconfig = msg.reconfig;
    }
    else {
      resolved.reconfig = false
    }

    return resolved;
    }
};

class StatusResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
      this.speed = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = false;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StatusResponse
    // Serialize message field [status]
    bufferOffset = _serializer.bool(obj.status, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.int64(obj.speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StatusResponse
    let len;
    let data = new StatusResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'panthera_locomotion/StatusResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '89719d7b7249d89966a637c334f4a8dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool status
    int64 speed
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StatusResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = false
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: StatusRequest,
  Response: StatusResponse,
  md5sum() { return '6eac67a75eb7db125401139ae8ae1357'; },
  datatype() { return 'panthera_locomotion/Status'; }
};
