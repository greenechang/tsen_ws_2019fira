// Auto-generated. Do not edit!

// (in-package op3_gripper.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class GripperPosition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Left = null;
      this.Right = null;
    }
    else {
      if (initObj.hasOwnProperty('Left')) {
        this.Left = initObj.Left
      }
      else {
        this.Left = 0;
      }
      if (initObj.hasOwnProperty('Right')) {
        this.Right = initObj.Right
      }
      else {
        this.Right = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperPosition
    // Serialize message field [Left]
    bufferOffset = _serializer.int64(obj.Left, buffer, bufferOffset);
    // Serialize message field [Right]
    bufferOffset = _serializer.int64(obj.Right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperPosition
    let len;
    let data = new GripperPosition(null);
    // Deserialize message field [Left]
    data.Left = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [Right]
    data.Right = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'op3_gripper/GripperPosition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5b0e67f9a5c183da3139b32cb4f14d70';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 Left
    int64 Right
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperPosition(null);
    if (msg.Left !== undefined) {
      resolved.Left = msg.Left;
    }
    else {
      resolved.Left = 0
    }

    if (msg.Right !== undefined) {
      resolved.Right = msg.Right;
    }
    else {
      resolved.Right = 0
    }

    return resolved;
    }
};

module.exports = GripperPosition;
