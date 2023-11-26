// Auto-generated. Do not edit!

// (in-package beginner_tutorials.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MyCustomMessageType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isValid = null;
      this.restartsRemaining = null;
      this.encoderCounts = null;
      this.distanceToStart = null;
      this.distanceToEnd = null;
      this.taskDescription = null;
      this.distancesToOtherRobots = null;
    }
    else {
      if (initObj.hasOwnProperty('isValid')) {
        this.isValid = initObj.isValid
      }
      else {
        this.isValid = false;
      }
      if (initObj.hasOwnProperty('restartsRemaining')) {
        this.restartsRemaining = initObj.restartsRemaining
      }
      else {
        this.restartsRemaining = 0;
      }
      if (initObj.hasOwnProperty('encoderCounts')) {
        this.encoderCounts = initObj.encoderCounts
      }
      else {
        this.encoderCounts = 0;
      }
      if (initObj.hasOwnProperty('distanceToStart')) {
        this.distanceToStart = initObj.distanceToStart
      }
      else {
        this.distanceToStart = 0.0;
      }
      if (initObj.hasOwnProperty('distanceToEnd')) {
        this.distanceToEnd = initObj.distanceToEnd
      }
      else {
        this.distanceToEnd = 0.0;
      }
      if (initObj.hasOwnProperty('taskDescription')) {
        this.taskDescription = initObj.taskDescription
      }
      else {
        this.taskDescription = '';
      }
      if (initObj.hasOwnProperty('distancesToOtherRobots')) {
        this.distancesToOtherRobots = initObj.distancesToOtherRobots
      }
      else {
        this.distancesToOtherRobots = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MyCustomMessageType
    // Serialize message field [isValid]
    bufferOffset = _serializer.bool(obj.isValid, buffer, bufferOffset);
    // Serialize message field [restartsRemaining]
    bufferOffset = _serializer.uint32(obj.restartsRemaining, buffer, bufferOffset);
    // Serialize message field [encoderCounts]
    bufferOffset = _serializer.int32(obj.encoderCounts, buffer, bufferOffset);
    // Serialize message field [distanceToStart]
    bufferOffset = _serializer.float32(obj.distanceToStart, buffer, bufferOffset);
    // Serialize message field [distanceToEnd]
    bufferOffset = _serializer.float64(obj.distanceToEnd, buffer, bufferOffset);
    // Serialize message field [taskDescription]
    bufferOffset = _serializer.string(obj.taskDescription, buffer, bufferOffset);
    // Serialize message field [distancesToOtherRobots]
    bufferOffset = _arraySerializer.float64(obj.distancesToOtherRobots, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MyCustomMessageType
    let len;
    let data = new MyCustomMessageType(null);
    // Deserialize message field [isValid]
    data.isValid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [restartsRemaining]
    data.restartsRemaining = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [encoderCounts]
    data.encoderCounts = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [distanceToStart]
    data.distanceToStart = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distanceToEnd]
    data.distanceToEnd = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [taskDescription]
    data.taskDescription = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [distancesToOtherRobots]
    data.distancesToOtherRobots = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.taskDescription);
    length += 8 * object.distancesToOtherRobots.length;
    return length + 29;
  }

  static datatype() {
    // Returns string type for a message object
    return 'beginner_tutorials/MyCustomMessageType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '669304755248d0e6a47460f571ab5053';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isValid
    uint32 restartsRemaining
    int32 encoderCounts
    float32 distanceToStart
    float64 distanceToEnd
    string taskDescription
    float64[] distancesToOtherRobots
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MyCustomMessageType(null);
    if (msg.isValid !== undefined) {
      resolved.isValid = msg.isValid;
    }
    else {
      resolved.isValid = false
    }

    if (msg.restartsRemaining !== undefined) {
      resolved.restartsRemaining = msg.restartsRemaining;
    }
    else {
      resolved.restartsRemaining = 0
    }

    if (msg.encoderCounts !== undefined) {
      resolved.encoderCounts = msg.encoderCounts;
    }
    else {
      resolved.encoderCounts = 0
    }

    if (msg.distanceToStart !== undefined) {
      resolved.distanceToStart = msg.distanceToStart;
    }
    else {
      resolved.distanceToStart = 0.0
    }

    if (msg.distanceToEnd !== undefined) {
      resolved.distanceToEnd = msg.distanceToEnd;
    }
    else {
      resolved.distanceToEnd = 0.0
    }

    if (msg.taskDescription !== undefined) {
      resolved.taskDescription = msg.taskDescription;
    }
    else {
      resolved.taskDescription = ''
    }

    if (msg.distancesToOtherRobots !== undefined) {
      resolved.distancesToOtherRobots = msg.distancesToOtherRobots;
    }
    else {
      resolved.distancesToOtherRobots = []
    }

    return resolved;
    }
};

module.exports = MyCustomMessageType;
