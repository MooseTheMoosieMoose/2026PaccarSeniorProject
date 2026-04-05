// Auto-generated. Do not edit!

// (in-package coop_per_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ObjectDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lat_y = null;
      this.long_x = null;
      this.altitude_z = null;
      this.depth = null;
      this.width = null;
      this.height = null;
      this.rotation = null;
      this.class_name = null;
      this.confidence = null;
    }
    else {
      if (initObj.hasOwnProperty('lat_y')) {
        this.lat_y = initObj.lat_y
      }
      else {
        this.lat_y = 0.0;
      }
      if (initObj.hasOwnProperty('long_x')) {
        this.long_x = initObj.long_x
      }
      else {
        this.long_x = 0.0;
      }
      if (initObj.hasOwnProperty('altitude_z')) {
        this.altitude_z = initObj.altitude_z
      }
      else {
        this.altitude_z = 0.0;
      }
      if (initObj.hasOwnProperty('depth')) {
        this.depth = initObj.depth
      }
      else {
        this.depth = 0.0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0.0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0.0;
      }
      if (initObj.hasOwnProperty('rotation')) {
        this.rotation = initObj.rotation
      }
      else {
        this.rotation = 0.0;
      }
      if (initObj.hasOwnProperty('class_name')) {
        this.class_name = initObj.class_name
      }
      else {
        this.class_name = '';
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectDetection
    // Serialize message field [lat_y]
    bufferOffset = _serializer.float64(obj.lat_y, buffer, bufferOffset);
    // Serialize message field [long_x]
    bufferOffset = _serializer.float64(obj.long_x, buffer, bufferOffset);
    // Serialize message field [altitude_z]
    bufferOffset = _serializer.float64(obj.altitude_z, buffer, bufferOffset);
    // Serialize message field [depth]
    bufferOffset = _serializer.float32(obj.depth, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.float32(obj.width, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.float32(obj.height, buffer, bufferOffset);
    // Serialize message field [rotation]
    bufferOffset = _serializer.float32(obj.rotation, buffer, bufferOffset);
    // Serialize message field [class_name]
    bufferOffset = _serializer.string(obj.class_name, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectDetection
    let len;
    let data = new ObjectDetection(null);
    // Deserialize message field [lat_y]
    data.lat_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [long_x]
    data.long_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [altitude_z]
    data.altitude_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [depth]
    data.depth = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rotation]
    data.rotation = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [class_name]
    data.class_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.class_name);
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'coop_per_msgs/ObjectDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2b10fb12cf3dbf0187359d2e5baae524';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Position in Global Space
    #Note that 32 bit floats have a worst case accuracy around ~1.7 meters
    #While a 64 bit float has a worset case accuracy around ~3.16 nanometers
    float64 lat_y
    float64 long_x
    float64 altitude_z
    
    #The objects dimensions in meters
    float32 depth
    float32 width
    float32 height
    
    #The objects rotation along its up/down (Z) axis
    float32 rotation
    
    #The class name and its confidence score
    string class_name
    float32 confidence
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjectDetection(null);
    if (msg.lat_y !== undefined) {
      resolved.lat_y = msg.lat_y;
    }
    else {
      resolved.lat_y = 0.0
    }

    if (msg.long_x !== undefined) {
      resolved.long_x = msg.long_x;
    }
    else {
      resolved.long_x = 0.0
    }

    if (msg.altitude_z !== undefined) {
      resolved.altitude_z = msg.altitude_z;
    }
    else {
      resolved.altitude_z = 0.0
    }

    if (msg.depth !== undefined) {
      resolved.depth = msg.depth;
    }
    else {
      resolved.depth = 0.0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0.0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0.0
    }

    if (msg.rotation !== undefined) {
      resolved.rotation = msg.rotation;
    }
    else {
      resolved.rotation = 0.0
    }

    if (msg.class_name !== undefined) {
      resolved.class_name = msg.class_name;
    }
    else {
      resolved.class_name = ''
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    return resolved;
    }
};

module.exports = ObjectDetection;
