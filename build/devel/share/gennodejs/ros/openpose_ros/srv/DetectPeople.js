// Auto-generated. Do not edit!

// (in-package openpose_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let PersonDetection = require('../msg/PersonDetection.js');

//-----------------------------------------------------------

class DetectPeopleRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectPeopleRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectPeopleRequest
    let len;
    let data = new DetectPeopleRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'openpose_ros/DetectPeopleRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DetectPeopleRequest(null);
    return resolved;
    }
};

class DetectPeopleResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.people_list = null;
    }
    else {
      if (initObj.hasOwnProperty('people_list')) {
        this.people_list = initObj.people_list
      }
      else {
        this.people_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectPeopleResponse
    // Serialize message field [people_list]
    // Serialize the length for message field [people_list]
    bufferOffset = _serializer.uint32(obj.people_list.length, buffer, bufferOffset);
    obj.people_list.forEach((val) => {
      bufferOffset = PersonDetection.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectPeopleResponse
    let len;
    let data = new DetectPeopleResponse(null);
    // Deserialize message field [people_list]
    // Deserialize array length for message field [people_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.people_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.people_list[i] = PersonDetection.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 532 * object.people_list.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'openpose_ros/DetectPeopleResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f9061c1888cb3379ce3b5278d009dde5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    PersonDetection[] people_list
    
    
    ================================================================================
    MSG: openpose_ros/PersonDetection
    BodyPartDetection Nose
    BodyPartDetection Neck
    BodyPartDetection RShoulder
    BodyPartDetection RElbow
    BodyPartDetection RWrist
    BodyPartDetection LShoulder
    BodyPartDetection LElbow
    BodyPartDetection LWrist
    BodyPartDetection RHip
    BodyPartDetection RKnee
    BodyPartDetection RAnkle
    BodyPartDetection LHip
    BodyPartDetection LKnee
    BodyPartDetection LAnkle
    BodyPartDetection REye
    BodyPartDetection LEye
    BodyPartDetection REar
    BodyPartDetection LEar
    BodyPartDetection Chest
    
    ================================================================================
    MSG: openpose_ros/BodyPartDetection
    geometry_msgs/Point pos
    float32 confidence
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
    const resolved = new DetectPeopleResponse(null);
    if (msg.people_list !== undefined) {
      resolved.people_list = new Array(msg.people_list.length);
      for (let i = 0; i < resolved.people_list.length; ++i) {
        resolved.people_list[i] = PersonDetection.Resolve(msg.people_list[i]);
      }
    }
    else {
      resolved.people_list = []
    }

    return resolved;
    }
};

module.exports = {
  Request: DetectPeopleRequest,
  Response: DetectPeopleResponse,
  md5sum() { return 'f9061c1888cb3379ce3b5278d009dde5'; },
  datatype() { return 'openpose_ros/DetectPeople'; }
};
