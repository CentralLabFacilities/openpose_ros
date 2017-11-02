// Auto-generated. Do not edit!

// (in-package openpose_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BodyPartDetection = require('./BodyPartDetection.js');

//-----------------------------------------------------------

class PersonDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Nose = null;
      this.Neck = null;
      this.RShoulder = null;
      this.RElbow = null;
      this.RWrist = null;
      this.LShoulder = null;
      this.LElbow = null;
      this.LWrist = null;
      this.RHip = null;
      this.RKnee = null;
      this.RAnkle = null;
      this.LHip = null;
      this.LKnee = null;
      this.LAnkle = null;
      this.REye = null;
      this.LEye = null;
      this.REar = null;
      this.LEar = null;
      this.Chest = null;
    }
    else {
      if (initObj.hasOwnProperty('Nose')) {
        this.Nose = initObj.Nose
      }
      else {
        this.Nose = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('Neck')) {
        this.Neck = initObj.Neck
      }
      else {
        this.Neck = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('RShoulder')) {
        this.RShoulder = initObj.RShoulder
      }
      else {
        this.RShoulder = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('RElbow')) {
        this.RElbow = initObj.RElbow
      }
      else {
        this.RElbow = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('RWrist')) {
        this.RWrist = initObj.RWrist
      }
      else {
        this.RWrist = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('LShoulder')) {
        this.LShoulder = initObj.LShoulder
      }
      else {
        this.LShoulder = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('LElbow')) {
        this.LElbow = initObj.LElbow
      }
      else {
        this.LElbow = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('LWrist')) {
        this.LWrist = initObj.LWrist
      }
      else {
        this.LWrist = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('RHip')) {
        this.RHip = initObj.RHip
      }
      else {
        this.RHip = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('RKnee')) {
        this.RKnee = initObj.RKnee
      }
      else {
        this.RKnee = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('RAnkle')) {
        this.RAnkle = initObj.RAnkle
      }
      else {
        this.RAnkle = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('LHip')) {
        this.LHip = initObj.LHip
      }
      else {
        this.LHip = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('LKnee')) {
        this.LKnee = initObj.LKnee
      }
      else {
        this.LKnee = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('LAnkle')) {
        this.LAnkle = initObj.LAnkle
      }
      else {
        this.LAnkle = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('REye')) {
        this.REye = initObj.REye
      }
      else {
        this.REye = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('LEye')) {
        this.LEye = initObj.LEye
      }
      else {
        this.LEye = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('REar')) {
        this.REar = initObj.REar
      }
      else {
        this.REar = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('LEar')) {
        this.LEar = initObj.LEar
      }
      else {
        this.LEar = new BodyPartDetection();
      }
      if (initObj.hasOwnProperty('Chest')) {
        this.Chest = initObj.Chest
      }
      else {
        this.Chest = new BodyPartDetection();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PersonDetection
    // Serialize message field [Nose]
    bufferOffset = BodyPartDetection.serialize(obj.Nose, buffer, bufferOffset);
    // Serialize message field [Neck]
    bufferOffset = BodyPartDetection.serialize(obj.Neck, buffer, bufferOffset);
    // Serialize message field [RShoulder]
    bufferOffset = BodyPartDetection.serialize(obj.RShoulder, buffer, bufferOffset);
    // Serialize message field [RElbow]
    bufferOffset = BodyPartDetection.serialize(obj.RElbow, buffer, bufferOffset);
    // Serialize message field [RWrist]
    bufferOffset = BodyPartDetection.serialize(obj.RWrist, buffer, bufferOffset);
    // Serialize message field [LShoulder]
    bufferOffset = BodyPartDetection.serialize(obj.LShoulder, buffer, bufferOffset);
    // Serialize message field [LElbow]
    bufferOffset = BodyPartDetection.serialize(obj.LElbow, buffer, bufferOffset);
    // Serialize message field [LWrist]
    bufferOffset = BodyPartDetection.serialize(obj.LWrist, buffer, bufferOffset);
    // Serialize message field [RHip]
    bufferOffset = BodyPartDetection.serialize(obj.RHip, buffer, bufferOffset);
    // Serialize message field [RKnee]
    bufferOffset = BodyPartDetection.serialize(obj.RKnee, buffer, bufferOffset);
    // Serialize message field [RAnkle]
    bufferOffset = BodyPartDetection.serialize(obj.RAnkle, buffer, bufferOffset);
    // Serialize message field [LHip]
    bufferOffset = BodyPartDetection.serialize(obj.LHip, buffer, bufferOffset);
    // Serialize message field [LKnee]
    bufferOffset = BodyPartDetection.serialize(obj.LKnee, buffer, bufferOffset);
    // Serialize message field [LAnkle]
    bufferOffset = BodyPartDetection.serialize(obj.LAnkle, buffer, bufferOffset);
    // Serialize message field [REye]
    bufferOffset = BodyPartDetection.serialize(obj.REye, buffer, bufferOffset);
    // Serialize message field [LEye]
    bufferOffset = BodyPartDetection.serialize(obj.LEye, buffer, bufferOffset);
    // Serialize message field [REar]
    bufferOffset = BodyPartDetection.serialize(obj.REar, buffer, bufferOffset);
    // Serialize message field [LEar]
    bufferOffset = BodyPartDetection.serialize(obj.LEar, buffer, bufferOffset);
    // Serialize message field [Chest]
    bufferOffset = BodyPartDetection.serialize(obj.Chest, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PersonDetection
    let len;
    let data = new PersonDetection(null);
    // Deserialize message field [Nose]
    data.Nose = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [Neck]
    data.Neck = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [RShoulder]
    data.RShoulder = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [RElbow]
    data.RElbow = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [RWrist]
    data.RWrist = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [LShoulder]
    data.LShoulder = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [LElbow]
    data.LElbow = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [LWrist]
    data.LWrist = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [RHip]
    data.RHip = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [RKnee]
    data.RKnee = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [RAnkle]
    data.RAnkle = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [LHip]
    data.LHip = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [LKnee]
    data.LKnee = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [LAnkle]
    data.LAnkle = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [REye]
    data.REye = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [LEye]
    data.LEye = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [REar]
    data.REar = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [LEar]
    data.LEar = BodyPartDetection.deserialize(buffer, bufferOffset);
    // Deserialize message field [Chest]
    data.Chest = BodyPartDetection.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 532;
  }

  static datatype() {
    // Returns string type for a message object
    return 'openpose_ros/PersonDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a992f141e745a3f47d598e0bc953d79';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new PersonDetection(null);
    if (msg.Nose !== undefined) {
      resolved.Nose = BodyPartDetection.Resolve(msg.Nose)
    }
    else {
      resolved.Nose = new BodyPartDetection()
    }

    if (msg.Neck !== undefined) {
      resolved.Neck = BodyPartDetection.Resolve(msg.Neck)
    }
    else {
      resolved.Neck = new BodyPartDetection()
    }

    if (msg.RShoulder !== undefined) {
      resolved.RShoulder = BodyPartDetection.Resolve(msg.RShoulder)
    }
    else {
      resolved.RShoulder = new BodyPartDetection()
    }

    if (msg.RElbow !== undefined) {
      resolved.RElbow = BodyPartDetection.Resolve(msg.RElbow)
    }
    else {
      resolved.RElbow = new BodyPartDetection()
    }

    if (msg.RWrist !== undefined) {
      resolved.RWrist = BodyPartDetection.Resolve(msg.RWrist)
    }
    else {
      resolved.RWrist = new BodyPartDetection()
    }

    if (msg.LShoulder !== undefined) {
      resolved.LShoulder = BodyPartDetection.Resolve(msg.LShoulder)
    }
    else {
      resolved.LShoulder = new BodyPartDetection()
    }

    if (msg.LElbow !== undefined) {
      resolved.LElbow = BodyPartDetection.Resolve(msg.LElbow)
    }
    else {
      resolved.LElbow = new BodyPartDetection()
    }

    if (msg.LWrist !== undefined) {
      resolved.LWrist = BodyPartDetection.Resolve(msg.LWrist)
    }
    else {
      resolved.LWrist = new BodyPartDetection()
    }

    if (msg.RHip !== undefined) {
      resolved.RHip = BodyPartDetection.Resolve(msg.RHip)
    }
    else {
      resolved.RHip = new BodyPartDetection()
    }

    if (msg.RKnee !== undefined) {
      resolved.RKnee = BodyPartDetection.Resolve(msg.RKnee)
    }
    else {
      resolved.RKnee = new BodyPartDetection()
    }

    if (msg.RAnkle !== undefined) {
      resolved.RAnkle = BodyPartDetection.Resolve(msg.RAnkle)
    }
    else {
      resolved.RAnkle = new BodyPartDetection()
    }

    if (msg.LHip !== undefined) {
      resolved.LHip = BodyPartDetection.Resolve(msg.LHip)
    }
    else {
      resolved.LHip = new BodyPartDetection()
    }

    if (msg.LKnee !== undefined) {
      resolved.LKnee = BodyPartDetection.Resolve(msg.LKnee)
    }
    else {
      resolved.LKnee = new BodyPartDetection()
    }

    if (msg.LAnkle !== undefined) {
      resolved.LAnkle = BodyPartDetection.Resolve(msg.LAnkle)
    }
    else {
      resolved.LAnkle = new BodyPartDetection()
    }

    if (msg.REye !== undefined) {
      resolved.REye = BodyPartDetection.Resolve(msg.REye)
    }
    else {
      resolved.REye = new BodyPartDetection()
    }

    if (msg.LEye !== undefined) {
      resolved.LEye = BodyPartDetection.Resolve(msg.LEye)
    }
    else {
      resolved.LEye = new BodyPartDetection()
    }

    if (msg.REar !== undefined) {
      resolved.REar = BodyPartDetection.Resolve(msg.REar)
    }
    else {
      resolved.REar = new BodyPartDetection()
    }

    if (msg.LEar !== undefined) {
      resolved.LEar = BodyPartDetection.Resolve(msg.LEar)
    }
    else {
      resolved.LEar = new BodyPartDetection()
    }

    if (msg.Chest !== undefined) {
      resolved.Chest = BodyPartDetection.Resolve(msg.Chest)
    }
    else {
      resolved.Chest = new BodyPartDetection()
    }

    return resolved;
    }
};

module.exports = PersonDetection;
