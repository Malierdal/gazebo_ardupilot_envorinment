// Auto-generated. Do not edit!

// (in-package autonomous_locking.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Target {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.pose = null;
      this.velocity = null;
      this.heading = null;
      this.speed = null;
      this.time_delay = null;
      this.distance = null;
      this.tail_angle = null;
      this.heading_alignment = null;
      this.zone_score = null;
      this.total_cost = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('heading')) {
        this.heading = initObj.heading
      }
      else {
        this.heading = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('time_delay')) {
        this.time_delay = initObj.time_delay
      }
      else {
        this.time_delay = 0.0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
      if (initObj.hasOwnProperty('tail_angle')) {
        this.tail_angle = initObj.tail_angle
      }
      else {
        this.tail_angle = 0.0;
      }
      if (initObj.hasOwnProperty('heading_alignment')) {
        this.heading_alignment = initObj.heading_alignment
      }
      else {
        this.heading_alignment = 0.0;
      }
      if (initObj.hasOwnProperty('zone_score')) {
        this.zone_score = initObj.zone_score
      }
      else {
        this.zone_score = 0.0;
      }
      if (initObj.hasOwnProperty('total_cost')) {
        this.total_cost = initObj.total_cost
      }
      else {
        this.total_cost = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Target
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.velocity, buffer, bufferOffset);
    // Serialize message field [heading]
    bufferOffset = _serializer.float32(obj.heading, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float32(obj.speed, buffer, bufferOffset);
    // Serialize message field [time_delay]
    bufferOffset = _serializer.float32(obj.time_delay, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float32(obj.distance, buffer, bufferOffset);
    // Serialize message field [tail_angle]
    bufferOffset = _serializer.float32(obj.tail_angle, buffer, bufferOffset);
    // Serialize message field [heading_alignment]
    bufferOffset = _serializer.float32(obj.heading_alignment, buffer, bufferOffset);
    // Serialize message field [zone_score]
    bufferOffset = _serializer.float32(obj.zone_score, buffer, bufferOffset);
    // Serialize message field [total_cost]
    bufferOffset = _serializer.float32(obj.total_cost, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Target
    let len;
    let data = new Target(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [heading]
    data.heading = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [time_delay]
    data.time_delay = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tail_angle]
    data.tail_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_alignment]
    data.heading_alignment = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [zone_score]
    data.zone_score = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [total_cost]
    data.total_cost = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 140;
  }

  static datatype() {
    // Returns string type for a message object
    return 'autonomous_locking/Target';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '563c719c20ae1eacdb952c207c883d48';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 id
    geometry_msgs/Pose pose
    geometry_msgs/Twist velocity
    float32 heading
    float32 speed
    float32 time_delay
    
    float32 distance
    float32 tail_angle
    float32 heading_alignment
    float32 zone_score
    float32 total_cost
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3 linear
    Vector3 angular
    
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
    const resolved = new Target(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = geometry_msgs.msg.Twist.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new geometry_msgs.msg.Twist()
    }

    if (msg.heading !== undefined) {
      resolved.heading = msg.heading;
    }
    else {
      resolved.heading = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.time_delay !== undefined) {
      resolved.time_delay = msg.time_delay;
    }
    else {
      resolved.time_delay = 0.0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    if (msg.tail_angle !== undefined) {
      resolved.tail_angle = msg.tail_angle;
    }
    else {
      resolved.tail_angle = 0.0
    }

    if (msg.heading_alignment !== undefined) {
      resolved.heading_alignment = msg.heading_alignment;
    }
    else {
      resolved.heading_alignment = 0.0
    }

    if (msg.zone_score !== undefined) {
      resolved.zone_score = msg.zone_score;
    }
    else {
      resolved.zone_score = 0.0
    }

    if (msg.total_cost !== undefined) {
      resolved.total_cost = msg.total_cost;
    }
    else {
      resolved.total_cost = 0.0
    }

    return resolved;
    }
};

module.exports = Target;
