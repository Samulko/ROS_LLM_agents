// Auto-generated. Do not edit!

// (in-package llm_agents.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class LLMQuery {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.query = null;
      this.agent_id = null;
    }
    else {
      if (initObj.hasOwnProperty('query')) {
        this.query = initObj.query
      }
      else {
        this.query = '';
      }
      if (initObj.hasOwnProperty('agent_id')) {
        this.agent_id = initObj.agent_id
      }
      else {
        this.agent_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LLMQuery
    // Serialize message field [query]
    bufferOffset = _serializer.string(obj.query, buffer, bufferOffset);
    // Serialize message field [agent_id]
    bufferOffset = _serializer.string(obj.agent_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LLMQuery
    let len;
    let data = new LLMQuery(null);
    // Deserialize message field [query]
    data.query = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [agent_id]
    data.agent_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.query);
    length += _getByteLength(object.agent_id);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'llm_agents/LLMQuery';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1692929dc490ff37a7139bda085c4e52';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string query
    string agent_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LLMQuery(null);
    if (msg.query !== undefined) {
      resolved.query = msg.query;
    }
    else {
      resolved.query = ''
    }

    if (msg.agent_id !== undefined) {
      resolved.agent_id = msg.agent_id;
    }
    else {
      resolved.agent_id = ''
    }

    return resolved;
    }
};

module.exports = LLMQuery;
