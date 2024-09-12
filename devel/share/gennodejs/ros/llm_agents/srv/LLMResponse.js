// Auto-generated. Do not edit!

// (in-package llm_agents.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class LLMResponseRequest {
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
    // Serializes a message object of type LLMResponseRequest
    // Serialize message field [query]
    bufferOffset = _serializer.string(obj.query, buffer, bufferOffset);
    // Serialize message field [agent_id]
    bufferOffset = _serializer.string(obj.agent_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LLMResponseRequest
    let len;
    let data = new LLMResponseRequest(null);
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
    // Returns string type for a service object
    return 'llm_agents/LLMResponseRequest';
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
    const resolved = new LLMResponseRequest(null);
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

class LLMResponseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.response = null;
    }
    else {
      if (initObj.hasOwnProperty('response')) {
        this.response = initObj.response
      }
      else {
        this.response = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LLMResponseResponse
    // Serialize message field [response]
    bufferOffset = _serializer.string(obj.response, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LLMResponseResponse
    let len;
    let data = new LLMResponseResponse(null);
    // Deserialize message field [response]
    data.response = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.response);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'llm_agents/LLMResponseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6de314e2dc76fbff2b6244a6ad70b68d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string response
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LLMResponseResponse(null);
    if (msg.response !== undefined) {
      resolved.response = msg.response;
    }
    else {
      resolved.response = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: LLMResponseRequest,
  Response: LLMResponseResponse,
  md5sum() { return '59dd40d73b99b23145c3779b008631a3'; },
  datatype() { return 'llm_agents/LLMResponse'; }
};
