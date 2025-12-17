# API Contract: Chat Service

## Overview
Contract for the RAG chatbot API service that handles user queries and returns contextually relevant responses.

## Service Information
- **Base URL**: `/api/v1`
- **Version**: 1.0.0
- **Protocol**: HTTPS/REST
- **Content-Type**: `application/json`

## Endpoints

### POST /chat

#### Description
Processes user queries and returns RAG-enhanced responses based on book content or selected text.

#### Request

**Headers:**
- `Authorization: Bearer {api_key}` (Required)
- `Content-Type: application/json` (Required)

**Body:**
```json
{
  "query": {
    "type": "string",
    "required": true,
    "minLength": 1,
    "maxLength": 1000,
    "description": "User's question or query"
  },
  "session_id": {
    "type": "string",
    "required": false,
    "description": "Unique session identifier for conversation history"
  },
  "selected_text": {
    "type": "string",
    "required": false,
    "maxLength": 5000,
    "description": "Text selected by user to restrict context"
  },
  "context_restrict": {
    "type": "boolean",
    "required": false,
    "default": false,
    "description": "Whether to restrict response to selected text only"
  },
  "max_results": {
    "type": "integer",
    "required": false,
    "default": 5,
    "minimum": 1,
    "maximum": 10,
    "description": "Maximum number of context results to retrieve"
  },
  "similarity_threshold": {
    "type": "number",
    "required": false,
    "default": 0.7,
    "minimum": 0.0,
    "maximum": 1.0,
    "description": "Minimum similarity score for retrieved context"
  }
}
```

**Example Request:**
```json
{
  "query": "What are the key principles of RAG systems?",
  "session_id": "sess_12345",
  "selected_text": "The key principles include retrieval accuracy and generation quality.",
  "context_restrict": true,
  "max_results": 5
}
```

#### Response

**Success Response (200 OK):**
```json
{
  "response": {
    "type": "string",
    "description": "The generated response to the user's query"
  },
  "sources": {
    "type": "array",
    "items": {
      "type": "object",
      "properties": {
        "content": { "type": "string" },
        "score": { "type": "number", "minimum": 0.0, "maximum": 1.0 },
        "source": { "type": "string" }
      },
      "required": ["content", "score", "source"]
    }
  },
  "session_id": {
    "type": "string",
    "description": "The session identifier (newly created if not provided)"
  },
  "query_time": {
    "type": "number",
    "description": "Time taken to process the query in seconds"
  }
}
```

**Example Response:**
```json
{
  "response": "RAG systems combine retrieval and generation components to provide accurate responses based on retrieved context...",
  "sources": [
    {
      "content": "The key principles of RAG systems include retrieval accuracy...",
      "score": 0.89,
      "source": "chapter_3_introduction.md"
    }
  ],
  "session_id": "sess_12345",
  "query_time": 1.23
}
```

**Error Responses:**

- `400 Bad Request`: Invalid request format
- `401 Unauthorized`: Missing or invalid API key
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Service error

#### Security
- All requests require API key authentication
- Rate limiting: 100 requests per minute per API key
- Input validation: Query length, content filtering

### POST /embeddings

#### Description
Generates embeddings for text content used in the RAG system.

#### Request

**Headers:**
- `Authorization: Bearer {api_key}` (Required)
- `Content-Type: application/json` (Required)

**Body:**
```json
{
  "text": {
    "type": "string",
    "required": true,
    "maxLength": 5000,
    "description": "Text to generate embeddings for"
  },
  "input_type": {
    "type": "string",
    "required": false,
    "enum": ["query", "document"],
    "default": "document",
    "description": "Type of input text (affects embedding generation)"
  }
}
```

#### Response

**Success Response (200 OK):**
```json
{
  "embeddings": {
    "type": "array",
    "items": { "type": "number" },
    "description": "Array of embedding values"
  },
  "text": {
    "type": "string",
    "description": "Original text that was embedded"
  },
  "model": {
    "type": "string",
    "description": "Model used for embedding generation"
  }
}
```

### GET /health

#### Description
Health check endpoint to verify service availability.

#### Response

**Success Response (200 OK):**
```json
{
  "status": {
    "type": "string",
    "enum": ["healthy", "degraded", "unhealthy"]
  },
  "services": {
    "type": "object",
    "properties": {
      "cohere": { "type": "boolean" },
      "qdrant": { "type": "boolean" },
      "postgres": { "type": "boolean" }
    }
  },
  "timestamp": {
    "type": "string",
    "format": "date-time"
  }
}
```

## Performance Requirements

- **Response Time**: 95% of requests should respond within 5 seconds
- **Availability**: 99% uptime
- **Throughput**: Support 100 concurrent users

## Error Handling

### Standard Error Format
```json
{
  "error": {
    "type": "string",
    "description": "Error type"
  },
  "message": {
    "type": "string",
    "description": "Human-readable error message"
  },
  "details": {
    "type": "object",
    "description": "Additional error details"
  }
}
```

### Common Error Types
- `INVALID_INPUT`: Request validation failed
- `AUTHENTICATION_FAILED`: Invalid or missing API key
- `RESOURCE_EXHAUSTED`: Rate limit or quota exceeded
- `SERVICE_UNAVAILABLE`: External service (Cohere/Qdrant) unavailable
- `INTERNAL_ERROR`: Unexpected server error

## Change Log

### 1.0.0 (2025-12-16)
- Initial contract definition
- Added chat and embeddings endpoints
- Defined health check endpoint