# Data Model: Integrated RAG Chatbot Development

## Overview
Data models for the RAG chatbot system including vector embeddings in Qdrant and structured data in Neon Postgres.

## Qdrant Vector Collection Schema

### Book Content Collection
- **Collection Name**: `book_content`
- **Vector Dimensions**: 1024 (Cohere multilingual v3 embedding model)
- **Distance Metric**: Cosine

#### Payload Structure:
```json
{
  "content": "string (the actual text content)",
  "source": "string (source document identifier)",
  "chunk_id": "string (unique identifier for this chunk)",
  "section": "string (book section/chapter)",
  "page_number": "integer (if applicable)",
  "metadata": {
    "word_count": "integer",
    "created_at": "timestamp",
    "updated_at": "timestamp"
  }
}
```

## Neon Postgres Schema

### Tables

#### 1. sessions
```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id VARCHAR(255),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB
);
```

#### 2. conversations
```sql
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID REFERENCES sessions(id),
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    selected_text TEXT,
    context_used TEXT[],
    sources JSONB, -- Array of source references
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB
);
```

#### 3. users (if needed for advanced features)
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    preferences JSONB
);
```

#### 4. documents
```sql
CREATE TABLE documents (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    title VARCHAR(500) NOT NULL,
    source_url VARCHAR(1000),
    content_hash VARCHAR(64), -- SHA256 hash for deduplication
    chunk_count INTEGER,
    word_count INTEGER,
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

## API Data Models

### Request Models

#### Chat Request
```python
class ChatRequest(BaseModel):
    query: str
    session_id: Optional[str] = None
    selected_text: Optional[str] = None
    context_restrict: bool = False
    max_results: int = 5
    similarity_threshold: float = 0.7
```

#### Embeddings Request
```python
class EmbeddingsRequest(BaseModel):
    text: str
    input_type: Literal["query", "document"] = "document"
```

### Response Models

#### Chat Response
```python
class Source(BaseModel):
    content: str
    score: float
    source: str

class ChatResponse(BaseModel):
    response: str
    sources: List[Source]
    session_id: str
    query_time: float
```

#### Embeddings Response
```python
class EmbeddingsResponse(BaseModel):
    embeddings: List[float]
    text: str
    model: str
```

## Data Flow Patterns

### Ingestion Flow
1. Document parsing → Text chunking
2. Chunk validation → Embedding generation
3. Embedding storage in Qdrant
4. Metadata storage in Postgres

### Query Flow
1. User query → Embedding generation
2. Vector similarity search in Qdrant
3. Context retrieval → Response generation
4. Response storage in Postgres
5. Response to user

### Selected Text Flow
1. Text selection capture → Context restriction flag
2. Direct context use → Response generation
3. No vector search → Faster response

## Data Relationships

```
sessions (1) ←→ (N) conversations
documents (1) ←→ (N) conversations (via sources)
users (1) ←→ (N) sessions (optional)
```

## Indexing Strategy

### Qdrant
- Auto-indexed on vector fields
- Payload index on frequently queried fields (source, section)

### Postgres
- Primary keys on all ID fields
- Index on session_id in conversations table
- Index on created_at for time-based queries
- Partial indexes for frequently accessed metadata

## Data Validation Rules

### Content Validation
- Minimum chunk size: 50 tokens
- Maximum chunk size: 1024 tokens
- Content deduplication using hash comparison
- Character encoding validation

### Embedding Validation
- Dimension consistency (1024 for Cohere embeddings)
- Numeric value validation
- Similarity threshold enforcement

### Session Validation
- Session expiration (24 hours of inactivity)
- Rate limiting per session
- Query length limits