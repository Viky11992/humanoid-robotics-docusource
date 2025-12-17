# Quickstart Guide: Integrated RAG Chatbot Development

## Overview
This guide provides a step-by-step process to set up and run the RAG chatbot system that integrates with your Docusaurus-published book.

## Prerequisites

### System Requirements
- Python 3.11+
- Node.js 18+ (for Docusaurus)
- Git
- Access to Cohere API (free tier key)
- Access to Qdrant Cloud (free tier)
- Access to Neon Serverless Postgres (free tier)

### Required API Keys
- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: Your Qdrant Cloud URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `NEON_DATABASE_URL`: Your Neon Postgres connection string
- `GENERAL_API_KEY`: Your custom API key for authentication

## Setup Instructions

### 1. Clone and Initialize Repository
```bash
git clone <your-repo-url>
cd <repo-name>
cd backend
```

### 2. Set Up Backend Environment
```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Configure Environment Variables
Create a `.env` file in the backend directory:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
NEON_DATABASE_URL=your_neon_database_url_here
GENERAL_API_KEY=your_general_api_key_here
QDRANT_COLLECTION_NAME=book_content
```

### 4. Install Backend Dependencies
```bash
pip install fastapi uvicorn cohere qdrant-client asyncpg python-dotenv pydantic
```

### 5. Set Up Frontend (Docusaurus)
```bash
cd ../frontend  # or wherever your Docusaurus site is located
npm install
```

## Running the System

### 1. Start the Backend API
```bash
cd backend
uvicorn src.main:app --reload --port 8000
```

### 2. Ingest Book Content
```bash
# Run the ingestion script to process your book content
python -m src.ingestion.book_ingester
```

### 3. Run Tests
```bash
# Backend tests
python -m pytest tests/

# Or with coverage
python -m pytest --cov=src tests/
```

### 4. Start Docusaurus Frontend
```bash
cd my-website  # or your Docusaurus directory
npm run start
```

## API Endpoints

### Chat Endpoint
```
POST /api/v1/chat
```
Request body:
```json
{
  "query": "Your question here",
  "session_id": "optional session ID",
  "selected_text": "optional selected text for context restriction"
}
```

### Embeddings Endpoint
```
POST /api/v1/embeddings
```
Request body:
```json
{
  "text": "Text to embed",
  "input_type": "query" or "document"
}
```

### Health Check
```
GET /api/v1/health
```

## Integration with Docusaurus

### 1. Add Chatbot Component
Include the chatbot component in your Docusaurus pages:
```jsx
// In your Docusaurus layout or specific pages
import RagChatbot from './src/components/RagChatbot';

function Layout({children}) {
  return (
    <>
      {children}
      <RagChatbot />
    </>
  );
}
```

### 2. Enable Text Selection Capture
The chatbot automatically captures selected text and restricts context when users select text and ask questions.

## Testing the System

### 1. Verify Backend
```bash
curl -X GET http://localhost:8000/api/v1/health
```

### 2. Test Chat Functionality
```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_general_api_key_here" \
  -d '{
    "query": "What is the main concept of this book?",
    "session_id": "test-session-123"
  }'
```

### 3. Test with Selected Text
```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_general_api_key_here" \
  -d '{
    "query": "Explain this concept in more detail",
    "selected_text": "The main concept is retrieval-augmented generation...",
    "session_id": "test-session-123"
  }'
```

## Troubleshooting

### Common Issues

#### API Rate Limits
- **Issue**: Cohere or Qdrant rate limits exceeded
- **Solution**: Implement request queuing or increase time between requests

#### Vector Database Connection
- **Issue**: Cannot connect to Qdrant
- **Solution**: Verify URL and API key in environment variables

#### Embedding Generation Failure
- **Issue**: Embedding generation fails
- **Solution**: Check Cohere API key and rate limits

### Checking System Status
```bash
# Check backend health
curl http://localhost:8000/api/v1/health

# Check if all services are accessible
python -c "import cohere, qdrant_client, asyncpg; print('All imports successful')"
```

## Next Steps

1. Customize the chatbot UI to match your Docusaurus theme
2. Add more sophisticated error handling
3. Implement caching for better performance
4. Add analytics to track usage patterns
5. Set up monitoring and alerting for production use