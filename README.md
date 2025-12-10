# RAG Chatbot for Book Content

A Retrieval-Augmented Generation chatbot integrated with Google Gemini 2.5 Flash, FastAPI backend, Neon Postgres, and Qdrant vector database. The agent answers questions about book content, including questions based on user-selected text.

## Features

- **Question Answering**: Ask questions about book content with context grounding
- **Text Selection**: Select text from book content to provide specific context
- **Vector Search**: Automatic vector search when no specific context is provided
- **API Authentication**: x-api-key authentication for all endpoints
- **Query Logging**: All queries and responses are logged to Neon Postgres
- **Source Tracking**: Shows which sources were used to generate answers

## Architecture

- **Backend**: FastAPI application with multiple modules
- **Vector Store**: Qdrant for efficient similarity search
- **Database**: Neon Postgres for metadata and query logs
- **AI Model**: Google Gemini 2.5 Flash with Google embedding model for embeddings
- **Frontend**: HTML/CSS/JavaScript chat interface

## Prerequisites

- Python 3.8+
- Google API key (for Gemini 2.5 Flash and embeddings)
- Qdrant Cloud account (or local instance)
- Neon Postgres database

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd rag-chatbot
   ```

2. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Configure environment variables**
   ```bash
   cp .env.example .env  # if you have an example file
   # Edit .env with your API keys and connection strings
   ```

4. **Set up Qdrant collection**
   - The application will automatically create the required collection on startup

5. **Set up database schema**
   - Execute the SQL from `backend/schema.sql` in your Neon Postgres database

## Environment Variables

Create a `.env` file with the following variables:

```env
GOOGLE_API_KEY=your_google_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=https://your-cluster-url.qdrant.tech  # or leave empty for local
NEON_DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require
API_KEY=your_api_key_for_authentication
```

## Running the Application

### Backend

```bash
cd backend
python main.py
```

The API will be available at `http://localhost:8000`

### Frontend

Serve the frontend files using any static file server, or open `frontend/index.html` directly in a browser.

## API Endpoints

### POST `/agent/ask`
Ask questions with optional selected text context.

Request body:
```json
{
  "question": "string",
  "selected_text": "string (optional)"
}
```

Response:
```json
{
  "answer": "string",
  "sources": ["string"]
}
```

### POST `/rag/ingest`
Ingest book text for RAG functionality.

Request body:
```json
{
  "text": "full book content as string"
}
```

Response:
```json
{
  "status": "success"
}
```

### POST `/rag/search`
Direct vector search in the RAG database.

Request body:
```json
{
  "query": "search query string"
}
```

Response:
```json
{
  "results": [
    {
      "id": "string",
      "text": "string",
      "score": "number"
    }
  ]
}
```

## Frontend Usage

1. Open `frontend/index.html` in your browser
2. Select text from the book content area to provide context
3. Type your question in the input field
4. Click "Send" or press Enter to get an answer
5. View sources used in the response in the sources panel

## Testing

Run the tests using pytest:

```bash
pytest tests/
```

## Deployment

### Backend
The application is designed to be deployed as serverless functions. Ensure all environment variables are properly configured in your deployment environment.

### Frontend
The frontend can be deployed to any static hosting service (Vercel, Netlify, etc.).

## Security

- All API endpoints require `x-api-key` authentication
- Input validation is performed on all endpoints
- SQL injection is prevented through parameterized queries
- Rate limiting should be implemented at the infrastructure level

## Performance

- Response times are optimized with efficient vector search
- Connection pooling is used for database operations
- Embedding generation is batched for efficiency