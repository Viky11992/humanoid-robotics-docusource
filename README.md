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

### Backend Deployment on Railway

1. **Deploy to Railway**:
   - Go to [Railway](https://railway.app)
   - Create a new project
   - Connect to your GitHub repository
   - Select this repository and the `001-rag-chatbot` branch
   - Railway will automatically detect the Dockerfile and deploy your application

2. **Set Environment Variables**:
   - `GOOGLE_API_KEY`: Your Google API key for Gemini model
   - `DATABASE_URL`: PostgreSQL database URL (Railway provides this automatically if you add a PostgreSQL plugin)
   - `QDRANT_URL`: (Optional) Qdrant vector database URL (defaults to localhost)
   - `QDRANT_API_KEY`: (Optional) Qdrant API key
   - `API_KEY`: Authentication key for API endpoints (defaults to "OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU")

3. **Add PostgreSQL Plugin** (if not using external database):
   - In your Railway project dashboard
   - Click "New" → "Database" → "PostgreSQL"
   - Railway will automatically connect it to your application

4. **Add Qdrant Database** (if not using local):
   - You can either self-host Qdrant or use Qdrant Cloud
   - Add the connection details as environment variables

### Frontend Deployment (Docusaurus) on Vercel

1. **Deploy Docusaurus**:
   - Go to [Vercel](https://vercel.com)
   - Import your GitHub repository
   - Set the root directory to `my-website`
   - Vercel will automatically detect Docusaurus and build your site

2. **Update Chatbot API URL**:
   - After deploying the backend, update the API URL in `my-website/static/js/rag-chatbot.js`
   - Change `this.apiBaseUrl` to your Railway backend URL

## Security

- All API endpoints require `x-api-key` authentication
- Input validation is performed on all endpoints
- SQL injection is prevented through parameterized queries
- Rate limiting should be implemented at the infrastructure level

## Performance

- Response times are optimized with efficient vector search
- Connection pooling is used for database operations
- Embedding generation is batched for efficiency