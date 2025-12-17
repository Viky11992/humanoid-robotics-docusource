# RAG Chatbot for Humanoid Robotics Book

This project implements a Retrieval-Augmented Generation (RAG) chatbot that integrates with a Docusaurus-published book about humanoid robotics. The system allows users to ask questions about the book content and receive accurate, contextually relevant responses.

## Features

- **Full-book queries**: Ask questions about the entire book content
- **Selected text queries**: Highlight text in the book and ask questions about only that selected content
- **Source citations**: Responses include citations to the original book content
- **Conversation history**: Maintains session context for follow-up questions
- **Docusaurus integration**: Seamless integration with Docusaurus sites
- **Rate limiting**: Prevents API abuse with configurable rate limits
- **Authentication**: API key protection for backend endpoints

## Architecture

The system consists of:

- **Frontend**: React components for Docusaurus integration
- **Backend**: FastAPI application with RAG capabilities
- **Embeddings**: Cohere for generating text embeddings
- **Vector Storage**: Qdrant for storing and retrieving embeddings
- **Metadata**: Neon Postgres for session and conversation history

## API Endpoints

### Chat Endpoint
- `POST /api/v1/chat` - Process queries using RAG methodology
- Supports both full-book and selected-text queries
- Returns response with source citations

### Agent Endpoint
- `POST /api/v1/agent/ask` - Special endpoint for Docusaurus integration
- Handles selected text context from page content
- Returns formatted responses for the chat interface

### Health Check
- `GET /api/v1/health` - Check service health and dependencies
- `GET /api/v1/ready` - Check if service is ready for requests

## Environment Variables

Create a `.env` file with the following variables:

```env
GENERAL_API_KEY=your_general_api_key_here
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
DATABASE_URL=your_database_url_here
```

## Frontend Integration

The Docusaurus integration is provided via `my-website/static/js/rag-chatbot.js`, which creates a floating chat widget that:
- Captures selected text from the current page
- Restricts context to selected text when available
- Provides a seamless user experience

## Development

1. Install dependencies: `pip install -r requirements.txt`
2. Set up environment variables in `.env`
3. Start the server: `python -m uvicorn src.main:app --reload --port 8000`
4. Access the API documentation at `http://localhost:8000/docs`

## Deployment

The application is configured for deployment to Railway with the provided `railway.toml` file. Ensure all environment variables are set in your deployment environment.