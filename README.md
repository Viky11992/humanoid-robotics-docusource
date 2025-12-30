# Humanoid Robotics - Physical AI Textbook with RAG Chatbot

This project combines a comprehensive textbook on Physical AI & Humanoid Robotics with an advanced RAG (Retrieval-Augmented Generation) chatbot for interactive learning.

## Project Structure

- `backend/` - FastAPI backend with RAG capabilities, vector search, and authentication
- `my-website/` - Docusaurus-based frontend with textbook content and chatbot integration
- `frontend/` - Additional React frontend components

## Features

- **Interactive Textbook**: Comprehensive content on Physical AI & Humanoid Robotics
- **RAG Chatbot**: Ask questions about the textbook content with source citations
- **Context Awareness**: Chatbot can focus on selected text from current page
- **Full-text Search**: Search across the entire textbook content
- **Session Management**: Maintain conversation history
- **API Protection**: Rate limiting and API key authentication

## Technologies

- **Backend**: FastAPI, Python
- **Frontend**: Docusaurus, React, JavaScript
- **Vector Database**: Qdrant for embeddings storage
- **Language Model**: Cohere for text embeddings
- **Database**: PostgreSQL for metadata and session storage
- **Deployment**: Vercel (frontend), Railway (backend)

## Deployment to Vercel and Railway

### Backend (Railway)

1. Create a Railway account and new project
2. Connect to your GitHub repository
3. Set the following environment variables:
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_URL`: Your Qdrant database URL
   - `QDRANT_API_KEY`: Your Qdrant API key
   - `NEON_DATABASE_URL`: Your Neon database URL
   - `GENERAL_API_KEY`: Your general API key
   - `QDRANT_COLLECTION_NAME`: Your Qdrant collection name
   - `DEBUG`: Set to "true" for development, "false" for production
4. Set the start command to: `python -m uvicorn src.main:app --host 0.0.0.0 --port $PORT`
5. Deploy the project

### Frontend (Vercel)

1. Create a Vercel account and import your project
2. Set the build command to: `npm run build`
3. Set the output directory to: `build`
4. Set the install command to: `npm install`
5. Add environment variables:
   - `API_BASE_URL`: Your Railway backend URL (e.g., `https://your-app-production.up.railway.app`)
   - `REACT_APP_API_BASE_URL`: Same as above
   - `REACT_APP_API_KEY`: Your API key
6. Deploy the project

### Environment Configuration

For the RAG chatbot to work across environments:

- Local development: Uses `http://127.0.0.1:8000` as backend
- Vercel frontend: Automatically connects to Railway backend
- GitHub Pages: Connects to Railway backend

## Local Development

### Backend Setup

```bash
cd backend
pip install -r requirements.txt
```

Create a `.env` file with required environment variables:

```env
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_db_url
GENERAL_API_KEY=your_general_api_key
QDRANT_COLLECTION_NAME=book_content
DEBUG=true
```

Start the backend:

```bash
cd backend
python -m uvicorn src.main:app --reload --port 8000
```

### Frontend Setup

```bash
cd my-website
npm install
npm start
```

## API Endpoints

### Backend API (Running on Railway)

- `GET /` - Root endpoint
- `GET /docs` - API documentation
- `POST /api/v1/chat` - Chat endpoint
- `POST /api/v1/agent/ask` - Agent endpoint for Docusaurus integration
- `GET /api/v1/health` - Health check
- `GET /api/v1/ready` - Readiness check

### Frontend (Running on Vercel)

- `http://localhost:3000` - Local development
- `https://your-project.vercel.app` - Production

## Architecture

The system consists of:

- **Frontend**: Docusaurus site with integrated chatbot widget
- **Backend**: FastAPI application with RAG capabilities
- **Embeddings**: Cohere for generating text embeddings
- **Vector Storage**: Qdrant for storing and retrieving embeddings
- **Metadata**: Neon Postgres for session and conversation history
- **Authentication**: API key protection for backend endpoints

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License.