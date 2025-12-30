# API Configuration

## Frontend API Configuration

The frontend needs to be properly configured to connect to the backend API.

### Environment Variables

Create a `.env` file in the `frontend` directory with the following variables:

```env
REACT_APP_API_BASE_URL=http://127.0.0.1:8000
REACT_APP_API_KEY=your_api_key_here  # Optional, depending on backend auth requirements
```

### Configuration Details

- **Base URL**: `http://127.0.0.1:8000` - Points to the local backend server
- **API Key**: Optional, can be set in environment variables or configured at runtime
- **Endpoints**:
  - Chat: `/api/v1/agent/ask`
  - Health: `/api/v1/health`
  - Embeddings: `/api/v1/embeddings`

### Backend Server

Make sure the backend server is running on `http://127.0.0.1:8000` before starting the frontend.

To start the backend:
```bash
cd backend
uvicorn src.main:app --host 127.0.0.1 --port 8000
```

To start the frontend:
```bash
cd frontend
npm start  # or your appropriate start command
```