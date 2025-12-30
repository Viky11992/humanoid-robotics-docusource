# Running Frontends for the Humanoid Robotics Project

This project has two frontend implementations:
1. A Docusaurus-based documentation site with integrated chatbot (recommended)
2. API service files that can be integrated into a React application

## Prerequisites

Before running any frontend, make sure:
1. Node.js (version 20 or higher) is installed
2. Python (with the backend dependencies) is installed
3. The backend server is running

## Running the Backend Server

First, start the backend server (required for both frontends):

```bash
cd backend
uvicorn src.main:app --host 127.0.0.1 --port 8000
```

Keep this terminal window open as the backend needs to remain running.

## Option 1: Running the Docusaurus Documentation Site (Recommended)

The Docusaurus site includes the RAG chatbot integrated directly into the documentation.

1. Open a new terminal window and navigate to the website directory:
```bash
cd my-website
```

2. Install dependencies (first time only):
```bash
npm install
```

3. Start the development server:
```bash
npm start
```

The site will be available at `http://localhost:3000` and the chatbot will be integrated into the pages.

## Option 2: Using the API Service in a React Application

The `frontend` directory contains API service files that can be integrated into a React application:

- API service: `frontend/src/services/api.js`
- Configuration: `frontend/.env`

To use these in a React application:
1. Create a new React app or use an existing one
2. Copy the API service files to your React project
3. Add the environment variables to your React app's `.env` file
4. Import and use the ApiService in your components

## API Configuration

Both implementations are configured to connect to the backend at `http://127.0.0.1:8000`:

- **Docusaurus site**: Configured in `my-website/docusaurus.config.ts`
- **React service**: Configured in `frontend/.env` and `frontend/src/services/api.js`

The main API endpoint used is `/api/v1/agent/ask` for chat functionality.

## Troubleshooting

If you encounter "Backend API is not available" errors:

1. Verify the backend server is running on `http://127.0.0.1:8000`
2. Check that CORS settings in the backend allow requests from your frontend origin
3. Ensure all servers are running simultaneously
4. Check the browser console for specific error messages

## Development Notes

- The Docusaurus site is the primary frontend with better integration
- The API service files provide a ready-to-use service for React applications
- The chatbot functionality is available in the Docusaurus site
- Make sure to run the backend server before starting the frontend