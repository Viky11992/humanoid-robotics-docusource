# Running the Frontend (Docusaurus Website)

To run the frontend website with the RAG Chatbot, follow these steps:

## Prerequisites

1. Make sure you have Node.js installed (version 20 or higher)
2. Make sure the backend server is running

## Running the Backend First

Before starting the frontend, make sure the backend API is running:

```bash
cd backend
uvicorn src.main:app --host 127.0.0.1 --port 8000
```

## Running the Frontend

1. Navigate to the `my-website` directory:
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

This will start the Docusaurus development server, typically on `http://localhost:3000`.

## Configuration

The RAG Chatbot is configured in `docusaurus.config.ts`:

- The chatbot script is loaded from `/js/rag-chatbot.js`
- API configuration is provided via meta tags in the `headTags` section
- For local development, the chatbot will use `http://127.0.0.1:8000` as the API base URL (fallback)
- For production, it uses the Railway deployment URL

## Troubleshooting

If you get "Backend API is not available" errors:

1. Make sure the backend server is running on `http://127.0.0.1:8000`
2. Check that the backend CORS settings allow requests from `http://localhost:3000`
3. Verify that both servers are running simultaneously

The chatbot should appear as a floating button on the bottom-right corner of the website.