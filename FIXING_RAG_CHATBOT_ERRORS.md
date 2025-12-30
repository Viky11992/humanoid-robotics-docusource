# Fixing RAG Chatbot "Error Processing Request" Issues

## Common Issues and Solutions

### 1. Authentication Issues
- **Problem**: The backend requires an API key for authentication
- **Solution**: Ensure the API key is properly configured in both backend and frontend

### 2. Fixed Backend Agent Router
- **Problem**: The agent router was not using the correct authentication method
- **Solution**: Updated the agent router to use `Depends(api_key_auth)` like other endpoints

### 3. Configuration Verification
The following configurations have been verified:

**Backend (.env file)**:
- `GENERAL_API_KEY=OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU`

**Docusaurus Configuration (docusaurus.config.ts)**:
- API key meta tag contains the correct value
- API base URL is set to `http://127.0.0.1:8000`

**Frontend Chatbot (rag-chatbot.js)**:
- Retrieves API key from meta tag properly
- Uses correct API base URL
- Sends Authorization header with Bearer token

## How to Run the System

### Step 1: Start the Backend Server
```bash
cd backend
uvicorn src.main:app --host 127.0.0.1 --port 8000
```

### Step 2: Start the Frontend (Docusaurus)
In a new terminal:
```bash
cd my-website
npm install  # if running for the first time
npm start
```

### Step 3: Access the Application
- Backend: http://127.0.0.1:8000
- Frontend: http://localhost:3000
- Chatbot: Available as a floating button on the bottom-right of the page

## Troubleshooting Steps

1. **Verify Backend is Running**:
   - Check that the backend server is accessible at http://127.0.0.1:8000
   - Test the health endpoint: http://127.0.0.1:8000/api/v1/health

2. **Check API Key Configuration**:
   - Ensure the API key in backend/.env matches the one in docusaurus.config.ts
   - Verify the meta tag in the Docusaurus config contains the correct API key

3. **Check Browser Console**:
   - Open browser developer tools (F12)
   - Look for network errors in the Network tab
   - Check console for JavaScript errors

4. **Verify CORS Settings**:
   - Backend should allow origins including http://localhost:3000

5. **Test API Directly**:
   ```bash
   curl -X POST http://127.0.0.1:8000/api/v1/agent/ask \
     -H "Content-Type: application/json" \
     -H "Authorization: Bearer OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU" \
     -d '{"question": "What is Physical AI?"}'
   ```

## Expected Response Format
When working correctly, the API should return:
```json
{
  "answer": "Response text...",
  "sources": [...],
  "session_id": "...",
  "query_time": 123.45
}
```

## Common Error Messages and Solutions

- **"Backend API is not available"**: Backend server is not running
- **"Invalid API key"**: API key mismatch between frontend and backend
- **"Authorization header with Bearer token required"**: Authorization header missing or malformed
- **"Internal server error"**: Backend processing error, check backend logs
- **"Sorry, there was an error processing your request"**: General error, check browser console and backend logs