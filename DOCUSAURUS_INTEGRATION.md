# Docusaurus Integration Guide for RAG Chatbot

This guide explains how to integrate the RAG Chatbot directly into your Docusaurus site with a button that opens the chat interface.

## 🚀 Quick Integration Steps

### 1. Add the Chatbot Component to Your Docusaurus Site

#### Option A: Using a Custom React Component (Recommended)

1. **Create a new component** in your Docusaurus site:
   ```
   my-website/src/components/RAGChatbot/
   ├── Chatbot.js
   └── Chatbot.module.css
   ```

2. **Copy the contents** of `chatbot-component.js` into your React component

3. **Import and use** the component in your Docusaurus layout:
   ```jsx
   // In src/theme/Layout/index.js or your main layout
   import React from 'react';
   import RAGChatbot from '@site/src/components/RAGChatbot/Chatbot';

   export default function Layout(props) {
     return (
       <>
         <OriginalLayout {...props} />
         <RAGChatbot />
       </>
     );
   }
   ```

#### Option B: Using a Custom HTML Script (Simpler)

The integration has already been completed! The chatbot component has been added to your Docusaurus project:

1. The chatbot script is located at: `my-website/static/js/rag-chatbot.js`
2. The script is already configured in your `docusaurus.config.ts`:
   ```javascript
   scripts: [
     { src: '/js/rag-chatbot.js', defer: true }
   ],
   ```

### 2. Update API Configuration

Before using, update the configuration in the script:

1. **Find these lines in** `my-website/static/js/rag-chatbot.js`:
   ```javascript
   this.apiBaseUrl = 'http://localhost:8000'; // Update this to your deployed backend URL
   this.apiKey = 'OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU'; // Update this to your API key
   ```

2. **Update with your actual values**:
   - `apiBaseUrl`: Your backend URL (for production deployment)
   - `apiKey`: Your actual API key

### 3. For Development (Testing)

If you're running the backend locally and want to test with your Docusaurus site:

1. **Start your Docusaurus site**:
   ```bash
   cd my-website
   npm run start
   ```

2. **Keep the backend running** (port 8000):
   ```bash
   cd D:\Heckathone-001\Humanoid-Robotics
   uvicorn backend.main:app --host 0.0.0.0 --port 8000
   ```

3. **Access your Docusaurus site** (usually at http://localhost:3000)
4. **The chatbot button** will appear in the bottom-right corner

### 4. For Production Deployment

1. **Deploy your backend** to a server (e.g., Render, Vercel, AWS, etc.)
2. **Update the API URL** in the chatbot component to point to your deployed backend
3. **Build and deploy** your Docusaurus site with the integrated chatbot

## 🎯 Features of the Integrated Chatbot

- **Floating button** in bottom-right corner
- **One-click open/close** functionality
- **Context-aware** - can use current page content as context
- **Responsive design** that works on all devices
- **Seamless integration** with your existing Docusaurus site

## 🔧 Customization Options

### Change Button Position
Modify the CSS in the component:
```css
#rag-chatbot-container {
  bottom: 20px;  /* Change position */
  right: 20px;   /* Change position */
}
```

### Change Colors
Update the color variables in the CSS:
```css
.chatbot-button {
  background: #your-color;  /* Change button color */
}
```

### Change Size
Adjust the window dimensions:
```css
.chatbot-window {
  width: 400px;   /* Adjust width */
  height: 500px;  /* Adjust height */
}
```

## 🚨 Important Notes

1. **CORS**: If your Docusaurus site and backend are on different domains, you may need to configure CORS in your backend.

2. **API Key Security**: For production, consider using a more secure method to handle the API key.

3. **Local Development**: The default setup assumes your backend runs on `http://localhost:8000`.

## 📞 Troubleshooting

### Chatbot doesn't appear
- Check browser console for JavaScript errors
- Verify the script is loaded in your Docusaurus config
- Ensure your backend is running

### API calls failing
- Verify the backend URL is correct
- Check that your API key is properly configured
- Ensure your backend is accessible from your Docusaurus site

### Button appears but chat doesn't work
- Check network tab for API call errors
- Verify the API key is correct
- Confirm the backend endpoints are accessible

Your RAG Chatbot is now ready to be integrated into your Docusaurus documentation! 🎉