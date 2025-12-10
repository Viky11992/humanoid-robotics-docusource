# Integrating Your Textbook Content with RAG Chatbot

This guide explains how to add your Docusaurus textbook content to the RAG Chatbot system.

## 📚 Method 1: Update Frontend Content (Recommended for browsing)

1. **Find your textbook content** from your Docusaurus site
2. **Edit the file**: `frontend/index.html`
3. **Replace the placeholder content** in the `<div class="book-text" id="bookText">` section with your actual textbook content
4. **Save the file** and refresh the browser

## 🚀 Method 2: API Content Ingestion (Recommended for full RAG)

### Using the Ingestion Script:
```bash
python ingest_textbook.py
```

This script provides two options:
- **Option 1**: Load content from a text file
- **Option 2**: Paste content directly

### Manual API Ingestion:
1. Go to: `http://localhost:8000/docs`
2. Click on `/rag/ingest`
3. Click "Try it out"
4. Paste your textbook content in the request body:
   ```json
   {
     "text": "YOUR ENTIRE TEXTBOOK CONTENT GOES HERE..."
   }
   ```
5. Click "Execute"

## 🎯 Using Your Textbook Content

Once your content is loaded:

### For Frontend Method:
- Open `frontend/index.html` in your browser
- You'll see your textbook content in the left panel
- Select text from your content to provide context
- Ask questions in the chat interface

### For API Method:
- Your content is stored in the vector database
- Ask questions without selecting text - the system will automatically search for relevant content
- The AI will respond based on your textbook content

## 📝 Tips for Best Results

1. **Format your content** with clear headings and paragraphs
2. **Include relevant keywords** that you might ask about
3. **Use structured content** with chapters, sections, etc.
4. **Test with sample questions** to ensure the RAG system works properly

## 🔄 Updating Content

- **For frontend changes**: Simply edit `frontend/index.html` and refresh the browser
- **For API content**: Use the ingestion script or API endpoint again (this will add to existing content)

## ✅ Verification

To verify your content is working:
1. Ask a question about a specific topic from your textbook
2. Check if the response references information from your content
3. Look for sources in the response that point to your textbook content

Your RAG Chatbot is now ready to work with your Docusaurus textbook content! 🎉