// RAG Chatbot Component for Docusaurus
// This component can be integrated into your Docusaurus site

class RAGChatbot extends HTMLElement {
  constructor() {
    super();
    this.isOpen = false;
    // Use environment-specific API base URL
    // For development: http://localhost:8000
    // For production deployment: update to your backend URL
    this.apiBaseUrl = this.getApiBaseUrl();
    // Get API key from meta tag or global config (to be replaced with environment variable or server-side config)
    this.apiKey = this.getApiKey(); // Updated to get API key dynamically
  }

  // Function to get API base URL based on environment
  getApiBaseUrl() {
    // Check if there's a meta tag with API base URL
    const apiBaseMeta = document.querySelector('meta[name="api-base-url"]');
    if (apiBaseMeta && apiBaseMeta.content) {
        return apiBaseMeta.content;
    }

    // For development, use localhost
    // For production GitHub Pages, you might need to update this to your backend URL
    return 'http://localhost:8000';
  }

  // Function to get API key from meta tag or other configuration source
  getApiKey() {
    // Look for API key in a meta tag or configuration element
    const apiKeyMeta = document.querySelector('meta[name="api-key"]');
    if (apiKeyMeta && apiKeyMeta.content) {
        return apiKeyMeta.content;
    }

    // You can also try to get it from a global variable set by your server-side template
    if (window.API_CONFIG && window.API_CONFIG.apiKey) {
        return window.API_CONFIG.apiKey;
    }

    // For development purposes only - this should be replaced in production
    console.warn("WARNING: Using fallback API key in Docusaurus chatbot. This should be changed before production deployment.");
    return 'OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU';
  }

  connectedCallback() {
    this.render();
    this.attachEventListeners();
  }

  render() {
    this.innerHTML = `
      <style>
        #rag-chatbot-container {
          position: fixed;
          bottom: 20px;
          right: 20px;
          z-index: 10000;
          font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
        }

        .chatbot-button {
          background: #3498db;
          color: white;
          border: none;
          border-radius: 50%;
          width: 60px;
          height: 60px;
          font-size: 24px;
          cursor: pointer;
          box-shadow: 0 4px 12px rgba(0,0,0,0.15);
          display: flex;
          align-items: center;
          justify-content: center;
          transition: all 0.3s ease;
        }

        .chatbot-button:hover {
          background: #2980b9;
          transform: scale(1.05);
        }

        .chatbot-window {
          position: absolute;
          bottom: 80px;
          right: 0;
          width: 400px;
          height: 500px;
          background: white;
          border-radius: 12px;
          box-shadow: 0 8px 30px rgba(0,0,0,0.2);
          display: flex;
          flex-direction: column;
          overflow: hidden;
          transform: translateY(20px);
          opacity: 0;
          transition: all 0.3s ease;
        }

        .chatbot-window.open {
          transform: translateY(0);
          opacity: 1;
        }

        .chat-header {
          background: #3498db;
          color: white;
          padding: 15px;
          font-weight: bold;
          display: flex;
          justify-content: space-between;
          align-items: center;
        }

        .close-button {
          background: none;
          border: none;
          color: white;
          font-size: 20px;
          cursor: pointer;
          padding: 0;
          width: 24px;
          height: 24px;
          display: flex;
          align-items: center;
          justify-content: center;
        }

        .chat-messages {
          flex: 1;
          padding: 15px;
          overflow-y: auto;
          display: flex;
          flex-direction: column;
          gap: 10px;
          background: #f8f9fa;
        }

        .message {
          max-width: 80%;
          padding: 10px 15px;
          border-radius: 18px;
          word-wrap: break-word;
        }

        .message.user {
          align-self: flex-end;
          background-color: #3498db;
          color: white;
        }

        .message.bot {
          align-self: flex-start;
          background-color: #e3f2fd;
          color: #2c3e50;
        }

        .chat-input-area {
          padding: 15px;
          background: white;
          border-top: 1px solid #eee;
          display: flex;
          gap: 10px;
        }

        .chat-input {
          flex: 1;
          padding: 10px 15px;
          border: 1px solid #ddd;
          border-radius: 20px;
          outline: none;
          font-size: 14px;
        }

        .chat-input:focus {
          border-color: #3498db;
        }

        .send-button {
          background: #3498db;
          color: white;
          border: none;
          border-radius: 20px;
          padding: 10px 15px;
          cursor: pointer;
        }

        .send-button:hover {
          background: #2980b9;
        }

        .typing-indicator {
          align-self: flex-start;
          background: #e3f2fd;
          color: #2c3e50;
          padding: 10px 15px;
          border-radius: 18px;
          font-style: italic;
          display: none;
        }

        .typing-indicator.visible {
          display: block;
        }
      </style>

      <div id="rag-chatbot-container">
        <button class="chatbot-button" id="chatbot-toggle">
          💬
        </button>
        <div class="chatbot-window" id="chatbot-window">
          <div class="chat-header">
            <span>RAG Chatbot</span>
            <button class="close-button" id="close-chat">×</button>
          </div>
          <div class="chat-messages" id="chat-messages">
            <div class="message bot">
              Hello! I'm your RAG Chatbot. I can answer questions about this documentation. What would you like to know?<br><br><i>Note: This chatbot requires a backend service to function. If responses fail, the backend may not be accessible.</i>
            </div>
          </div>
          <div class="typing-indicator" id="typing-indicator">AI is thinking...</div>
          <div class="chat-input-area">
            <input type="text" class="chat-input" id="chat-input" placeholder="Ask about this documentation...">
            <button class="send-button" id="send-button">Send</button>
          </div>
        </div>
      </div>
    `;
  }

  attachEventListeners() {
    const toggleButton = this.querySelector('#chatbot-toggle');
    const closeButton = this.querySelector('#close-chat');
    const sendButton = this.querySelector('#send-button');
    const chatInput = this.querySelector('#chat-input');
    const chatWindow = this.querySelector('#chatbot-window');

    toggleButton.addEventListener('click', () => this.toggleChat());
    closeButton.addEventListener('click', () => this.closeChat());
    sendButton.addEventListener('click', () => this.sendMessage());

    chatInput.addEventListener('keypress', (e) => {
      if (e.key === 'Enter') {
        this.sendMessage();
      }
    });

    // Close chat when clicking outside
    document.addEventListener('click', (e) => {
      if (!chatWindow.contains(e.target) &&
          !toggleButton.contains(e.target) &&
          this.isOpen) {
        // Don't close if clicking on other interactive elements
        if (!e.target.closest('.button') && !e.target.closest('a') && !e.target.closest('input')) {
          this.closeChat();
        }
      }
    });
  }

  toggleChat() {
    this.isOpen = !this.isOpen;
    const chatWindow = this.querySelector('#chatbot-window');
    chatWindow.classList.toggle('open', this.isOpen);

    if (this.isOpen) {
      this.querySelector('#chat-input').focus();
    }
  }

  closeChat() {
    this.isOpen = false;
    const chatWindow = this.querySelector('#chatbot-window');
    chatWindow.classList.remove('open');
  }

  async sendMessage() {
    const input = this.querySelector('#chat-input');
    const message = input.value.trim();

    if (!message) return;

    // Add user message to chat
    this.addMessage(message, 'user');
    input.value = '';

    // Show typing indicator
    const typingIndicator = this.querySelector('#typing-indicator');
    typingIndicator.classList.add('visible');

    try {
      // Get current page content as context (if available)
      const currentPageContent = this.getCurrentPageContent();

      // Call the RAG API
      const response = await fetch(`${this.apiBaseUrl}/api/v1/agent/ask`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.apiKey}`
        },
        body: JSON.stringify({
          question: message,
          selected_text: currentPageContent ? currentPageContent.substring(0, 2000) : null, // Limit context
          context_restrict: !!currentPageContent // Restrict to selected text if available
        })
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Hide typing indicator
      typingIndicator.classList.remove('visible');

      // Add bot response to chat
      this.addMessage(data.answer, 'bot');

      // Log the interaction
      console.log('Chat response:', data);

    } catch (error) {
      console.error('Error sending message:', error);
      typingIndicator.classList.remove('visible');

      // Provide a more helpful message depending on the error type
      if (error.message.includes('404') || error.message.includes('Failed to fetch')) {
        this.addMessage('Backend API is not available. This chatbot requires a running backend service to function. The API endpoint might not be accessible from this environment.', 'bot');
      } else {
        this.addMessage('Sorry, there was an error processing your request. Please try again.', 'bot');
      }
    }
  }

  addMessage(text, sender) {
    const messagesContainer = this.querySelector('#chat-messages');
    const messageDiv = document.createElement('div');
    messageDiv.className = `message ${sender}`;
    messageDiv.textContent = text;
    messagesContainer.appendChild(messageDiv);

    // Scroll to bottom
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }

  getCurrentPageContent() {
    // Try to get content from common Docusaurus elements
    const contentSelectors = [
      '.markdown', // Docusaurus markdown content
      '.theme-doc-markdown', // Docusaurus doc content
      'article', // General article content
      '.main-wrapper' // Main content wrapper
    ];

    for (const selector of contentSelectors) {
      const element = document.querySelector(selector);
      if (element) {
        return element.innerText || element.textContent || '';
      }
    }

    // Fallback to body content
    return document.body.innerText || document.body.textContent || '';
  }
}

// Register the custom element
customElements.define('rag-chatbot', RAGChatbot);

// Auto-initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
  // Wait a bit to ensure Docusaurus has fully rendered
  setTimeout(() => {
    if (!document.querySelector('rag-chatbot')) {
      const chatbot = document.createElement('rag-chatbot');
      document.body.appendChild(chatbot);
    }
  }, 500); // Small delay to ensure Docusaurus page is fully loaded
});

// Also try to initialize when the window loads (backup initialization)
window.addEventListener('load', () => {
  setTimeout(() => {
    if (!document.querySelector('rag-chatbot')) {
      const chatbot = document.createElement('rag-chatbot');
      document.body.appendChild(chatbot);
    }
  }, 1000);
});