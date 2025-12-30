// API service for RAG Chatbot
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL ||
                     (process.env.NODE_ENV === 'production' ?
                      'https://your-railway-backend-production.up.railway.app' :
                      'http://127.0.0.1:8000');

class ApiService {
  constructor() {
    this.apiKey = process.env.REACT_APP_API_KEY || '';
  }

  // Set API key if not available in environment
  setApiKey(apiKey) {
    this.apiKey = apiKey;
  }

  // Make authenticated API request
  async makeRequest(endpoint, options = {}) {
    const url = `${API_BASE_URL}${endpoint}`;
    const defaultHeaders = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.apiKey}`
    };

    const config = {
      ...options,
      headers: {
        ...defaultHeaders,
        ...options.headers
      }
    };

    try {
      const response = await fetch(url, config);

      if (!response.ok) {
        throw new Error(`API request failed: ${response.status} ${response.statusText}`);
      }

      return await response.json();
    } catch (error) {
      console.error('API request error:', error);
      throw error;
    }
  }

  // Chat endpoint
  async chat(query, sessionId = null, selectedText = null) {
    const body = {
      question: query,  // Agent endpoint expects "question" instead of "query"
      selected_text: selectedText || null,
      context_restrict: !!selectedText  // Restrict context if selected text is provided
    };

    return this.makeRequest('/agent/ask', {
      method: 'POST',
      body: JSON.stringify(body)
    });
  }

  // Embeddings endpoint
  async generateEmbeddings(text, inputType = 'document') {
    const body = {
      text: text,
      input_type: inputType
    };

    return this.makeRequest('/embeddings', {
      method: 'POST',
      body: JSON.stringify(body)
    });
  }

  // Health check endpoint
  async healthCheck() {
    return this.makeRequest('/health', {
      method: 'GET'
    });
  }
}

export default new ApiService();