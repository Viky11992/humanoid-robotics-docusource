import React, { useState, useEffect } from 'react';
import './RagChatbot.css';
import SelectedTextHandler from './SelectedTextHandler';
import apiService from '../services/api';

const RagChatbot = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState('');

  // Handle selected text from the page
  const handleTextSelected = (text) => {
    if (text.length > 0 && text.length < 2000) { // Limit selected text size
      setSelectedText(text);
    }
  };

  // Function to send message to backend
  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Use the API service to make the request
      const data = await apiService.chat(inputValue, sessionId, selectedText);

      if (!sessionId && data.session_id) {
        setSessionId(data.session_id);
      }

      const botMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'bot',
        sources: data.sources || []
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setInputValue('');
      setIsLoading(false);
      // Clear selected text after sending a message
      if (selectedText) {
        setSelectedText('');
      }
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className="rag-chatbot">
      <SelectedTextHandler onTextSelected={handleTextSelected} />
      <div className="chat-header">
        <h3>Book Assistant</h3>
        {selectedText && (
          <div className="selected-text-indicator">
            Context: "{selectedText.substring(0, 60)}{selectedText.length > 60 ? '...' : ''}"
          </div>
        )}
      </div>
      <div className="chat-messages">
        {messages.map((message) => (
          <div key={message.id} className={`message ${message.sender}`}>
            <div className="message-text">{message.text}</div>
            {message.sources && message.sources.length > 0 && (
              <div className="sources">
                <small>Sources:</small>
                <ul>
                  {message.sources.slice(0, 3).map((source, index) => (
                    <li key={index}>{source.content.substring(0, 100)}...</li>
                  ))}
                </ul>
              </div>
            )}
          </div>
        ))}
        {isLoading && (
          <div className="message bot">
            <div className="message-text">Thinking...</div>
          </div>
        )}
      </div>
      <div className="chat-input">
        <textarea
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder={selectedText
            ? "Ask about selected text..."
            : "Ask a question about the book..."}
          rows="2"
        />
        <button onClick={sendMessage} disabled={isLoading}>
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </div>
  );
};

export default RagChatbot;