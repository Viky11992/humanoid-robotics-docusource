// Configuration
const API_BASE_URL = 'http://localhost:8000';
const API_KEY = 'OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU'; // This should be configured securely in production

// DOM Elements
const chatHistory = document.getElementById('chatHistory');
const userInput = document.getElementById('userInput');
const sendButton = document.getElementById('sendButton');
const bookText = document.getElementById('bookText');
const selectedTextPreview = document.getElementById('selectedTextPreview');
const selectedTextContent = document.getElementById('selectedTextContent');
const clearSelectionButton = document.getElementById('clearSelection');
const sourcesPanel = document.getElementById('sourcesPanel');
const sourcesList = document.getElementById('sourcesList');

// State
let selectedText = '';

// Initialize the application
function init() {
    setupEventListeners();
    setupTextSelection();
}

// Set up event listeners
function setupEventListeners() {
    sendButton.addEventListener('click', handleSendMessage);
    userInput.addEventListener('keypress', function(e) {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            handleSendMessage();
        }
    });

    clearSelectionButton.addEventListener('click', clearSelection);
}

// Set up text selection functionality
function setupTextSelection() {
    bookText.addEventListener('mouseup', function() {
        const selection = window.getSelection();
        if (selection.toString().trim() !== '') {
            selectedText = selection.toString().trim();
            showSelectedTextPreview(selectedText);
        }
    });
}

// Show selected text preview
function showSelectedTextPreview(text) {
    selectedTextContent.textContent = text;
    selectedTextPreview.style.display = 'block';
}

// Clear selected text
function clearSelection() {
    selectedText = '';
    selectedTextPreview.style.display = 'none';
    selectedTextContent.textContent = '';
    window.getSelection().removeAllRanges();
}

// Handle sending a message
function handleSendMessage() {
    const message = userInput.value.trim();
    if (!message) return;

    // Add user message to chat
    addMessageToChat(message, 'user');

    // Clear input
    userInput.value = '';

    // Clear selection after sending if desired
    // clearSelection();

    // Send to API
    sendMessageToAPI(message, selectedText);
}

// Add message to chat history
function addMessageToChat(message, sender) {
    const messageDiv = document.createElement('div');
    messageDiv.className = `message ${sender}`;

    const contentDiv = document.createElement('div');
    contentDiv.className = 'message-content';
    contentDiv.textContent = message;

    messageDiv.appendChild(contentDiv);
    chatHistory.appendChild(messageDiv);

    // Scroll to bottom
    chatHistory.scrollTop = chatHistory.scrollHeight;
}

// Send message to API
async function sendMessageToAPI(question, selectedText) {
    try {
        const response = await fetch(`${API_BASE_URL}/agent/ask`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'x-api-key': API_KEY
            },
            body: JSON.stringify({
                question: question,
                selected_text: selectedText || null
            })
        });

        if (!response.ok) {
            throw new Error(`API error: ${response.status}`);
        }

        const data = await response.json();

        // Add bot response to chat
        addMessageToChat(data.answer, 'bot');

        // Show sources if available
        if (data.sources && data.sources.length > 0) {
            showSources(data.sources);
        }
    } catch (error) {
        console.error('Error sending message:', error);
        addMessageToChat('Sorry, there was an error processing your request. Please try again.', 'bot');
    }
}

// Show sources used in the response
function showSources(sources) {
    if (sources && sources.length > 0) {
        sourcesList.innerHTML = '';

        sources.forEach(source => {
            const li = document.createElement('li');
            li.textContent = source;
            sourcesList.appendChild(li);
        });

        sourcesPanel.style.display = 'block';
    } else {
        sourcesPanel.style.display = 'none';
    }
}

// Initialize the app when DOM is loaded
document.addEventListener('DOMContentLoaded', init);