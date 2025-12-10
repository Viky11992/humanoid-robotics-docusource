import pytest
from unittest.mock import patch, MagicMock
import sys
import os

# Add backend to the path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.agent import GoogleGeminiAgent

def test_google_agent_initialization():
    """Test that the Google Gemini Agent initializes properly."""
    with patch.dict(os.environ, {"GOOGLE_API_KEY": "test_key"}):
        agent = GoogleGeminiAgent()
        assert agent is not None
        assert "rag_answer_synthesis" in agent.tools


def test_agent_ask_method():
    """Test the agent's ask method."""
    with patch.dict(os.environ, {"GOOGLE_API_KEY": "test_key"}), \
         patch('backend.rag_tool.rag_tool') as mock_rag_tool:

        # Mock the RAG tool response
        mock_rag_tool.synthesize_answer.return_value = {
            "answer": "This is a test answer.",
            "sources": ["source1"]
        }

        agent = GoogleGeminiAgent()
        result = agent.ask("What is the main theme?", "Sample context text.")

        assert "answer" in result
        assert "sources" in result
        assert result["answer"] == "This is a test answer."
        assert "source1" in result["sources"]


def test_agent_context_only_response():
    """Test that the agent ensures context-only responses."""
    with patch.dict(os.environ, {"GOOGLE_API_KEY": "test_key"}), \
         patch('backend.agent.genai.GenerativeModel') as mock_model:

        # Mock the model response
        mock_response = MagicMock()
        mock_response.text = "This is an answer based on context."
        mock_model.return_value.generate_content.return_value = mock_response

        agent = GoogleGeminiAgent()
        result = agent.ensure_context_only_response(
            "What is the main theme?",
            "Sample context text."
        )

        assert result == "This is an answer based on context."
        # Verify that the model was called
        mock_model.return_value.generate_content.assert_called_once()


def test_agent_tool_functions():
    """Test the agent's tool functions."""
    with patch.dict(os.environ, {"GOOGLE_API_KEY": "test_key"}), \
         patch('backend.vector_search_tool.vector_search_tool') as mock_vector_search:

        # Mock vector search result
        mock_vector_search.search.return_value = [
            {"id": "chunk_1", "text": "Sample text", "score": 0.9}
        ]

        agent = GoogleGeminiAgent()
        result = agent.tool_functions["vector_search"]("test query", 5)

        assert len(result) == 1
        assert result[0]["id"] == "chunk_1"


def test_agent_without_context():
    """Test the agent's behavior when no context is provided."""
    with patch.dict(os.environ, {"GOOGLE_API_KEY": "test_key"}), \
         patch('backend.agent.genai.GenerativeModel') as mock_model:

        # Mock the model response
        mock_response = MagicMock()
        mock_response.text = "No context provided for this question."
        mock_model.return_value.generate_content.return_value = mock_response

        agent = GoogleGeminiAgent()
        result = agent.ensure_context_only_response("What is the main theme?")

        assert "No context provided" in result


if __name__ == "__main__":
    pytest.main()