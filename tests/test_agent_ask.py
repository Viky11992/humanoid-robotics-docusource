import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import sys
import os

# Add backend to the path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.main import app

client = TestClient(app)

def test_agent_ask_with_selected_text():
    """Test the /agent/ask endpoint with selected text provided."""
    with patch('backend.rag_tool.rag_tool') as mock_rag_tool:
        # Mock the RAG tool response
        mock_rag_tool.synthesize_answer.return_value = {
            "answer": "This is a test answer based on the provided context.",
            "sources": ["source1", "source2"]
        }

        # Make request with selected text
        response = client.post(
            "/agent/ask",
            json={
                "question": "What is the main theme of the book?",
                "selected_text": "The book is about artificial intelligence and its impact on society."
            },
            headers={"x-api-key": "your_api_key_for_authentication"}
        )

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert isinstance(data["sources"], list)
        assert data["answer"] == "This is a test answer based on the provided context."


def test_agent_ask_without_selected_text():
    """Test the /agent/ask endpoint without selected text."""
    with patch('backend.rag_tool.rag_tool') as mock_rag_tool:
        # Mock the RAG tool response
        mock_rag_tool.synthesize_answer.return_value = {
            "answer": "This is a test answer without specific context.",
            "sources": []
        }

        # Make request without selected text
        response = client.post(
            "/agent/ask",
            json={
                "question": "What is the main theme of the book?"
            },
            headers={"x-api-key": "your_api_key_for_authentication"}
        )

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert data["sources"] == []


def test_agent_ask_missing_question():
    """Test the /agent/ask endpoint with missing question."""
    response = client.post(
        "/agent/ask",
        json={},
        headers={"x-api-key": "your_api_key_for_authentication"}
    )

    # Should return validation error since question is required
    assert response.status_code == 422


def test_agent_ask_invalid_api_key():
    """Test the /agent/ask endpoint with invalid API key."""
    response = client.post(
        "/agent/ask",
        json={
            "question": "What is the main theme of the book?",
            "selected_text": "Test context"
        },
        headers={"x-api-key": "invalid_api_key"}
    )

    assert response.status_code == 403


def test_agent_ask_no_api_key():
    """Test the /agent/ask endpoint without API key."""
    response = client.post(
        "/agent/ask",
        json={
            "question": "What is the main theme of the book?",
            "selected_text": "Test context"
        }
    )

    assert response.status_code == 403


if __name__ == "__main__":
    pytest.main()