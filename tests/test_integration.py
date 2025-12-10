import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import sys
import os

# Add backend to the path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.main import app

client = TestClient(app)

def test_full_rag_flow_with_selected_text():
    """Integration test for the full RAG flow with selected text."""
    with patch('backend.rag_tool.rag_tool') as mock_rag_tool, \
         patch('backend.logging_service.log_query') as mock_log_query:

        # Mock the RAG tool response
        mock_rag_tool.synthesize_answer.return_value = {
            "answer": "Based on the provided context, the main theme of the book is artificial intelligence.",
            "sources": ["user_selected_text"]
        }

        # Mock the logging service
        mock_log_query.return_value = 1  # Return a log ID

        # Make request with selected text
        response = client.post(
            "/agent/ask",
            json={
                "question": "What is the main theme of the book?",
                "selected_text": "The book explores the impact of artificial intelligence on modern society and its potential future developments."
            },
            headers={"x-api-key": "your_api_key_for_authentication"}
        )

        # Verify response
        assert response.status_code == 200
        data = response.json()
        assert data["answer"] == "Based on the provided context, the main theme of the book is artificial intelligence."
        assert "user_selected_text" in data["sources"]

        # Verify that logging was called
        mock_log_query.assert_called_once()


def test_full_rag_flow_without_selected_text():
    """Integration test for the full RAG flow without selected text (will use vector search in full implementation)."""
    with patch('backend.rag_tool.rag_tool') as mock_rag_tool, \
         patch('backend.logging_service.log_query') as mock_log_query:

        # Mock the RAG tool response (in a full implementation, this would trigger vector search)
        mock_rag_tool.synthesize_answer.return_value = {
            "answer": "The main theme of the book appears to be related to technology based on the available information.",
            "sources": []
        }

        # Mock the logging service
        mock_log_query.return_value = 2  # Return a log ID

        # Make request without selected text
        response = client.post(
            "/agent/ask",
            json={
                "question": "What is the main theme of the book?"
            },
            headers={"x-api-key": "your_api_key_for_authentication"}
        )

        # Verify response
        assert response.status_code == 200
        data = response.json()
        assert "technology" in data["answer"].lower()
        assert data["sources"] == []

        # Verify that logging was called
        mock_log_query.assert_called_once()


def test_api_key_authentication_integration():
    """Integration test for API key authentication."""
    # Try without API key
    response_without_key = client.post(
        "/agent/ask",
        json={
            "question": "What is the main theme of the book?",
            "selected_text": "Test context"
        }
    )
    assert response_without_key.status_code == 403

    # Try with wrong API key
    response_wrong_key = client.post(
        "/agent/ask",
        json={
            "question": "What is the main theme of the book?",
            "selected_text": "Test context"
        },
        headers={"x-api-key": "wrong_api_key"}
    )
    assert response_wrong_key.status_code == 403

    # Try with correct API key
    response_correct_key = client.post(
        "/agent/ask",
        json={
            "question": "What is the main theme of the book?",
            "selected_text": "Test context"
        },
        headers={"x-api-key": "your_api_key_for_authentication"}
    )
    assert response_correct_key.status_code == 200


if __name__ == "__main__":
    pytest.main()