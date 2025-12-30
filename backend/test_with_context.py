import requests
import json

def test_with_context():
    print("Testing with provided context (bypasses Qdrant)...")
    headers = {
        "Content-Type": "application/json",
        "Authorization": "Bearer OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU"
    }

    # Test with context restriction (this bypasses Qdrant vector search)
    payload = {
        "question": "What is Physical AI?",
        "selected_text": "Physical AI is an approach to robotics that emphasizes the importance of physical interaction with the environment for intelligent behavior. It combines principles from physics, machine learning, and robotics to create systems that can understand and interact with the physical world effectively.",
        "context_restrict": True  # This should bypass Qdrant search
    }

    try:
        response = requests.post(
            "http://127.0.0.1:8000/api/v1/agent/ask",
            headers=headers,
            data=json.dumps(payload)
        )
        print(f"Status: {response.status_code}")
        print(f"Response: {response.text}")
    except Exception as e:
        print(f"Error: {e}")

def test_chat_endpoint():
    print("\nTesting the /api/v1/chat endpoint (might work differently)...")
    headers = {
        "Content-Type": "application/json",
        "Authorization": "Bearer OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU"
    }

    payload = {
        "query": "What is Physical AI?",
        "session_id": "test-session"
    }

    try:
        response = requests.post(
            "http://127.0.0.1:8000/api/v1/chat",
            headers=headers,
            data=json.dumps(payload)
        )
        print(f"Chat endpoint status: {response.status_code}")
        print(f"Chat response: {response.text}")
    except Exception as e:
        print(f"Chat endpoint error: {e}")

if __name__ == "__main__":
    test_with_context()
    test_chat_endpoint()