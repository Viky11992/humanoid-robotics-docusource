import requests
import json

def test_search_capability():
    print("Testing if the system can search in the ingested documents...")
    headers = {
        "Content-Type": "application/json",
        "Authorization": "Bearer OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU"
    }

    # Test with a query that should match the ingested content
    payload = {
        "question": "What is Physical AI?",
        "selected_text": None,
        "context_restrict": False
    }

    try:
        response = requests.post(
            "http://127.0.0.1:8000/api/v1/agent/ask",
            headers=headers,
            data=json.dumps(payload)
        )
        print(f"Status: {response.status_code}")
        if response.status_code == 200:
            print(f"Success! Response: {response.text}")
        else:
            print(f"Error response: {response.text}")

            # Let's also try the chat endpoint
            print("\nTrying the /api/v1/chat endpoint...")
            chat_payload = {
                "query": "What is Physical AI?",
                "session_id": "test-session-123"
            }
            chat_response = requests.post(
                "http://127.0.0.1:8000/api/v1/chat",
                headers=headers,
                data=json.dumps(chat_payload)
            )
            print(f"Chat endpoint status: {chat_response.status_code}")
            print(f"Chat response: {chat_response.text}")
    except Exception as e:
        print(f"Request error: {e}")

def test_direct_qdrant():
    print("\nTesting Qdrant connection directly through the service...")
    try:
        import sys
        import os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

        from src.services.qdrant_service import qdrant_service
        print(f"Qdrant client: {qdrant_service.client}")
        print(f"Collection name: {qdrant_service.collection_name}")

        # Try to get collection info
        if qdrant_service.client is not None:
            collections = qdrant_service.client.get_collections()
            print(f"Collections: {collections}")
        else:
            print("Qdrant client is None - using mock mode")
            print(f"Mock storage length: {len(qdrant_service._mock_storage)}")
    except Exception as e:
        print(f"Direct Qdrant test error: {e}")

if __name__ == "__main__":
    test_search_capability()
    test_direct_qdrant()