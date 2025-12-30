import requests
import json

def test_api():
    # Test the health endpoint first
    print("Testing health endpoint...")
    try:
        health_response = requests.get("http://127.0.0.1:8000/api/v1/health")
        print(f"Health endpoint status: {health_response.status_code}")
        print(f"Health response: {health_response.text}")
    except Exception as e:
        print(f"Health endpoint error: {e}")

    # Test the agent endpoint
    print("\nTesting agent endpoint...")
    headers = {
        "Content-Type": "application/json",
        "Authorization": "Bearer OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU"
    }

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
        print(f"Agent endpoint status: {response.status_code}")
        print(f"Agent response: {response.text}")
    except Exception as e:
        print(f"Agent endpoint error: {e}")
        print("Make sure the backend server is running on port 8000")

if __name__ == "__main__":
    test_api()