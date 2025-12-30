import requests
import json

# Test the agent endpoint
def test_agent_endpoint():
    url = "http://127.0.0.1:8000/api/v1/agent/ask"

    headers = {
        "Content-Type": "application/json",
        "Authorization": "Bearer OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU"  # Same key as in .env
    }

    payload = {
        "question": "What is Physical AI?",
        "selected_text": None,
        "context_restrict": False
    }

    try:
        response = requests.post(url, headers=headers, data=json.dumps(payload))
        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.text}")

        if response.status_code == 200:
            print("\n[SUCCESS] Agent endpoint is working correctly!")
            data = response.json()
            print(f"Answer: {data.get('answer', 'No answer field')}")
        else:
            print(f"\n[ERROR] Error: {response.status_code} - {response.text}")

    except requests.exceptions.ConnectionError:
        print("[ERROR] Connection error: Backend server may not be running")
        print("Make sure to start the backend with: uvicorn src.main:app --host 127.0.0.1 --port 8000")
    except Exception as e:
        print(f"[ERROR] Exception occurred: {str(e)}")

if __name__ == "__main__":
    test_agent_endpoint()