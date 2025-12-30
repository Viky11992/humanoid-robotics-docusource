import requests
import json

def test_auth_endpoints():
    base_url = "http://127.0.0.1:8000/api/v1"

    # Test registration
    print("Testing registration endpoint...")
    register_payload = {
        "email": "testuser@example.com",
        "password": "testpassword123",
        "name": "Test User"
    }

    try:
        register_response = requests.post(
            f"{base_url}/auth/register",
            headers={"Content-Type": "application/json"},
            data=json.dumps(register_payload)
        )
        print(f"Registration status: {register_response.status_code}")
        print(f"Registration response: {register_response.text}")

        if register_response.status_code == 200:
            response_data = register_response.json()
            token = response_data.get("token")
            print(f"Registration successful! Token: {token[:20]}..." if token else "No token in response")
        else:
            print("Registration failed!")

    except Exception as e:
        print(f"Registration error: {e}")

    # Test login
    print("\nTesting login endpoint...")
    login_payload = {
        "email": "testuser@example.com",
        "password": "testpassword123"
    }

    try:
        login_response = requests.post(
            f"{base_url}/auth/login",
            headers={"Content-Type": "application/json"},
            data=json.dumps(login_payload)
        )
        print(f"Login status: {login_response.status_code}")
        print(f"Login response: {login_response.text}")
    except Exception as e:
        print(f"Login error: {e}")

    # Test registration with existing user (should fail)
    print("\nTesting duplicate registration (should fail)...")
    try:
        duplicate_response = requests.post(
            f"{base_url}/auth/register",
            headers={"Content-Type": "application/json"},
            data=json.dumps(register_payload)
        )
        print(f"Duplicate registration status: {duplicate_response.status_code}")
        print(f"Duplicate registration response: {duplicate_response.text}")
    except Exception as e:
        print(f"Duplicate registration error: {e}")

if __name__ == "__main__":
    test_auth_endpoints()