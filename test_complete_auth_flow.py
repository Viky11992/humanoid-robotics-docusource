#!/usr/bin/env python3
"""
Test the complete authentication flow
"""
import asyncio
import sys
import os
import json
from pydantic import BaseModel

# Add the backend directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from src.api.auth_router import register, login
from src.database import get_user_by_email
from src.utils.security import verify_token

# Define the same models as in auth_router
class UserRegister(BaseModel):
    email: str
    password: str
    name: str

class UserLogin(BaseModel):
    email: str
    password: str

async def test_complete_auth_flow():
    print("Testing complete authentication flow...")

    # Test registration
    print("\n1. Testing registration...")
    try:
        user_data = UserRegister(email='testuser@example.com', password='test123', name='Test User')
        registration_result = await register(user_data)
        print(f"[SUCCESS] Registration successful!")
        print(f"  Token: {registration_result['token'][:20]}...")
        print(f"  User: {registration_result['user'].name}")

        # Verify user was created in database
        user_from_db = await get_user_by_email('testuser@example.com')
        if user_from_db:
            print(f"[SUCCESS] User found in database: {user_from_db.name}")
        else:
            print("[ERROR] User not found in database")
            return False

    except Exception as e:
        print(f"[ERROR] Registration failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    # Test login
    print("\n2. Testing login...")
    try:
        login_data = UserLogin(email='testuser@example.com', password='test123')
        login_result = await login(login_data)
        print(f"[SUCCESS] Login successful!")
        print(f"  Token: {login_result['token'][:20]}...")
        print(f"  User: {login_result['user'].name}")
    except Exception as e:
        print(f"[ERROR] Login failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    # Test token validation
    print("\n3. Testing token validation...")
    try:
        token = registration_result['token']
        email = verify_token(token)
        if email == 'testuser@example.com':
            print(f"[SUCCESS] Token validation successful: {email}")
        else:
            print(f"[ERROR] Token validation failed: {email}")
            return False
    except Exception as e:
        print(f"[ERROR] Token validation failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    print("\n[SUCCESS] All authentication tests passed!")
    return True

if __name__ == "__main__":
    success = asyncio.run(test_complete_auth_flow())
    if success:
        print("\n[SUCCESS] Authentication flow is working correctly!")
    else:
        print("\n[ERROR] Authentication flow has issues!")
        sys.exit(1)