#!/usr/bin/env python3
"""
Test script to directly test the user creation function
"""
import asyncio
import sys
import os

# Add the backend/src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

from database import create_user
from utils.security import get_password_hash

async def test_create_user():
    try:
        print("Testing direct user creation...")

        # Test creating a user directly
        hashed_password = get_password_hash("testpassword123")
        new_user = await create_user(
            email="test@example.com",
            hashed_password=hashed_password,
            name="Test User"
        )

        print(f"User created successfully: {new_user}")
        print(f"User ID: {new_user.id}")
        print(f"User email: {new_user.email}")
        print(f"Created at: {new_user.created_at}")

    except Exception as e:
        print(f"Error creating user: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_create_user())