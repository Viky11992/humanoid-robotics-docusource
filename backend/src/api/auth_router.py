from fastapi import APIRouter, HTTPException, Depends, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import jwt
import os
from datetime import datetime, timedelta
from passlib.context import CryptContext
import uuid

# Import database models and services
from ..models.user import UserInDB, UserCreate, UserUpdate, UserResponse
from ..database import get_user_by_email, create_user, update_user_profile
from ..utils.security import create_access_token, verify_token, get_password_hash, verify_password

router = APIRouter(prefix="/auth", tags=["Authentication"])

security = HTTPBearer()

SECRET_KEY = os.getenv("OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU", "your-secret-key-change-in-production")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

class UserRegister(BaseModel):
    email: str
    password: str
    name: str

class UserLogin(BaseModel):
    email: str
    password: str

class UserProfileUpdate(BaseModel):
    softwareBackground: Optional[str] = None
    hardwareExperience: Optional[str] = None
    roboticsInterest: Optional[str] = None
    experienceLevel: Optional[str] = None  # 'beginner', 'intermediate', 'advanced'
    learningGoals: Optional[List[str]] = None

class OnboardingData(BaseModel):
    softwareBackground: Optional[str] = None
    hardwareExperience: Optional[str] = None
    roboticsInterest: Optional[str] = None
    experienceLevel: Optional[str] = None  # 'beginner', 'intermediate', 'advanced'
    learningGoals: Optional[List[str]] = None

@router.post("/register")
async def register(user_data: UserRegister):
    # Check if user already exists
    existing_user = await get_user_by_email(user_data.email)
    if existing_user:
        raise HTTPException(status_code=400, detail="User with this email already exists")

    # Validate password length for bcrypt
    if len(user_data.password.encode('utf-8')) > 72:
        raise HTTPException(status_code=400, detail="Password must not exceed 72 bytes")

    # Hash the password
    hashed_password = get_password_hash(user_data.password)

    # Create new user
    new_user = await create_user(
        email=user_data.email,
        hashed_password=hashed_password,
        name=user_data.name
    )

    # Create access token
    access_token = create_access_token(data={"sub": new_user.email})

    # Convert to UserResponse for proper serialization
    user_response = UserResponse.from_db(new_user)

    return {
        "token": access_token,
        "user": user_response
    }

@router.post("/login")
async def login(user_data: UserLogin):
    # Validate password length for bcrypt
    if len(user_data.password.encode('utf-8')) > 72:
        raise HTTPException(status_code=400, detail="Password must not exceed 72 bytes")

    # Get user from database
    user = await get_user_by_email(user_data.email)
    if not user or not verify_password(user_data.password, user.hashed_password):
        raise HTTPException(status_code=401, detail="Incorrect email or password")

    # Create access token
    access_token = create_access_token(data={"sub": user.email})

    # Convert to UserResponse for proper serialization
    user_response = UserResponse.from_db(user)

    return {
        "token": access_token,
        "user": user_response
    }

@router.get("/me")
async def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)):
    token = credentials.credentials
    email = verify_token(token)

    if not email:
        raise HTTPException(status_code=401, detail="Invalid or expired token")

    user = await get_user_by_email(email)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")

    # Convert to UserResponse for proper serialization
    user_response = UserResponse.from_db(user)
    return user_response

@router.put("/profile")
async def update_profile(
    profile_data: UserProfileUpdate,
    credentials: HTTPAuthorizationCredentials = Depends(security)
):
    token = credentials.credentials
    email = verify_token(token)

    if not email:
        raise HTTPException(status_code=401, detail="Invalid or expired token")

    # Update user profile
    updated_user = await update_user_profile(
        email=email,
        software_background=profile_data.softwareBackground,
        hardware_experience=profile_data.hardwareExperience,
        robotics_interest=profile_data.roboticsInterest,
        experience_level=profile_data.experienceLevel,
        learning_goals=profile_data.learningGoals
    )

    if not updated_user:
        raise HTTPException(status_code=404, detail="User not found")

    # Convert to UserResponse for proper serialization
    user_response = UserResponse.from_db(updated_user)
    return user_response

@router.post("/onboarding")
async def complete_onboarding(
    onboarding_data: OnboardingData,
    credentials: HTTPAuthorizationCredentials = Depends(security)
):
    token = credentials.credentials
    email = verify_token(token)

    if not email:
        raise HTTPException(status_code=401, detail="Invalid or expired token")

    # Update user profile with onboarding data
    updated_user = await update_user_profile(
        email=email,
        software_background=onboarding_data.softwareBackground,
        hardware_experience=onboarding_data.hardwareExperience,
        robotics_interest=onboarding_data.roboticsInterest,
        experience_level=onboarding_data.experienceLevel,
        learning_goals=onboarding_data.learningGoals
    )

    if not updated_user:
        raise HTTPException(status_code=404, detail="User not found")

    # Convert to UserResponse for proper serialization
    user_response = UserResponse.from_db(updated_user)
    return user_response

@router.post("/logout")
async def logout(credentials: HTTPAuthorizationCredentials = Depends(security)):
    # In a real implementation, you might add the token to a blacklist
    # For now, we just return a success message
    return {"message": "Successfully logged out"}