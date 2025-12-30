from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime

class UserBase(BaseModel):
    email: str
    name: str

class UserCreate(UserBase):
    password: str

class UserUpdate(BaseModel):
    software_background: Optional[str] = None
    hardware_experience: Optional[str] = None
    robotics_interest: Optional[str] = None
    experience_level: Optional[str] = None  # 'beginner', 'intermediate', 'advanced'
    learning_goals: Optional[List[str]] = None

class UserInDB(UserBase):
    id: str
    hashed_password: str
    created_at: datetime
    software_background: Optional[str] = None
    hardware_experience: Optional[str] = None
    robotics_interest: Optional[str] = None
    experience_level: Optional[str] = None
    learning_goals: Optional[List[str]] = None

class UserResponse(UserBase):
    id: str
    created_at: datetime
    profile: Optional[dict] = None

    @classmethod
    def from_db(cls, user: UserInDB):
        # Check if any profile fields have values
        has_profile_data = any([
            user.software_background,
            user.hardware_experience,
            user.robotics_interest,
            user.experience_level,
            user.learning_goals and len(user.learning_goals) > 0
        ])

        profile = {
            "softwareBackground": user.software_background,
            "hardwareExperience": user.hardware_experience,
            "roboticsInterest": user.robotics_interest,
            "experienceLevel": user.experience_level,
            "learningGoals": user.learning_goals if user.learning_goals else []
        } if has_profile_data else None

        return cls(
            id=user.id,
            email=user.email,
            name=user.name,
            created_at=user.created_at,
            profile=profile
        )