import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

interface User {
  id: string;
  email: string;
  name: string;
  profile?: {
    softwareBackground?: string;
    hardwareExperience?: string;
    roboticsInterest?: string;
    learningGoals?: string[];
    experienceLevel?: 'beginner' | 'intermediate' | 'advanced';
    preferredLearningPath?: string;
    timezone?: string;
  };
  createdAt: string;
}

interface AuthContextType {
  user: User | null;
  loading: boolean;
  login: (email: string, password: string) => Promise<void>;
  register: (email: string, password: string, name: string) => Promise<void>;
  logout: () => void;
  updateUserProfile: (profileData: Partial<User['profile']>) => Promise<void>;
  completeOnboarding: (backgroundData: Partial<User['profile']>) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

// Helper function to get API base URL from meta tag
const getApiBaseUrl = (): string => {
  const metaTag = document.querySelector('meta[name="api-base-url"]') as HTMLMetaElement;
  return metaTag ? metaTag.content : 'http://127.0.0.1:8000'; // fallback to default
};

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  // Check for existing session on component mount
  useEffect(() => {
    const checkSession = async () => {
      try {
        // Check if user has a session token in localStorage
        const token = localStorage.getItem('auth_token');
        if (token) {
          // Verify token with backend
          const response = await fetch(`${getApiBaseUrl()}/api/v1/auth/me`, {
            headers: {
              'Authorization': `Bearer ${token}`,
              'Content-Type': 'application/json',
            },
          });

          if (response.ok) {
            const userData = await response.json();
            setUser(userData);
          } else {
            // Token is invalid, clear it
            localStorage.removeItem('auth_token');
          }
        }
      } catch (error) {
        console.error('Error checking session:', error);
      } finally {
        setLoading(false);
      }
    };

    checkSession();
  }, []);

  const login = async (email: string, password: string) => {
    try {
      const response = await fetch(`${getApiBaseUrl()}/api/v1/auth/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      if (!response.ok) {
        throw new Error('Login failed');
      }

      const data = await response.json();
      localStorage.setItem('auth_token', data.token);
      setUser(data.user);
    } catch (error) {
      console.error('Login error:', error);
      throw error;
    }
  };

  const register = async (email: string, password: string, name: string) => {
    try {
      const response = await fetch(`${getApiBaseUrl()}/api/v1/auth/register`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password, name }),
      });

      if (!response.ok) {
        throw new Error('Registration failed');
      }

      const data = await response.json();
      localStorage.setItem('auth_token', data.token);
      setUser(data.user);
    } catch (error) {
      console.error('Registration error:', error);
      throw error;
    }
  };

  const logout = () => {
    localStorage.removeItem('auth_token');
    setUser(null);
  };

  const updateUserProfile = async (profileData: Partial<User['profile']>) => {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token || !user) {
        throw new Error('User not authenticated');
      }

      const response = await fetch(`${getApiBaseUrl()}/api/v1/auth/profile`, {
        method: 'PUT',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(profileData),
      });

      if (!response.ok) {
        throw new Error('Failed to update profile');
      }

      const updatedUser = await response.json();
      setUser(updatedUser);
    } catch (error) {
      console.error('Profile update error:', error);
      throw error;
    }
  };

  const completeOnboarding = async (backgroundData: Partial<User['profile']>) => {
    try {
      const token = localStorage.getItem('auth_token');
      if (!token || !user) {
        throw new Error('User not authenticated');
      }

      const response = await fetch(`${getApiBaseUrl()}/api/v1/auth/onboarding`, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(backgroundData),
      });

      if (!response.ok) {
        throw new Error('Failed to complete onboarding');
      }

      const updatedUser = await response.json();
      setUser(updatedUser);
    } catch (error) {
      console.error('Onboarding completion error:', error);
      throw error;
    }
  };

  const value = {
    user,
    loading,
    login,
    register,
    logout,
    updateUserProfile,
    completeOnboarding,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};