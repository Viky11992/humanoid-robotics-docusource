import React from 'react';
import Layout from '@theme/Layout';
import { AuthProvider } from '../../components/auth/AuthContext';
import SignupForm from '../../components/auth/SignupForm';
import LoginForm from '../../components/auth/LoginForm';
import { useAuth } from '../../components/auth/AuthContext';

const AuthPage: React.FC = () => {
  const { user } = useAuth();

  return (
    <Layout title="Authentication" description="User authentication for personalized learning">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            {user ? (
              <div className="text--center padding-vert--md">
                <h1>Welcome back, {user.name}!</h1>
                <p>You are already signed in.</p>
                <a href="/">Go to Home</a>
              </div>
            ) : (
              <div className="card">
                <div className="card__header">
                  <h2>Sign In or Sign Up</h2>
                </div>
                <div className="card__body">
                  <LoginForm />
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </Layout>
  );
};

const AuthPageWrapper: React.FC = () => (
  <AuthProvider>
    <AuthPage />
  </AuthProvider>
);

export default AuthPageWrapper;