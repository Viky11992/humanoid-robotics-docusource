import React from 'react';
import Layout from '@theme/Layout';
import { AuthProvider } from '../../components/auth/AuthContext';
import SignupForm from '../../components/auth/SignupForm';
import { useAuth } from '../../components/auth/AuthContext';

const SignupPage: React.FC = () => {
  const { user } = useAuth();

  return (
    <Layout title="Sign Up" description="Create an account for personalized learning">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            {user ? (
              <div className="text--center padding-vert--md">
                <h1>Welcome, {user.name}!</h1>
                <p>You are already signed in.</p>
                <a href="/">Go to Home</a>
              </div>
            ) : (
              <div className="card">
                <div className="card__header">
                  <h2>Create Your Account</h2>
                  <p>Sign up to get personalized content based on your background</p>
                </div>
                <div className="card__body">
                  <SignupForm />
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </Layout>
  );
};

const SignupPageWrapper: React.FC = () => (
  <AuthProvider>
    <SignupPage />
  </AuthProvider>
);

export default SignupPageWrapper;