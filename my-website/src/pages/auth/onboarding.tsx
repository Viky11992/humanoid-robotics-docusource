import React from 'react';
import Layout from '@theme/Layout';
import { AuthProvider } from '../../components/auth/AuthContext';
import OnboardingFlow from '../../components/auth/OnboardingFlow';
import { useAuth } from '../../components/auth/AuthContext';

const OnboardingPage: React.FC = () => {
  const { user, loading } = useAuth();

  if (loading) {
    return (
      <Layout title="Onboarding" description="Complete your profile setup">
        <div className="container margin-vert--lg">
          <div className="text--center">
            <h1>Loading...</h1>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Welcome! Let's Get Started" description="Complete your profile to personalize your learning experience">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--10 col--offset-1">
            {!user ? (
              <div className="text--center padding-vert--md">
                <h1>Sign In Required</h1>
                <p>Please sign in to complete your onboarding.</p>
                <a href="/auth">Sign In</a>
              </div>
            ) : (
              <OnboardingFlow />
            )}
          </div>
        </div>
      </div>
    </Layout>
  );
};

const OnboardingPageWrapper: React.FC = () => (
  <AuthProvider>
    <OnboardingPage />
  </AuthProvider>
);

export default OnboardingPageWrapper;