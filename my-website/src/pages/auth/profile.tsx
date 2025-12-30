import React from 'react';
import Layout from '@theme/Layout';
import { AuthProvider } from '../../components/auth/AuthContext';
import ProfileForm from '../../components/auth/ProfileForm';
import { useAuth } from '../../components/auth/AuthContext';

const ProfilePage: React.FC = () => {
  const { user, loading } = useAuth();

  if (loading) {
    return (
      <Layout title="Loading Profile" description="Loading your profile information">
        <div className="container margin-vert--lg">
          <div className="text--center">
            <h1>Loading...</h1>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Your Profile" description="Manage your profile and preferences">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            {!user ? (
              <div className="text--center padding-vert--md">
                <h1>Please Sign In</h1>
                <p>You need to be signed in to view your profile.</p>
                <a href="/auth">Sign In</a>
              </div>
            ) : (
              <div className="card">
                <div className="card__header">
                  <h2>Your Profile</h2>
                  <p>Manage your background information and learning preferences</p>
                </div>
                <div className="card__body">
                  <ProfileForm />
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </Layout>
  );
};

const ProfilePageWrapper: React.FC = () => (
  <AuthProvider>
    <ProfilePage />
  </AuthProvider>
);

export default ProfilePageWrapper;