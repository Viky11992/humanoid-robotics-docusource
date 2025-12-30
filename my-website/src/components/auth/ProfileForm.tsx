import React, { useState, useEffect } from 'react';
import { useAuth } from './AuthContext';

const ProfileForm: React.FC = () => {
  const { user, loading, updateUserProfile } = useAuth();
  const [formData, setFormData] = useState({
    softwareBackground: user?.profile?.softwareBackground || '',
    hardwareExperience: user?.profile?.hardwareExperience || '',
    roboticsInterest: user?.profile?.roboticsInterest || '',
    experienceLevel: user?.profile?.experienceLevel || 'beginner',
    learningGoals: user?.profile?.learningGoals || [],
  });
  const [isEditing, setIsEditing] = useState(false);
  const [saveLoading, setSaveLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (user) {
      setFormData({
        softwareBackground: user.profile?.softwareBackground || '',
        hardwareExperience: user.profile?.hardwareExperience || '',
        roboticsInterest: user?.profile?.roboticsInterest || '',
        experienceLevel: user?.profile?.experienceLevel || 'beginner',
        learningGoals: user?.profile?.learningGoals || [],
      });
    }
  }, [user]);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleMultiSelectChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const options = Array.from(e.target.selectedOptions);
    const values = options.map(option => option.value);
    setFormData(prev => ({ ...prev, learningGoals: values }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setSaveLoading(true);
    setError(null);

    try {
      await updateUserProfile({
        softwareBackground: formData.softwareBackground,
        hardwareExperience: formData.hardwareExperience,
        roboticsInterest: formData.roboticsInterest,
        experienceLevel: formData.experienceLevel as any,
        learningGoals: formData.learningGoals,
      });

      setIsEditing(false);
      alert('Profile updated successfully!');
    } catch (err) {
      setError('Failed to update profile. Please try again.');
      console.error('Profile update error:', err);
    } finally {
      setSaveLoading(false);
    }
  };

  if (loading) {
    return <div>Loading...</div>;
  }

  if (!user) {
    return <div>Please sign in to view your profile.</div>;
  }

  return (
    <div className="profile-container">
      <h2>User Profile</h2>

      <div className="profile-info">
        <h3>Account Information</h3>
        <p><strong>Name:</strong> {user.name}</p>
        <p><strong>Email:</strong> {user.email}</p>
        <p><strong>Member Since:</strong> {new Date(user.createdAt).toLocaleDateString()}</p>
      </div>

      <div className="profile-form">
        <h3>Background Information</h3>

        {isEditing ? (
          <form onSubmit={handleSubmit}>
            {error && <div className="error-message">{error}</div>}

            <div className="form-group">
              <label htmlFor="softwareBackground">Software Background</label>
              <textarea
                id="softwareBackground"
                name="softwareBackground"
                value={formData.softwareBackground}
                onChange={handleChange}
                placeholder="Describe your software development experience, programming languages you know, etc."
                rows={4}
              />
            </div>

            <div className="form-group">
              <label htmlFor="hardwareExperience">Hardware Experience</label>
              <textarea
                id="hardwareExperience"
                name="hardwareExperience"
                value={formData.hardwareExperience}
                onChange={handleChange}
                placeholder="Describe your experience with electronics, robotics, hardware prototyping, etc."
                rows={4}
              />
            </div>

            <div className="form-group">
              <label htmlFor="roboticsInterest">Interest in Robotics</label>
              <textarea
                id="roboticsInterest"
                name="roboticsInterest"
                value={formData.roboticsInterest}
                onChange={handleChange}
                placeholder="What interests you most about robotics? What applications appeal to you?"
                rows={3}
              />
            </div>

            <div className="form-group">
              <label htmlFor="experienceLevel">Experience Level</label>
              <select
                id="experienceLevel"
                name="experienceLevel"
                value={formData.experienceLevel}
                onChange={handleChange}
              >
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className="form-group">
              <label htmlFor="learningGoals">Learning Goals</label>
              <select
                id="learningGoals"
                name="learningGoals"
                multiple
                value={formData.learningGoals}
                onChange={handleMultiSelectChange}
              >
                <option value="ros_mastery">Master ROS 2</option>
                <option value="simulation_expert">Simulation Expert</option>
                <option value="ai_integration">AI Integration</option>
                <option value="hardware_control">Hardware Control</option>
                <option value="computer_vision">Computer Vision</option>
                <option value="motion_planning">Motion Planning</option>
                <option value="humanoid_design">Humanoid Robot Design</option>
                <option value="research_application">Research Applications</option>
              </select>
            </div>

            <div className="form-actions">
              <button type="button" onClick={() => setIsEditing(false)}>Cancel</button>
              <button type="submit" disabled={saveLoading}>
                {saveLoading ? 'Saving...' : 'Save Changes'}
              </button>
            </div>
          </form>
        ) : (
          <div className="profile-display">
            <div className="profile-field">
              <strong>Software Background:</strong>
              <p>{formData.softwareBackground || 'Not provided'}</p>
            </div>

            <div className="profile-field">
              <strong>Hardware Experience:</strong>
              <p>{formData.hardwareExperience || 'Not provided'}</p>
            </div>

            <div className="profile-field">
              <strong>Robotics Interest:</strong>
              <p>{formData.roboticsInterest || 'Not provided'}</p>
            </div>

            <div className="profile-field">
              <strong>Experience Level:</strong>
              <p>{formData.experienceLevel || 'Not provided'}</p>
            </div>

            <div className="profile-field">
              <strong>Learning Goals:</strong>
              <p>
                {formData.learningGoals.length > 0
                  ? formData.learningGoals.join(', ')
                  : 'Not provided'}
              </p>
            </div>

            <button onClick={() => setIsEditing(true)}>Edit Profile</button>
          </div>
        )}
      </div>
    </div>
  );
};

export default ProfileForm;