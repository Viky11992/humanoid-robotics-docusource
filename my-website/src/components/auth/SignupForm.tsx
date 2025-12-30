import React, { useState } from 'react';
import { useAuth } from './AuthContext';

interface FormData {
  email: string;
  password: string;
  name: string;
  softwareBackground: string;
  hardwareExperience: string;
  roboticsInterest: string;
  experienceLevel: 'beginner' | 'intermediate' | 'advanced';
  learningGoals: string[];
}

const SignupForm: React.FC = () => {
  const { register, completeOnboarding } = useAuth();
  const [formData, setFormData] = useState<FormData>({
    email: '',
    password: '',
    name: '',
    softwareBackground: '',
    hardwareExperience: '',
    roboticsInterest: '',
    experienceLevel: 'beginner',
    learningGoals: [],
  });
  const [currentStep, setCurrentStep] = useState(1); // 1: Account, 2: Background
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleMultiSelectChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const options = Array.from(e.target.selectedOptions);
    const values = options.map(option => option.value);
    setFormData(prev => ({ ...prev, learningGoals: values }));
  };

  const handleAccountSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      // First register the user account
      await register(formData.email, formData.password, formData.name);
      // Move to the next step
      setCurrentStep(2);
    } catch (err) {
      setError('Registration failed. Please try again.');
      console.error('Registration error:', err);
    } finally {
      setLoading(false);
    }
  };

  const handleBackgroundSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      // Complete the onboarding with background information
      await completeOnboarding({
        softwareBackground: formData.softwareBackground,
        hardwareExperience: formData.hardwareExperience,
        roboticsInterest: formData.roboticsInterest,
        experienceLevel: formData.experienceLevel,
        learningGoals: formData.learningGoals,
      });

      // Reset form and show success
      alert('Registration and onboarding completed successfully!');
      setFormData({
        email: '',
        password: '',
        name: '',
        softwareBackground: '',
        hardwareExperience: '',
        roboticsInterest: '',
        experienceLevel: 'beginner',
        learningGoals: [],
      });
      setCurrentStep(1);
    } catch (err) {
      setError('Onboarding failed. Please try again.');
      console.error('Onboarding error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <div className="auth-form">
        <h2>{currentStep === 1 ? 'Create Account' : 'Tell Us About Yourself'}</h2>

        {error && <div className="error-message">{error}</div>}

        {currentStep === 1 ? (
          <form onSubmit={handleAccountSubmit}>
            <div className="form-group">
              <label htmlFor="name">Full Name</label>
              <input
                type="text"
                id="name"
                name="name"
                value={formData.name}
                onChange={handleChange}
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="email">Email</label>
              <input
                type="email"
                id="email"
                name="email"
                value={formData.email}
                onChange={handleChange}
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="password">Password</label>
              <input
                type="password"
                id="password"
                name="password"
                value={formData.password}
                onChange={handleChange}
                required
                minLength={8}
              />
            </div>

            <button type="submit" disabled={loading}>
              {loading ? 'Creating Account...' : 'Create Account'}
            </button>
          </form>
        ) : (
          <form onSubmit={handleBackgroundSubmit}>
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
              <label htmlFor="learningGoals">Learning Goals (Select all that apply)</label>
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

            <div className="form-navigation">
              <button
                type="button"
                onClick={() => setCurrentStep(1)}
                disabled={loading}
              >
                Back
              </button>
              <button type="submit" disabled={loading}>
                {loading ? 'Saving...' : 'Complete Registration'}
              </button>
            </div>
          </form>
        )}

        <div className="step-indicator">
          Step {currentStep} of 2
        </div>
      </div>
    </div>
  );
};

export default SignupForm;