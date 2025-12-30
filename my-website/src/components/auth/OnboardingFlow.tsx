import React, { useState } from 'react';
import { useAuth } from './AuthContext';

interface OnboardingData {
  softwareBackground: string;
  hardwareExperience: string;
  roboticsInterest: string;
  experienceLevel: 'beginner' | 'intermediate' | 'advanced';
  learningGoals: string[];
}

const OnboardingFlow: React.FC = () => {
  const { user, completeOnboarding, loading } = useAuth();
  const [step, setStep] = useState(1); // 1-5 for each onboarding step
  const [data, setData] = useState<OnboardingData>({
    softwareBackground: user?.profile?.softwareBackground || '',
    hardwareExperience: user?.profile?.hardwareExperience || '',
    roboticsInterest: user?.profile?.roboticsInterest || '',
    experienceLevel: user?.profile?.experienceLevel || 'beginner',
    learningGoals: user?.profile?.learningGoals || [],
  });
  const [error, setError] = useState<string | null>(null);

  const totalSteps = 5;

  const handleInputChange = (field: keyof OnboardingData, value: any) => {
    setData(prev => ({ ...prev, [field]: value }));
  };

  const handleMultiSelect = (field: keyof OnboardingData, value: string) => {
    setData(prev => {
      const currentValues = [...prev[field] as string[]];
      if (currentValues.includes(value)) {
        return { ...prev, [field]: currentValues.filter(v => v !== value) };
      } else {
        return { ...prev, [field]: [...currentValues, value] };
      }
    });
  };

  const nextStep = () => {
    if (step < totalSteps) {
      setStep(step + 1);
    }
  };

  const prevStep = () => {
    if (step > 1) {
      setStep(step - 1);
    }
  };

  const handleSubmit = async () => {
    try {
      await completeOnboarding(data);
      alert('Welcome! Your profile has been updated.');
      // Redirect to dashboard or appropriate page
      window.location.href = '/';
    } catch (err) {
      setError('Failed to complete onboarding. Please try again.');
      console.error('Onboarding error:', err);
    }
  };

  // Step 1: Software Background
  if (step === 1) {
    return (
      <div className="onboarding-container">
        <div className="onboarding-header">
          <h2>Software Background</h2>
          <p className="step-indicator">Step {step} of {totalSteps}</p>
        </div>

        <div className="onboarding-content">
          <p>Tell us about your software development experience:</p>
          <textarea
            value={data.softwareBackground}
            onChange={(e) => handleInputChange('softwareBackground', e.target.value)}
            placeholder="Describe your programming experience, languages you know, robotics software experience, etc."
            rows={6}
          />
        </div>

        <div className="onboarding-actions">
          <div className="progress-bar">
            <div
              className="progress-fill"
              style={{ width: `${(step / totalSteps) * 100}%` }}
            ></div>
          </div>
          <div className="action-buttons">
            <button onClick={nextStep} disabled={!data.softwareBackground.trim()}>
              Next
            </button>
          </div>
        </div>
      </div>
    );
  }

  // Step 2: Hardware Experience
  if (step === 2) {
    return (
      <div className="onboarding-container">
        <div className="onboarding-header">
          <h2>Hardware Experience</h2>
          <p className="step-indicator">Step {step} of {totalSteps}</p>
        </div>

        <div className="onboarding-content">
          <p>Describe your experience with hardware:</p>
          <textarea
            value={data.hardwareExperience}
            onChange={(e) => handleInputChange('hardwareExperience', e.target.value)}
            placeholder="Tell us about your experience with electronics, robotics, hardware prototyping, microcontrollers, etc."
            rows={6}
          />
        </div>

        <div className="onboarding-actions">
          <div className="progress-bar">
            <div
              className="progress-fill"
              style={{ width: `${(step / totalSteps) * 100}%` }}
            ></div>
          </div>
          <div className="action-buttons">
            <button onClick={prevStep}>Previous</button>
            <button onClick={nextStep} disabled={!data.hardwareExperience.trim()}>
              Next
            </button>
          </div>
        </div>
      </div>
    );
  }

  // Step 3: Robotics Interest
  if (step === 3) {
    return (
      <div className="onboarding-container">
        <div className="onboarding-header">
          <h2>Robotics Interest</h2>
          <p className="step-indicator">Step {step} of {totalSteps}</p>
        </div>

        <div className="onboarding-content">
          <p>What interests you most about robotics?</p>
          <textarea
            value={data.roboticsInterest}
            onChange={(e) => handleInputChange('roboticsInterest', e.target.value)}
            placeholder="What applications of robotics excite you? What problems do you want to solve?"
            rows={6}
          />
        </div>

        <div className="onboarding-actions">
          <div className="progress-bar">
            <div
              className="progress-fill"
              style={{ width: `${(step / totalSteps) * 100}%` }}
            ></div>
          </div>
          <div className="action-buttons">
            <button onClick={prevStep}>Previous</button>
            <button onClick={nextStep} disabled={!data.roboticsInterest.trim()}>
              Next
            </button>
          </div>
        </div>
      </div>
    );
  }

  // Step 4: Experience Level
  if (step === 4) {
    return (
      <div className="onboarding-container">
        <div className="onboarding-header">
          <h2>Experience Level</h2>
          <p className="step-indicator">Step {step} of {totalSteps}</p>
        </div>

        <div className="onboarding-content">
          <p>How would you rate your current experience level?</p>
          <div className="radio-group">
            {(['beginner', 'intermediate', 'advanced'] as const).map(level => (
              <label key={level} className="radio-option">
                <input
                  type="radio"
                  name="experienceLevel"
                  checked={data.experienceLevel === level}
                  onChange={() => handleInputChange('experienceLevel', level)}
                />
                <span className="radio-label">
                  <strong>{level.charAt(0).toUpperCase() + level.slice(1)}</strong>
                  <span className="radio-description">
                    {level === 'beginner' && 'Just starting out with robotics'}
                    {level === 'intermediate' && 'Some experience with robotics concepts'}
                    {level === 'advanced' && 'Significant experience with robotics systems'}
                  </span>
                </span>
              </label>
            ))}
          </div>
        </div>

        <div className="onboarding-actions">
          <div className="progress-bar">
            <div
              className="progress-fill"
              style={{ width: `${(step / totalSteps) * 100}%` }}
            ></div>
          </div>
          <div className="action-buttons">
            <button onClick={prevStep}>Previous</button>
            <button onClick={nextStep}>Next</button>
          </div>
        </div>
      </div>
    );
  }

  // Step 5: Learning Goals
  if (step === 5) {
    const goals = [
      { id: 'ros_mastery', label: 'Master ROS 2 (Robot Operating System)' },
      { id: 'simulation_expert', label: 'Become a simulation expert (Gazebo, Unity)' },
      { id: 'ai_integration', label: 'Integrate AI with robotics systems' },
      { id: 'hardware_control', label: 'Master hardware control systems' },
      { id: 'computer_vision', label: 'Apply computer vision to robotics' },
      { id: 'motion_planning', label: 'Learn motion planning algorithms' },
      { id: 'humanoid_design', label: 'Design humanoid robots' },
      { id: 'research_application', label: 'Apply robotics to research problems' },
    ];

    return (
      <div className="onboarding-container">
        <div className="onboarding-header">
          <h2>Learning Goals</h2>
          <p className="step-indicator">Step {step} of {totalSteps} (Final Step)</p>
        </div>

        <div className="onboarding-content">
          <p>What are your primary learning goals? Select all that apply:</p>
          <div className="checkbox-group">
            {goals.map(goal => (
              <label key={goal.id} className="checkbox-option">
                <input
                  type="checkbox"
                  checked={data.learningGoals.includes(goal.id)}
                  onChange={() => handleMultiSelect('learningGoals', goal.id)}
                />
                <span className="checkbox-label">{goal.label}</span>
              </label>
            ))}
          </div>
        </div>

        <div className="onboarding-actions">
          <div className="progress-bar">
            <div
              className="progress-fill"
              style={{ width: `${(step / totalSteps) * 100}%` }}
            ></div>
          </div>
          <div className="action-buttons">
            <button onClick={prevStep}>Previous</button>
            <button onClick={handleSubmit} disabled={data.learningGoals.length === 0}>
              Complete Setup
            </button>
          </div>
        </div>
      </div>
    );
  }

  return null;
};

export default OnboardingFlow;