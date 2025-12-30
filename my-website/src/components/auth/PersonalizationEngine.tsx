import React, { createContext, useContext, ReactNode } from 'react';
import { useAuth } from './AuthContext';

interface PersonalizationContextType {
  getPersonalizedContent: (content: any, contentType: string) => any;
  getRecommendedPath: () => string[];
  getDifficultyAdjustedContent: (content: string, baseDifficulty: string) => string;
  getInterestAlignedExamples: (topic: string) => string[];
}

const PersonalizationContext = createContext<PersonalizationContextType | undefined>(undefined);

interface PersonalizationProviderProps {
  children: ReactNode;
}

export const PersonalizationProvider: React.FC<PersonalizationProviderProps> = ({ children }) => {
  const { user } = useAuth();

  // Get personalized content based on user profile
  const getPersonalizedContent = (content: any, contentType: string) => {
    if (!user?.profile) {
      // Return default content if no profile
      return content;
    }

    const profile = user.profile;

    // Adjust content based on experience level
    if (contentType === 'text' || contentType === 'markdown') {
      return adjustTextComplexity(content, profile.experienceLevel);
    }

    // Adjust examples based on user interests
    if (contentType === 'examples') {
      return filterExamplesByInterest(content, profile.roboticsInterest);
    }

    // Adjust exercises based on experience level
    if (contentType === 'exercises') {
      return adjustExerciseDifficulty(content, profile.experienceLevel);
    }

    return content;
  };

  // Generate recommended learning path based on user profile
  const getRecommendedPath = (): string[] => {
    if (!user?.profile) {
      // Default path for unprofiled users
      return [
        '/docs/intro/physical-ai-overview',
        '/docs/module-1-ros2/ros2-intro',
        '/docs/module-2-simulation/gazebo-intro',
        '/docs/module-3-isaac/isaac-intro',
        '/docs/module-4-vla/vla-intro'
      ];
    }

    const profile = user.profile;
    const recommendedPath: string[] = [];

    // Customize path based on experience level
    if (profile.experienceLevel === 'beginner') {
      recommendedPath.push(
        '/docs/intro/physical-ai-overview',
        '/docs/intro/embodied-intelligence',
        '/docs/module-1-ros2/ros2-intro',
        '/docs/module-1-ros2/nodes-topics-services'
      );
    } else {
      // More advanced users can start with core concepts
      recommendedPath.push(
        '/docs/module-1-ros2/ros2-intro',
        '/docs/module-2-simulation/gazebo-intro',
        '/docs/module-3-isaac/isaac-intro'
      );
    }

    // Customize based on specific interests
    if (profile.learningGoals?.includes('simulation_expert')) {
      recommendedPath.push(
        '/docs/module-2-simulation/physics-simulation',
        '/docs/module-2-simulation/unity-hri'
      );
    }

    if (profile.learningGoals?.includes('ai_integration')) {
      recommendedPath.push(
        '/docs/module-4-vla/vla-intro',
        '/docs/module-4-vla/voice-to-action'
      );
    }

    if (profile.learningGoals?.includes('ros_mastery')) {
      recommendedPath.push(
        '/docs/module-1-ros2/rclpy-bridge',
        '/docs/module-1-ros2/ros2-exercises'
      );
    }

    // Add general content
    recommendedPath.push(
      '/docs/hardware-setup/workstation-setup',
      '/docs/appendices/glossary'
    );

    return recommendedPath;
  };

  // Adjust content difficulty based on user experience level
  const getDifficultyAdjustedContent = (content: string, baseDifficulty: string): string => {
    if (!user?.profile) {
      return content;
    }

    const experienceLevel = user.profile.experienceLevel;

    // Adjust content based on experience level
    switch (experienceLevel) {
      case 'beginner':
        // Simplify content, add more explanations
        return simplifyContent(content);
      case 'intermediate':
        // Keep content as is but add some advanced references
        return addAdvancedReferences(content);
      case 'advanced':
        // Add more technical depth and advanced examples
        return addTechnicalDepth(content);
      default:
        return content;
    }
  };

  // Get examples aligned with user interests
  const getInterestAlignedExamples = (topic: string): string[] => {
    if (!user?.profile) {
      // Default examples
      return getDefaultExamples(topic);
    }

    const interests = user.profile.learningGoals || [];
    const interestExamples: string[] = [];

    // Generate examples based on user interests
    if (interests.includes('roboticsInterest')?.toLowerCase().includes('industrial')) {
      interestExamples.push(`Industrial robotics example for ${topic}`);
    }

    if (interests.includes('roboticsInterest')?.toLowerCase().includes('service')) {
      interestExamples.push(`Service robotics example for ${topic}`);
    }

    if (interests.includes('roboticsInterest')?.toLowerCase().includes('research')) {
      interestExamples.push(`Research-oriented example for ${topic}`);
    }

    // Add examples based on learning goals
    if (interests.includes('computer_vision')) {
      interestExamples.push(`Computer vision application for ${topic}`);
    }

    if (interests.includes('motion_planning')) {
      interestExamples.push(`Motion planning implementation for ${topic}`);
    }

    if (interests.includes('ai_integration')) {
      interestExamples.push(`AI integration approach for ${topic}`);
    }

    // If no specific interest-aligned examples, return defaults
    if (interestExamples.length === 0) {
      return getDefaultExamples(topic);
    }

    return interestExamples;
  };

  // Helper functions
  const adjustTextComplexity = (content: string, level: string | undefined): string => {
    if (!level) return content;

    switch (level) {
      case 'beginner':
        // Replace complex terms with simpler explanations
        return content
          .replace(/\bROS 2\b/g, 'Robot Operating System (ROS 2)')
          .replace(/\bQdrant\b/g, 'vector database (Qdrant)')
          .replace(/\bCohere\b/g, 'AI service (Cohere)')
          .replace(/algorithm/g, 'method or procedure');
      case 'advanced':
        // Add technical depth
        return content + '\n\n*Advanced Note: This concept can be extended with additional optimizations...';
      default:
        return content;
    }
  };

  const filterExamplesByInterest = (examples: any[], interest: string | undefined): any[] => {
    if (!interest) return examples;

    // Filter examples based on user interest
    const filtered = examples.filter(example => {
      // This is a simplified example - in reality, you'd have more sophisticated matching
      return example.tags?.some((tag: string) =>
        interest.toLowerCase().includes(tag.toLowerCase())
      );
    });

    return filtered.length > 0 ? filtered : examples; // Return all if none match
  };

  const adjustExerciseDifficulty = (exercises: any[], level: string | undefined): any[] => {
    if (!level) return exercises;

    switch (level) {
      case 'beginner':
        return exercises.map(ex => ({
          ...ex,
          hints: ex.hints || ['Start with the basics...'],
          difficulty: 'beginner'
        }));
      case 'advanced':
        return exercises.map(ex => ({
          ...ex,
          extensions: ex.extensions || ['For advanced users: Try implementing...'],
          difficulty: 'advanced'
        }));
      default:
        return exercises;
    }
  };

  const simplifyContent = (content: string): string => {
    // Simplify complex sentences and add explanations
    return content + '\n\n*Beginner tip: Take your time to understand each concept before moving forward.*';
  };

  const addAdvancedReferences = (content: string): string => {
    // Add references to more advanced materials
    return content + '\n\n*Advanced reference: For more depth, see...*';
  };

  const addTechnicalDepth = (content: string): string => {
    // Add more technical details
    return content + '\n\n*Technical detail: The underlying implementation involves...*';
  };

  const getDefaultExamples = (topic: string): string[] => {
    // Return default examples for a topic
    const defaultExamples: Record<string, string[]> = {
      'ros2': [
        'Example: Controlling a TurtleBot with ROS 2',
        'Example: Creating a ROS 2 package for sensor data'
      ],
      'gazebo': [
        'Example: Simulating a mobile robot in Gazebo',
        'Example: Creating custom Gazebo models'
      ],
      'isaac': [
        'Example: Training a robot with Isaac Sim',
        'Example: Implementing perception in Isaac'
      ],
      'vla': [
        'Example: Vision-language-action pipeline',
        'Example: Natural language robot control'
      ]
    };

    return defaultExamples[topic.toLowerCase()] || [
      `Default example for ${topic}`,
      `Standard implementation of ${topic}`
    ];
  };

  const value = {
    getPersonalizedContent,
    getRecommendedPath,
    getDifficultyAdjustedContent,
    getInterestAlignedExamples,
  };

  return (
    <PersonalizationContext.Provider value={value}>
      {children}
    </PersonalizationContext.Provider>
  );
};

export const usePersonalization = () => {
  const context = useContext(PersonalizationContext);
  if (context === undefined) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
};

// Higher-order component to wrap content with personalization
export const withPersonalization = (Component: React.ComponentType<any>) => {
  return (props: any) => (
    <PersonalizationProvider>
      <Component {...props} />
    </PersonalizationProvider>
  );
};