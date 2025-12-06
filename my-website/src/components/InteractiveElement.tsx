import React from 'react';
import useIsBrowser from '@docusaurus/useIsBrowser';
import CodeBlock from '@theme/CodeBlock';

interface InteractiveElementProps {
  type: 'quiz' | 'simulation' | 'code-playground' | 'markdown-live-code';
  content: string; // The content for the interactive element
  language?: string; // For code-playground, e.g., 'python', 'javascript'
  title?: string; // Optional title for the element
}

const InteractiveElement: React.FC<InteractiveElementProps> = ({ type, content, language, title }) => {
  const isBrowser = useIsBrowser();

  const renderContent = () => {
    switch (type) {
      case 'quiz':
        // Placeholder for a quiz component
        return (
          <div className="interactive-quiz">
            {title && <h3>{title}</h3>}
            <p>Quiz: {content}</p>
            {/* Implement quiz logic here */}
          </div>
        );
      case 'simulation':
        // Placeholder for a simulation component
        return (
          <div className="interactive-simulation">
            {title && <h3>{title}</h3>}
            <p>Simulation: {content}</p>
            {/* Implement simulation embedding here */}
          </div>
        );
      case 'code-playground':
        // This will be a standard code block for display, not live
        return (
          <div className="interactive-code-playground">
            {title && <h3>{title}</h3>}
            <CodeBlock language={language || 'javascript'}>
              {content}
            </CodeBlock>
          </div>
        );
      case 'markdown-live-code':
        // This will render a live code block if @docusaurus/theme-live-codeblock is configured
        // The content itself should be a fenced code block with the 'live' attribute
        return (
          <div className="interactive-live-code">
            {title && <h3>{title}</h3>}
            {/* Docusaurus processes MDX live code blocks directly,
                so we can render the raw markdown content here,
                and if it's a live code block, it will be interactive.
                This assumes MDX is enabled for the docs and processes this.
                For now, just displaying the content. Actual live rendering
                happens when a markdown file directly contains ```jsx live
            */}
            <CodeBlock language={language || 'markdown'}>
                {content}
            </CodeBlock>
          </div>
        );
      default:
        return <p>Unsupported interactive element type: {type}</p>;
    }
  };

  return (
    <div className="interactive-element-wrapper">
      {isBrowser ? renderContent() : <div>Loading interactive content...</div>}
    </div>
  );
};

export default InteractiveElement;