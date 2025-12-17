import { useEffect } from 'react';

const SelectedTextHandler = ({ onTextSelected }) => {
  useEffect(() => {
    const handleMouseUp = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        onTextSelected(selectedText);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [onTextSelected]);

  return null; // This component doesn't render anything
};

export default SelectedTextHandler;