import React, { useEffect, useState } from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import { motion } from 'framer-motion';

const ThemeToggle = () => {
  const { colorMode, setColorMode } = useColorMode();
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  if (!mounted) {
    return null;
  }

  const toggleTheme = () => {
    setColorMode(colorMode === 'light' ? 'dark' : 'light');
  };

  return (
    <motion.button
      onClick={toggleTheme}
      className="navbar__item navbar__link"
      aria-label={`Switch to ${colorMode === 'light' ? 'dark' : 'light'} mode`}
      whileHover={{ scale: 1.05 }}
      whileTap={{ scale: 0.95 }}
      style={{
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        width: '40px',
        height: '40px',
        borderRadius: '50%',
        border: 'none',
        background: 'transparent',
        cursor: 'pointer'
      }}
    >
      {colorMode === 'light' ? (
        <span role="img" aria-label="moon">
          🌙
        </span>
      ) : (
        <span role="img" aria-label="sun">
          ☀️
        </span>
      )}
    </motion.button>
  );
};

export default ThemeToggle;