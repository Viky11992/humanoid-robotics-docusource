import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import useIsBrowser from '@docusaurus/useIsBrowser';
import clsx from 'clsx';
import styles from './MobileNavbar.module.css';

const MobileNavbar: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isScrolled, setIsScrolled] = useState(false);
  const location = useLocation();
  const isBrowser = useIsBrowser();

  useEffect(() => {
    const handleScroll = () => {
      setIsScrolled(window.scrollY > 10);
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  useEffect(() => {
    // Close mobile menu when route changes
    setIsOpen(false);
  }, [location.pathname]);

  useEffect(() => {
    // Prevent body scroll when mobile menu is open
    if (isOpen && isBrowser) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = 'unset';
    }

    return () => {
      document.body.style.overflow = 'unset';
    };
  }, [isOpen, isBrowser]);

  const toggleMenu = () => {
    setIsOpen(!isOpen);
  };

  const closeMenu = () => {
    setIsOpen(false);
  };

  return (
    <>
      <div className={clsx(styles.mobileNavbar, isScrolled && styles.scrolled)}>
        <button
          className={clsx(styles.menuButton, isOpen && styles.menuButtonOpen)}
          onClick={toggleMenu}
          aria-label={isOpen ? 'Close menu' : 'Open menu'}
          aria-expanded={isOpen}
        >
          <span></span>
          <span></span>
          <span></span>
        </button>
        <div className={styles.navbarTitle}>
          Physical AI & Humanoid Robotics
        </div>
      </div>

      {isOpen && (
        <div className={styles.overlay} onClick={closeMenu}>
          <div
            className={styles.sidebar}
            onClick={(e) => e.stopPropagation()}
          >
            <div className={styles.sidebarHeader}>
              <div className={styles.sidebarTitle}>
                Physical AI & Humanoid Robotics
              </div>
              <button
                className={styles.closeButton}
                onClick={closeMenu}
                aria-label="Close menu"
              >
                ×
              </button>
            </div>
            <nav className={styles.sidebarNav}>
              <a href="/" className={clsx(styles.navLink, 'touch-target')} onClick={closeMenu}>
                Home
              </a>
              <a href="/docs/intro" className={clsx(styles.navLink, 'touch-target')} onClick={closeMenu}>
                Tutorial
              </a>
              <a href="/docs/intro/ui-demo" className={clsx(styles.navLink, 'touch-target')} onClick={closeMenu}>
                UI Demo
              </a>
              <a
                href="https://github.com/Viky11992/humanoid-robotics-docusource"
                className={clsx(styles.navLink, 'touch-target')}
                onClick={closeMenu}
              >
                GitHub
              </a>
            </nav>
          </div>
        </div>
      )}
    </>
  );
};

export default MobileNavbar;