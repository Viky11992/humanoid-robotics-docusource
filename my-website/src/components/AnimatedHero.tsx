import React, { useEffect, useState } from 'react';
import { motion } from 'framer-motion';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from '../pages/index.module.css';

function AnimatedHero() {
  const { siteConfig } = useDocusaurusContext();
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    setIsVisible(true);
  }, []);

  const container = {
    hidden: { opacity: 0 },
    visible: {
      opacity: 1,
      transition: {
        staggerChildren: 0.2
      }
    }
  };

  const item = {
    hidden: { y: 20, opacity: 0 },
    visible: {
      y: 0,
      opacity: 1,
      transition: {
        duration: 0.6,
        ease: "easeOut"
      }
    }
  };

  return (
    <motion.header
      className={clsx('heroBanner')}
      initial="hidden"
      animate={isVisible ? "visible" : "hidden"}
      variants={container}
    >
      <div className="container">
        <motion.div variants={item}>
          <h1 className={clsx('hero__title', styles['responsive-hero-title'])}>
            {siteConfig.title}
          </h1>
        </motion.div>
        <motion.div variants={item}>
          <p className={clsx('hero__subtitle', styles['responsive-hero-subtitle'])}>{siteConfig.tagline}</p>
        </motion.div>
        <motion.div
          className={styles.roboticsHeroButtons}
          variants={item}
        >
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning
          </Link>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro/ui-demo">
            View UI Demo
          </Link>
        </motion.div>
      </div>
    </motion.header>
  );
}

export default AnimatedHero;