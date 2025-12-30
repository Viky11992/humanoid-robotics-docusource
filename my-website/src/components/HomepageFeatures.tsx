import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';
import Link from '@docusaurus/Link';
import { motion } from 'framer-motion';

type FeatureItem = {
  title: string;
  icon: string;
  description: JSX.Element;
  link?: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI Fundamentals',
    icon: '🤖',
    description: (
      <>
        Learn the core principles of Physical AI, where intelligence is embedded
        in physical systems that interact with the real world through sensors,
        actuators, and control systems.
      </>
    ),
    link: '/docs/intro/physical-ai-overview'
  },
  {
    title: 'Humanoid Robotics',
    icon: '🦾',
    description: (
      <>
        Explore the design, control, and implementation of humanoid robots that
        mimic human locomotion, manipulation, and interaction capabilities.
      </>
    ),
    link: '/docs/module-1-ros2/ros2-intro'
  },
  {
    title: 'Simulation & Control',
    icon: '🎮',
    description: (
      <>
        Master simulation environments like Gazebo and Unity, and learn advanced
        control systems for robot navigation and manipulation tasks.
      </>
    ),
    link: '/docs/module-2-simulation/gazebo-intro'
  },
];

function Feature({ title, icon, description, link }: FeatureItem) {
  const content = (
    <motion.div
      className="col col--4"
      initial={{ opacity: 0, y: 30 }}
      whileInView={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.6 }}
      viewport={{ once: true }}
      whileHover={{ y: -10 }}
      style={{ marginBottom: '2rem' }}
    >
      <div className={clsx('feature-card', 'responsive-card')}>
        <div className={styles.roboticsFeatureIcon} style={{ fontSize: '3rem', marginBottom: '1rem' }}>
          {icon}
        </div>
        <h3 className={styles.roboticsFeatureTitle}>{title}</h3>
        <p className={styles.roboticsFeatureDescription}>{description}</p>
      </div>
    </motion.div>
  );

  if (link) {
    return (
      <Link to={link} style={{ textDecoration: 'none', color: 'inherit' }}>
        {content}
      </Link>
    );
  }

  return content;
}

export default function HomepageFeatures(): JSX.Element {
  return (
    <section className={styles.roboticsFeatureCards}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}