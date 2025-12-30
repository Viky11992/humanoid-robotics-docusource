import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ResponsiveLayout from '@site/src/components/ResponsiveLayout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import AnimatedHero from '@site/src/components/AnimatedHero';

import styles from './index.module.css';

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <ResponsiveLayout
      title={`Physical AI & Humanoid Robotics`}
      description="A comprehensive textbook on Physical AI and Humanoid Robotics, covering ROS 2, simulation environments, NVIDIA Isaac, and Vision-Language-Action systems">
      <AnimatedHero />
      <main>
        <HomepageFeatures />
      </main>
      {/* RAG Chatbot will be initialized by the script in docusaurus.config.ts */}
    </ResponsiveLayout>
  );
}
