import React from 'react';
import clsx from 'clsx';
import styles from './RoboticsCard.module.css';

type RoboticsCardProps = {
  title: string;
  children: React.ReactNode;
  type?: 'info' | 'warning' | 'success' | 'danger';
  icon?: string;
};

const RoboticsCard: React.FC<RoboticsCardProps> = ({
  title,
  children,
  type = 'info',
  icon = 'ℹ️'
}) => {
  const cardClass = clsx(
    styles.roboticsCard,
    styles[`roboticsCard--${type}`]
  );

  return (
    <div className={cardClass}>
      <div className={styles.roboticsCardHeader}>
        <span className={styles.roboticsCardIcon}>{icon}</span>
        <h3 className={styles.roboticsCardTitle}>{title}</h3>
      </div>
      <div className={styles.roboticsCardContent}>
        {children}
      </div>
    </div>
  );
};

export default RoboticsCard;