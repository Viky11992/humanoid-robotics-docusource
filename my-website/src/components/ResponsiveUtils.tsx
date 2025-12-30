import React from 'react';
import { useThemeConfig } from '@docusaurus/theme-common';
import type { Props } from '@theme/Navbar/Layout';

export interface ResponsiveBreakpoints {
  xs: number;
  sm: number;
  md: number;
  lg: number;
  xl: number;
  '2xl': number;
}

export const DEFAULT_BREAKPOINTS: ResponsiveBreakpoints = {
  xs: 0,
  sm: 576,
  md: 768,
  lg: 992,
  xl: 1200,
  '2xl': 1400,
};

export interface ResponsiveUtilsProps {
  children: React.ReactNode;
}

const ResponsiveUtils: React.FC<ResponsiveUtilsProps> = ({ children }) => {
  return <>{children}</>;
};

export default ResponsiveUtils;

// Utility functions for responsive design
export const useResponsive = () => {
  if (typeof window === 'undefined') {
    return {
      isMobile: false,
      isTablet: false,
      isDesktop: true,
      isTouchDevice: false,
      currentBreakpoint: 'desktop',
      breakpoints: DEFAULT_BREAKPOINTS,
    };
  }

  const isMobile = window.matchMedia('(max-width: 767px)').matches;
  const isTablet = window.matchMedia('(min-width: 768px) and (max-width: 991px)').matches;
  const isDesktop = window.matchMedia('(min-width: 992px)').matches;
  const isTouchDevice = 'ontouchstart' in window || navigator.maxTouchPoints > 0;

  let currentBreakpoint = 'desktop';
  if (isMobile) currentBreakpoint = 'mobile';
  else if (isTablet) currentBreakpoint = 'tablet';

  return {
    isMobile,
    isTablet,
    isDesktop,
    isTouchDevice,
    currentBreakpoint,
    breakpoints: DEFAULT_BREAKPOINTS,
  };
};

// Responsive component that renders different content based on screen size
export interface ResponsiveComponentProps {
  mobile?: React.ReactNode;
  tablet?: React.ReactNode;
  desktop?: React.ReactNode;
  fallback?: React.ReactNode;
  children?: React.ReactNode;
}

export const ResponsiveComponent: React.FC<ResponsiveComponentProps> = ({
  mobile,
  tablet,
  desktop,
  fallback,
  children,
}) => {
  const { isMobile, isTablet, isDesktop } = useResponsive();

  if (isMobile && mobile) return <>{mobile}</>;
  if (isTablet && tablet) return <>{tablet}</>;
  if (isDesktop && desktop) return <>{desktop}</>;

  return <>{children || fallback || null}</>;
};

// Hook for custom responsive behavior
export const useMediaQuery = (query: string): boolean => {
  if (typeof window === 'undefined') {
    return false;
  }

  const [matches, setMatches] = React.useState<boolean>(false);

  React.useEffect(() => {
    const media = window.matchMedia(query);
    if (media.matches !== matches) {
      setMatches(media.matches);
    }
    const listener = () => setMatches(media.matches);
    media.addEventListener('change', listener);
    return () => media.removeEventListener('change', listener);
  }, [matches, query]);

  return matches;
};

// Hook for screen size
export const useScreenSize = () => {
  const [screenSize, setScreenSize] = React.useState({
    width: typeof window !== 'undefined' ? window.innerWidth : 0,
    height: typeof window !== 'undefined' ? window.innerHeight : 0,
  });

  React.useEffect(() => {
    const handleResize = () => {
      setScreenSize({
        width: window.innerWidth,
        height: window.innerHeight,
      });
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  return screenSize;
};

// Responsive grid component
export interface ResponsiveGridProps {
  children: React.ReactNode;
  cols?: {
    xs?: number;
    sm?: number;
    md?: number;
    lg?: number;
    xl?: number;
  };
  gap?: string;
  className?: string;
}

export const ResponsiveGrid: React.FC<ResponsiveGridProps> = ({
  children,
  cols = { xs: 1, sm: 2, md: 2, lg: 3, xl: 4 },
  gap = '1.5rem',
  className = '',
}) => {
  const { width } = useScreenSize();

  let currentCols = cols.xs || 1;
  if (width >= DEFAULT_BREAKPOINTS.sm) currentCols = cols.sm || currentCols;
  if (width >= DEFAULT_BREAKPOINTS.md) currentCols = cols.md || currentCols;
  if (width >= DEFAULT_BREAKPOINTS.lg) currentCols = cols.lg || currentCols;
  if (width >= DEFAULT_BREAKPOINTS.xl) currentCols = cols.xl || currentCols;

  const gridStyle: React.CSSProperties = {
    display: 'grid',
    gridTemplateColumns: `repeat(${currentCols}, 1fr)`,
    gap,
  };

  return (
    <div
      className={`responsive-grid ${className}`}
      style={gridStyle}
    >
      {children}
    </div>
  );
};

// Responsive typography component
export interface ResponsiveTextProps {
  children: React.ReactNode;
  as?: 'h1' | 'h2' | 'h3' | 'h4' | 'h5' | 'h6' | 'p' | 'span' | 'div';
  className?: string;
}

export const ResponsiveText: React.FC<ResponsiveTextProps> = ({
  children,
  as: Component = 'p',
  className = '',
}) => {
  const componentClasses: Record<string, string> = {
    h1: 'responsive-h1',
    h2: 'responsive-h2',
    h3: 'responsive-h3',
    p: 'responsive-text',
    span: 'responsive-text',
    div: 'responsive-text',
  };

  return (
    <Component className={`${componentClasses[Component]} ${className}`}>
      {children}
    </Component>
  );
};