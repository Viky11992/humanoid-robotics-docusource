import React, { useEffect } from 'react';
import type { ReactNode } from 'react';
import Layout from '@theme/Layout';
import MobileNavbar from '@site/src/components/MobileNavbar';
import { useLocation } from '@docusaurus/router';

interface ResponsiveLayoutProps {
  children: ReactNode;
  title?: string;
  description?: string;
  image?: string;
  wrapperClassName?: string;
  pageClassName?: string;
  searchMetadatas?: {
    version?: string;
    tag?: string;
  };
}

const ResponsiveLayout: React.FC<ResponsiveLayoutProps> = ({
  children,
  title,
  description,
  image,
  wrapperClassName,
  pageClassName,
  searchMetadatas,
}) => {
  const location = useLocation();

  useEffect(() => {
    // Add responsive classes to body for better CSS targeting
    document.body.classList.add('responsive-layout');

    return () => {
      document.body.classList.remove('responsive-layout');
    };
  }, [location.pathname]);

  return (
    <Layout
      title={title}
      description={description}
      image={image}
      wrapperClassName={wrapperClassName}
      pageClassName={pageClassName}
      searchMetadatas={searchMetadatas}
    >
      <div className="main-content responsive-container">
        {children}
      </div>
    </Layout>
  );
};

export default ResponsiveLayout;