import React from 'react';
import Layout from '@theme/Layout';

export default function MDXLayout(props) {
  const { wrapper: MDXWrapper, ...layoutProps } = props;

  return (
    <Layout {...layoutProps}>
      <MDXWrapper>
        {props.children}
      </MDXWrapper>
    </Layout>
  );
}