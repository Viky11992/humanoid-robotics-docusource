import React from 'react';
import Layout from '@theme/Layout';

export default function DocItemLayout(props) {
  return (
    <Layout>
      <>{props.children}</>
    </Layout>
  );
}