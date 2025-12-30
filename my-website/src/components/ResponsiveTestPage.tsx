import React from 'react';
import ResponsiveUtils, { ResponsiveComponent, ResponsiveGrid, ResponsiveText } from './ResponsiveUtils';

// Test component to verify responsive functionality
const ResponsiveTestPage = () => {
  return (
    <div className="responsive-container">
      <h1>Responsive Design Test Page</h1>

      {/* Test responsive text */}
      <ResponsiveText as="h1">This is a responsive heading</ResponsiveText>
      <ResponsiveText as="h2">This is a responsive subheading</ResponsiveText>
      <ResponsiveText as="p">This is responsive paragraph text that adjusts based on screen size.</ResponsiveText>

      {/* Test responsive component */}
      <ResponsiveComponent
        mobile={
          <div>
            <h3>Mobile View</h3>
            <p>This content is shown only on mobile devices.</p>
          </div>
        }
        tablet={
          <div>
            <h3>Tablet View</h3>
            <p>This content is shown only on tablet devices.</p>
          </div>
        }
        desktop={
          <div>
            <h3>Desktop View</h3>
            <p>This content is shown only on desktop devices.</p>
          </div>
        }
        fallback={
          <div>
            <h3>Default View</h3>
            <p>This content is shown when no specific device type matches.</p>
          </div>
        }
      />

      {/* Test responsive grid */}
      <h3>Responsive Grid Test</h3>
      <ResponsiveGrid
        cols={{ xs: 1, sm: 2, md: 2, lg: 3, xl: 4 }}
        gap="1.5rem"
      >
        {[1, 2, 3, 4, 5, 6].map((item) => (
          <div key={item} className="responsive-card">
            <h4>Grid Item {item}</h4>
            <p>This card adjusts its layout based on screen size.</p>
          </div>
        ))}
      </ResponsiveGrid>

      {/* Test responsive utilities */}
      <div className="responsive-padding">
        <h3>Responsive Padding</h3>
        <p>This section has responsive padding that adjusts based on screen size.</p>
      </div>

      <div className="responsive-margin">
        <h3>Responsive Margin</h3>
        <p>This section has responsive margin that adjusts based on screen size.</p>
      </div>

      {/* Test responsive buttons */}
      <div className="responsive-margin">
        <button className="responsive-button responsive-button--primary">Primary Button</button>
        <button className="responsive-button responsive-button--secondary">Secondary Button</button>
        <button className="responsive-button responsive-button--lg">Large Button</button>
        <button className="responsive-button responsive-button--sm">Small Button</button>
      </div>

      {/* Test responsive display utilities */}
      <div className="responsive-margin">
        <div className="d-block d-md-none">Visible on mobile only</div>
        <div className="d-none d-md-block">Visible on desktop only</div>
      </div>
    </div>
  );
};

export default ResponsiveTestPage;