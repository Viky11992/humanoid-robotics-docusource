# Docusaurus UI Enhancements for Physical AI & Humanoid Robotics Documentation

This document outlines the UI enhancements made to improve the user experience of the Physical AI & Humanoid Robotics textbook documentation site.

## Enhancements Made

### 1. Color Scheme & Theme
- Updated primary color from green to a robotics-themed blue (#1a73e8) with orange accent (#ff6d01)
- Added custom CSS variables for consistent theming across the site
- Enhanced dark mode colors to complement the new theme

### 2. Homepage Improvements
- Enhanced hero section with gradient background using robotics theme colors
- Improved button styling with better hover effects and animations
- Added feature cards component with hover effects and robotics-themed icons
- Added secondary button to view UI demo page

### 3. Documentation UI Components
- Created custom CSS classes for technical highlights with gradient backgrounds
- Added learning module components with progress bars
- Enhanced admonition (note, tip, caution, danger) styling
- Improved code block styling with better borders and padding
- Enhanced table styling for better readability

### 4. RAG Chatbot Integration
- Updated chatbot button and header to match the new theme colors
- Maintained all existing functionality while improving visual consistency

### 5. New Components
- Created `RoboticsCard` component for displaying special content blocks with different types (info, warning, success, danger)
- Created `ui-demo.md` page to demonstrate all new UI components

### 6. Navigation & Layout
- Enhanced sidebar hover effects
- Improved mobile responsiveness for buttons and layout
- Added custom styling for technical diagrams and images

## Files Modified

- `src/css/custom.css` - Main styling enhancements
- `src/pages/index.tsx` - Homepage updates
- `src/pages/index.module.css` - Homepage-specific styles
- `sidebars.ts` - Added UI demo page to navigation
- `docs/intro/ui-demo.md` - UI demonstration page
- `src/components/HomepageFeatures.tsx` - Enhanced homepage features component
- `src/components/RoboticsCard.tsx` - New custom component for technical content
- `src/components/RoboticsCard.module.css` - Styles for the new component

## How to Use New Components

### Technical Highlight
```html
<div className="technical-highlight">
**Important Technical Concept**: Your content here...
</div>
```

### Learning Module
```html
<div className="learning-module">
### Module: Module Title
Your module content here...
<div className="module-progress-bar">
  <div className="module-progress-fill" style={{width: '30%'}}></div>
</div>
</div>
```

### Robotics Card (React Component)
```jsx
import RoboticsCard from '@site/src/components/RoboticsCard';

<RoboticsCard title="Card Title" type="info" icon="🤖">
  Card content goes here...
</RoboticsCard>
```

## Running the Site

To see these enhancements in action:

1. Navigate to the `my-website` directory
2. Install dependencies: `npm install`
3. Start the development server: `npm start`
4. Visit `http://localhost:3000` to view the enhanced UI

The RAG chatbot will work if the backend service is running on the configured endpoint.