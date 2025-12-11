# Quickstart Guide: Simple Hero Homepage Implementation

## Prerequisites
- Node.js 18+ installed
- Docusaurus 3.x project already set up
- Basic knowledge of React 18+ and CSS modules
- Understanding of the project constitution and design requirements

## Installation Steps

### 1. No Additional Dependencies Required
The simple hero homepage uses existing Docusaurus functionality and requires no additional dependencies beyond the current project setup.

### 2. Create the Landing Page Component
Create `src/pages/index.js` with the hero section:

```javascript
import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import clsx from 'clsx';
import Link from '@docusaurus/Link';

import HeroSection from '@site/src/components/Hero/HeroSection';
import './index.module.css';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`${siteConfig.title}`}
      description={siteConfig.tagline}>
      <HeroSection />
    </Layout>
  );
}
```

### 3. Create the Hero Section Component
Create `src/components/Hero/HeroSection.js`:

```javascript
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

import './HeroSection.module.css';

export default function HeroSection() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <section className="hero-section">
      <div className="hero-section__container">
        <h1 className="hero-section__title">
          {siteConfig.title}
        </h1>
        <p className="hero-section__subtitle">
          {siteConfig.tagline}
        </p>
        <div className="hero-section__cta-container">
          <Link
            className="button button--primary button--lg hero-section__cta-button"
            to="/chapters/c1-foundations">
            Start Reading
          </Link>
          <Link
            className="button button--secondary button--lg hero-section__cta-button hero-section__github-button"
            to="https://github.com/your-repo-url">
            GitHub
          </Link>
        </div>
      </div>
    </section>
  );
}
```

### 4. Create the Cyber Design System
Create `src/css/custom.css` with cyber color palette:

```css
/**
 * Cyber Design System - Physical AI & Humanoid Robotics Textbook
 */

:root {
  /* Cyber Color Palette */
  --color-cyber-blue: #00D9FF;
  --color-electric-purple: #8B5CF6;
  --color-neon-green: #10B981;
  --color-deep-space: #0A0E27;
  --color-soft-white: #F8FAFC;

  /* Semantic Color Tokens */
  --color-primary: var(--color-cyber-blue);
  --color-accent: var(--color-electric-purple);
  --color-success: var(--color-neon-green);
  --color-background: var(--color-deep-space);
  --color-text-primary: #FFFFFF;
  --color-text-secondary: rgba(255, 255, 255, 0.8);
  --color-text-muted: rgba(255, 255, 255, 0.6);

  /* Gradients */
  --gradient-cyber: linear-gradient(135deg, var(--color-cyber-blue) 0%, var(--color-electric-purple) 100%);
  --gradient-hero: linear-gradient(180deg, var(--color-deep-space) 0%, #1a1f3a 100%);

  /* Typography */
  --font-size-hero: clamp(2.5rem, 5vw, 4.5rem);
  --font-size-h1: clamp(2rem, 4vw, 3rem);
  --font-size-body: clamp(1rem, 1.5vw, 1.125rem);
  --font-family-heading: 'Inter', 'SF Pro Display', system-ui, sans-serif;
  --font-family-body: 'Inter', system-ui, sans-serif;

  /* Spacing System (8px base grid) */
  --spacing-xs: 8px;
  --spacing-sm: 16px;
  --spacing-md: 24px;
  --spacing-lg: 32px;
  --spacing-xl: 48px;
  --spacing-xxl: 64px;
}
```

### 5. Create Glassmorphism Effects
Create `src/css/glassmorphism.css`:

```css
/**
 * Glassmorphism Utility Classes
 */

.glass-card {
  background: rgba(255, 255, 255, 0.1);
  backdrop-filter: blur(10px) saturate(180%);
  -webkit-backdrop-filter: blur(10px) saturate(180%);
  border: 1px solid rgba(255, 255, 255, 0.18);
  border-radius: 12px;
  box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37);
}

/* Fallback for browsers without backdrop-filter support */
@supports not (backdrop-filter: blur(10px)) {
  .glass-card {
    background: rgba(255, 255, 255, 0.2);
    border: 1px solid rgba(255, 255, 255, 0.25);
  }
}
```

### 6. Create Animation Effects
Create `src/css/animations.css`:

```css
/**
 * Spring Physics Animations
 */

:root {
  --ease-spring: cubic-bezier(0.34, 1.56, 0.64, 1);
  --duration-fast: 200ms;
  --duration-normal: 300ms;
  --duration-slow: 500ms;
}

/* Fade In with Spring Physics */
@keyframes fadeIn {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.animate-fadeIn {
  animation: fadeIn var(--duration-normal) var(--ease-spring);
}

/* Hover/Focus animations */
.hover-scale {
  transition: transform var(--duration-normal) var(--ease-spring);
}

.hover-scale:hover {
  transform: scale(1.05);
}

/* Reduced motion support */
@media (prefers-reduced-motion: reduce) {
  *,
  *::before,
  *::after {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }
}
```

### 7. Update Docusaurus Configuration
Update `docusaurus.config.js` to include the new CSS files:

```javascript
module.exports = {
  // ... existing configuration
  stylesheets: [
    // ... existing stylesheets
    {
      href: '/css/custom.css',
      type: 'text/css',
    },
    {
      href: '/css/glassmorphism.css',
      type: 'text/css',
    },
    {
      href: '/css/animations.css',
      type: 'text/css',
    },
  ],
  // ... rest of configuration
};
```

### 8. Create Component-Specific Styles
Create `src/components/Hero/HeroSection.module.css`:

```css
.hero-section {
  display: flex;
  align-items: center;
  justify-content: center;
  min-height: 100vh;
  padding: var(--spacing-lg) var(--spacing-md);
  background: var(--gradient-hero);
  text-align: center;
  color: var(--color-text-primary);
}

.hero-section__container {
  max-width: 800px;
  padding: var(--spacing-xl);
}

.hero-section__title {
  font-size: var(--font-size-hero);
  font-weight: 700;
  margin: 0 0 var(--spacing-md) 0;
  line-height: 1.1;
  animation: fadeIn var(--duration-normal) var(--ease-spring);
}

.hero-section__subtitle {
  font-size: var(--font-size-body);
  margin: 0 0 var(--spacing-lg) 0;
  line-height: 1.5;
  color: var(--color-text-secondary);
  animation: fadeIn var(--duration-normal) var(--ease-spring) 0.1s both;
}

.hero-section__cta-container {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-md);
  align-items: center;
  animation: fadeIn var(--duration-normal) var(--ease-spring) 0.2s both;
}

@media (min-width: 768px) {
  .hero-section__cta-container {
    flex-direction: row;
    justify-content: center;
    gap: var(--spacing-lg);
  }
}

.hero-section__cta-button {
  min-width: 200px;
  font-weight: 600;
  border-radius: 8px;
  padding: var(--spacing-sm) var(--spacing-md);
  transition: all var(--duration-normal) var(--ease-spring);
}

.hero-section__cta-button:hover {
  transform: translateY(-2px);
  box-shadow: 0 10px 25px rgba(0, 0, 0, 0.2);
}

.hero-section__github-button {
  background: transparent;
  border: 2px solid var(--color-cyber-blue);
  color: var(--color-cyber-blue);
}

.hero-section__github-button:hover {
  background: rgba(0, 217, 255, 0.1);
  border-color: var(--color-electric-purple);
  color: var(--color-electric-purple);
}
```

## Running the Application

### Development
```bash
npm start
```

The application will be available at `http://localhost:3000` with:
- Simple hero homepage at root route
- Centered book title with subtitle
- Two buttons: "Start Reading" and "GitHub"
- Cyber aesthetic with glassmorphism effects
- Responsive design across mobile, tablet, and desktop

### Production Build
```bash
npm run build
```

## Testing and Validation

### Performance Testing
1. Run `npm run build && npm run serve` to create and serve a production build
2. Use browser devtools to verify Core Web Vitals:
   - LCP < 2.5s
   - FID < 100ms
   - CLS < 0.1
3. Verify page load time is under 3 seconds

### Accessibility Testing
1. Verify 44x44px minimum touch target size for buttons
2. Test keyboard navigation and focus indicators
3. Validate color contrast ratios (minimum 4.5:1)
4. Test with screen readers
5. Verify `prefers-reduced-motion` support

### Responsive Testing
1. Test on mobile devices (width < 768px)
2. Test on tablets (768px - 1024px)
3. Test on desktop (width > 1024px)
4. Verify proper layout adaptation at all breakpoints