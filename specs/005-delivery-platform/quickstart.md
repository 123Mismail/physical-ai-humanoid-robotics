# Quickstart Guide: Documentation Homepage Implementation for Delivery Platform

## Prerequisites
- Node.js 18+ installed
- Docusaurus 3.x project already set up
- Basic knowledge of React 18+ and CSS modules
- Understanding of the project constitution and specifications

## Installation Steps

### 1. Install Dependencies
```bash
npm install react-i18next i18next i18next-browser-languagedetector
```

### 2. Configure Docusaurus for Internationalization
Update `docusaurus.config.js` to include internationalization settings:
```javascript
module.exports = {
  // ... existing config
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'], // Include Urdu for RTL support
    localeConfigs: {
      ur: {
        direction: 'rtl',
      },
    },
  },
};
```

### 3. Create Documentation Homepage
Create `src/pages/index.js` with standard Docusaurus documentation homepage:
```javascript
import React, { lazy, Suspense } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import clsx from 'clsx';
import Translate, {translate} from '@docusaurus/Translate';

import HeroSection from '@site/src/components/Hero/HeroSection';
import styles from './index.module.css';

// Lazy load non-critical components for better TTI
const HomepageFeatures = lazy(() => import('./HomepageFeatures'));

export default function Home() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`${siteConfig.title}`}
      description={translate({
        id: 'homepage.description',
        description: 'Documentation homepage meta description',
        message: 'A comprehensive university-level textbook on Physical AI and Humanoid Robotics covering ROS 2, simulation, and practical implementations.'
      })}>
      <HeroSection />
      <main>
        <section className={clsx(styles.introduction, "padding-vert--lg")}>
          <div className="container">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 className="text--center padding-bottom--md">
                  <Translate id="homepage.introductionTitle" description="Introduction section title">
                    About This Textbook
                  </Translate>
                </h2>
                <p className={clsx(styles.introductionText, 'text--center')}>
                  <Translate id="homepage.introduction" description="Documentation homepage introduction">
                    Welcome to the university-level textbook on Physical AI and Humanoid Robotics. This book provides students and researchers with deep understanding of humanoid systems, ROS 2, differential mechanisms, and implementations.
                  </Translate>
                </p>
              </div>
            </div>
          </div>
        </section>
        <Suspense fallback={<div className={styles.featuresLoading}>Loading features...</div>}>
          <HomepageFeatures />
        </Suspense>
      </main>
    </Layout>
  );
}
```

### 4. Create Modern Hero Section
Create `src/components/Hero/HeroSection.js`:
```javascript
import React, { useEffect, useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './HeroSection.module.css';

/**
 * Hero Section Component
 * Physical AI & Humanoid Robotics Textbook
 *
 * Constitution Compliance: Principle IX - Modern UI/UX Design Standards
 * Spec: specs/005-delivery-platform/spec.md (M-06, FR-010, FR-012)
 */
export default function HeroSection() {
  const { siteConfig } = useDocusaurusContext();
  const [isVisible, setIsVisible] = useState(false);
  const [elementInView, setElementInView] = useState(false);

  useEffect(() => {
    setIsVisible(true);

    // Set up Intersection Observer for parallax effect
    const heroElement = document.querySelector('.hero-section');
    if (!heroElement) return;

    const observer = new IntersectionObserver(
      ([entry]) => {
        setElementInView(entry.isIntersecting);
        if (entry.isIntersecting) {
          heroElement.classList.add('hero-section--in-view');
        } else {
          heroElement.classList.remove('hero-section--in-view');
        }
      },
      { threshold: 0.1 }
    );

    observer.observe(heroElement);

    return () => {
      if (observer && heroElement) {
        observer.unobserve(heroElement);
      }
    };
  }, []);

  return (
    <section className={clsx('hero-section', {
      'hero-section--visible': isVisible,
      'hero-section--animated': elementInView
    })}>
      <div className="hero-section__container">
        <div className="hero-section__content">
          <h1 className="hero-section__title">
            {siteConfig.title}
          </h1>
          <p className="hero-section__subtitle">
            {siteConfig.tagline}
          </p>
          <div className="hero-section__cta">
            <Link
              className="button button--primary button--lg hero-section__cta-button"
              to="/chapters/c1-foundations">
              Read the Book
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}
```

### 5. Create Feature Cards Component
Create `src/components/Cards/FeatureCard.js`:
```javascript
import React, { useEffect, useRef } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import './FeatureCard.module.css';

/**
 * Feature Card Component
 * Physical AI & Humanoid Robotics Textbook
 *
 * Constitution Compliance: Principle IX - Modern UI/UX Design Standards
 * Spec: specs/005-delivery-platform/spec.md (M-06, FR-011, FR-017)
 */
export default function FeatureCard({
  title,
  description,
  to,
  icon,
  index = 0,
  className,
  ...props
}) {
  const cardRef = useRef(null);

  useEffect(() => {
    const card = cardRef.current;
    if (!card) return;

    // Add staggered animation delay
    card.style.setProperty('--stagger-index', index);

    // Add intersection observer for animations
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          card.classList.add('feature-card--animate');
          observer.unobserve(card);
        }
      },
      { threshold: 0.1, rootMargin: '0px 0px -50px 0px' }
    );

    observer.observe(card);

    return () => {
      if (observer && card) {
        observer.unobserve(card);
      }
    };
  }, [index]);

  const cardClasses = clsx(
    'feature-card',
    'glass-card',
    'animate-fadeIn-stagger',
    className
  );

  const content = (
    <>
      {icon && <div className="feature-card__icon">{icon}</div>}
      <h3 className="feature-card__title">{title}</h3>
      <p className="feature-card__description">{description}</p>
    </>
  );

  if (to) {
    return (
      <Link
        ref={cardRef}
        className={cardClasses}
        to={to}
        {...props}
      >
        {content}
      </Link>
    );
  }

  return (
    <div
      ref={cardRef}
      className={cardClasses}
      {...props}
    >
      {content}
    </div>
  );
}
```

### 6. Set Up Modern Design System
Create `src/css/custom.css` with cyber color palette and typography:
```css
/**
 * Physical AI & Humanoid Robotics Textbook
 * Custom Design System - Cyber Color Palette & Typography
 *
 * Constitution Compliance: Principle IX - Modern UI/UX Design Standards
 * Spec: specs/005-delivery-platform/spec.md (M-06, REQ-06, FR-010, FR-013)
 */

/* =============================================================================
   CYBER COLOR PALETTE
   ============================================================================= */

:root {
  /* Primary Cyber Colors */
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

  /* Additional Palette */
  --color-text-primary: #FFFFFF;
  --color-text-secondary: rgba(255, 255, 255, 0.8);
  --color-text-muted: rgba(255, 255, 255, 0.6);

  /* Gradient Backgrounds */
  --gradient-cyber: linear-gradient(135deg, var(--color-cyber-blue) 0%, var(--color-electric-purple) 100%);
  --gradient-hero: linear-gradient(180deg, var(--color-deep-space) 0%, #1a1f3a 100%);

  /* Docusaurus Theme Colors (mapped to cyber palette) */
  --ifm-color-primary: var(--color-cyber-blue);
  --ifm-color-primary-dark: #00C4E6;
  --ifm-color-primary-darker: #00B8DB;
  --ifm-color-primary-darkest: #0098B8;
  --ifm-color-primary-light: #1AE0FF;
  --ifm-color-primary-lighter: #33E5FF;
  --ifm-color-primary-lightest: #66EBFF;
  --ifm-code-font-size: 95%;
  --docusaurus-highlighted-code-line-bg: rgba(0, 217, 255, 0.1);
}

/* =============================================================================
   FLUID TYPOGRAPHY SYSTEM
   ============================================================================= */

:root {
  /* Font Families - T123: Geometric fonts with improved character spacing */
  --font-family-heading: 'SF Pro Display', 'Inter', 'Roboto Mono', system-ui, -apple-system, 'Segoe UI', sans-serif;
  --font-family-body: 'Inter', 'Roboto Mono', system-ui, -apple-system, 'Segoe UI', sans-serif;

  /* Fluid Font Sizes using clamp() */
  --font-size-hero: clamp(2.5rem, 8vw, 5rem);
  --font-size-h1: clamp(2rem, 5vw, 4rem);
  --font-size-h2: clamp(1.5rem, 4vw, 3rem);
  --font-size-h3: clamp(1.25rem, 3vw, 2rem);
  --font-size-body: clamp(1rem, 2vw, 1.125rem);
  --font-size-small: clamp(0.875rem, 1.5vw, 1rem);

  /* Line Heights */
  --line-height-tight: 1.2;
  --line-height-normal: 1.5;
  --line-height-relaxed: 1.75;

  /* Font Weights */
  --font-weight-normal: 400;
  --font-weight-medium: 500;
  --font-weight-semibold: 600;
  --font-weight-bold: 700;

  /* Character spacing for improved legibility - T123 */
  --font-letter-spacing-tight: -0.02em;
  --font-letter-spacing-normal: 0;
  --font-letter-spacing-wide: 0.02em;
  --font-letter-spacing-extra-wide: 0.05em;
}
```

### 7. Add Glassmorphism Effects
Create `src/css/glassmorphism.css`:
```css
/**
 * Glassmorphism Utility Classes
 * Physical AI & Humanoid Robotics Textbook
 *
 * Constitution Compliance: Principle IX - Modern UI/UX Design Standards
 * Spec: specs/005-delivery-platform/spec.md (M-06, IR-06, FR-011)
 */

/* =============================================================================
   GLASSMORPHISM BASE CARD
   ============================================================================= */

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

/* =============================================================================
   GLASSMORPHISM VARIANTS
   ============================================================================= */

/* Light variant */
.glass-card-light {
  background: rgba(255, 255, 255, 0.15);
  backdrop-filter: blur(12px) saturate(200%);
  -webkit-backdrop-filter: blur(12px) saturate(200%);
  border: 1px solid rgba(255, 255, 255, 0.25);
  border-radius: 12px;
  box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.2);
}

/* Dark variant */
.glass-card-dark {
  background: rgba(10, 14, 39, 0.6);
  backdrop-filter: blur(10px) saturate(180%);
  -webkit-backdrop-filter: blur(10px) saturate(180%);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 12px;
  box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.5);
}

/* =============================================================================
   GLASSMORPHISM INTERACTIVE STATES
   ============================================================================= */

.glass-card:hover,
.glass-card-light:hover,
.glass-card-dark:hover {
  background: rgba(255, 255, 255, 0.2);
  border-color: rgba(255, 255, 255, 0.3);
  box-shadow: 0 12px 48px 0 rgba(31, 38, 135, 0.5);
  transform: translateY(-2px);
  transition: all 300ms cubic-bezier(0.34, 1.56, 0.64, 1);
}

/* Focus state for accessibility */
.glass-card:focus-within,
.glass-card-light:focus-within,
.glass-card-dark:focus-within {
  outline: 3px solid var(--color-primary, #00D9FF);
  outline-offset: 2px;
  border-radius: 12px;
}
```

### 8. Add Animations with Spring Physics
Create `src/css/animations.css`:
```css
/**
 * Spring Physics Animations
 * Physical AI & Humanoid Robotics Textbook
 *
 * Constitution Compliance: Principle IX - Modern UI/UX Design Standards
 * Spec: specs/005-delivery-platform/spec.md (M-06, IR-06, FR-014, FR-016)
 */

:root {
  /* Spring Physics Easing (Principle IX) */
  --ease-spring: cubic-bezier(0.34, 1.56, 0.64, 1);

  /* Additional Easing Functions */
  --ease-in-out: cubic-bezier(0.4, 0, 0.2, 1);
  --ease-in: cubic-bezier(0.4, 0, 1, 1);
  --ease-out: cubic-bezier(0, 0, 0.2, 1);

  /* Animation Durations (Principle IX) */
  --duration-fast: 200ms;
  --duration-normal: 300ms;
  --duration-slow: 500ms;
  --duration-slower: 700ms;
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

/* Slide Up with Spring Physics */
@keyframes slideUp {
  from {
    opacity: 0;
    transform: translateY(40px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.animate-slideUp {
  animation: slideUp var(--duration-normal) var(--ease-spring);
}

/* Staggered animations for feature cards */
.animate-fadeIn-stagger {
  animation: fadeIn var(--duration-normal) var(--ease-spring);
  animation-delay: calc(var(--stagger-index, 0) * 100ms);
  opacity: 0;
  animation-fill-mode: forwards;
}

/* Hover/Focus animations */
.hover-scale {
  transition: transform var(--duration-normal) var(--ease-spring);
}

.hover-scale:hover {
  transform: scale(1.05);
}

.hover-lift {
  transition: transform var(--duration-normal) var(--ease-spring),
              box-shadow var(--duration-normal) var(--ease-spring);
}

.hover-lift:hover {
  transform: translateY(-4px);
  box-shadow: 0 12px 24px rgba(0, 0, 0, 0.3);
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

### 9. Update Docusaurus Configuration
Update `docusaurus.config.js` to include the new CSS files:
```javascript
module.exports = {
  // ... existing configuration
  presets: [
    [
      'classic',
      {
        // ... existing preset config
        theme: {
          customCss: [
            require.resolve('./src/css/custom.css'),
            require.resolve('./src/css/glassmorphism.css'),
            require.resolve('./src/css/animations.css'),
          ],
        },
        // ... rest of config
      },
    ],
  ],
  // ... rest of configuration
};
```

## Running the Application
```bash
npm start
```

The application will be available at `http://localhost:3000` with:
- Documentation homepage at root route with essential elements
- Cyber color palette and glassmorphism effects
- Spring physics animations and fluid typography
- Urdu locale support with RTL layout
- Fully responsive design across mobile, tablet, and desktop
- Module-based footer navigation

## Performance & Accessibility Testing

### Performance Testing
1. Run `npm run build` to create a production build
2. Use Lighthouse to audit Core Web Vitals:
   - LCP < 2.5s
   - FID < 100ms
   - CLS < 0.1
3. Verify page load time is under 3 seconds

### Accessibility Testing
1. Verify 3px visible focus indicators on all interactive elements
2. Test with `prefers-reduced-motion` media query
3. Check color contrast ratios (minimum 4.5:1)
4. Validate semantic HTML structure

### Responsive Testing
1. Test on mobile devices (width < 768px)
2. Test on tablets (768px - 1024px)
3. Test on desktop (width > 1024px)
4. Verify touch targets are minimum 44x44px