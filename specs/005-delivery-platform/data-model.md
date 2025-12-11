# Data Model: Documentation Homepage Elements for Delivery Platform

## Entity: Documentation Homepage Configuration
**Description**: Main configuration for the documentation homepage with essential elements
**Fields**:
- id: string (unique identifier for the documentation homepage configuration)
- title: string (main headline that communicates the value proposition immediately)
- subtitle: string (1-2 sentences providing additional context and benefits)
- primaryCtaText: string (text for the primary call-to-action button, e.g., "Read the Book")
- primaryCtaUrl: string (destination URL for the primary CTA, e.g., "/chapters/c1-foundations")
- layoutType: string (layout type - 'documentation', 'sidebar', etc.)
- performanceMetrics: object (Core Web Vitals targets and performance requirements)
- isResponsive: boolean (whether the page is fully responsive across devices)

**Validation rules**:
- title must be between 10-100 characters and clearly communicate value proposition
- subtitle must be between 20-200 characters and provide context/benefits
- primaryCtaText must be between 3-20 characters
- primaryCtaUrl must be a valid internal route starting with '/'
- layoutType must be one of the supported layout types
- performanceMetrics must include LCP < 2.5s, FID < 100ms, CLS < 0.1

## Entity: Social Proof Element
**Description**: Elements that establish credibility and trust (testimonials, credentials, statistics)
**Fields**:
- id: string (unique identifier for the social proof element)
- type: string (type of social proof - 'testimonial', 'credential', 'statistic', 'endorsement')
- content: string (the actual testimonial, credential, or statistic)
- author: string (author name for testimonials, credential name for credentials)
- avatarUrl: string (URL for author's avatar image, optional)
- position: number (display position on the landing page)
- isVisible: boolean (whether the element is currently visible)

**Validation rules**:
- content must be between 10-200 characters
- author must be between 2-50 characters if type is 'testimonial'
- avatarUrl must be a valid image URL if provided
- type must be one of the supported social proof types

## Entity: Feature Card
**Description**: Feature cards that highlight key aspects of the textbook
**Fields**:
- id: string (unique identifier for the feature card)
- title: string (feature title)
- description: string (feature description)
- icon: string (icon identifier or emoji)
- url: string (URL to navigate to when feature card is clicked)
- position: number (position in the grid layout)
- isInteractive: boolean (whether the card has hover/interaction effects)
- glassmorphismEnabled: boolean (whether to apply glassmorphism effect)

**Validation rules**:
- title must be between 5-50 characters
- description must be between 20-200 characters
- icon must be a valid emoji or icon identifier
- url must be a valid internal or external URL
- position must be a positive integer

## Entity: Locale Configuration
**Description**: Language and regional settings including RTL/LTR layout rules
**Fields**:
- locale: string (locale code, e.g., 'en', 'ur')
- direction: string ('ltr' or 'rtl' based on locale)
- translations: object (localized strings for the locale)
- isRtl: boolean (whether the layout should be right-to-left)

**Validation rules**:
- locale must be one of the supported locales ('en', 'ur')
- direction must be either 'ltr' or 'rtl'
- isRtl must match the expected direction for the locale
- translations must include all required strings for the locale

## Entity: Module Structure
**Description**: High-level textbook organization derived from sidebar metadata
**Fields**:
- id: string (unique identifier for the module)
- title: string (display title of the module)
- description: string (brief description of the module)
- firstChapterUrl: string (route to the first chapter in the module)
- order: number (display order in footer navigation)
- maxDisplayCount: number (maximum modules to display in footer, default 3)

**Validation rules**:
- title must be non-empty and between 5-100 characters
- firstChapterUrl must be a valid route
- order must be a positive integer
- maxDisplayCount must be between 1-10

## Entity: Visual Design Configuration
**Description**: Configuration for the modern UI/UX design elements
**Fields**:
- colorPalette: object (cyber color palette configuration)
- typography: object (font and sizing configuration)
- animationsEnabled: boolean (whether animations are enabled)
- animationType: string (type of animations - 'spring-physics', 'ease-in-out', etc.)
- glassmorphismEnabled: boolean (whether glassmorphism effects are enabled)
- responsiveBreakpoints: object (breakpoints for different device sizes)

**Validation rules**:
- colorPalette must include Cyber Blue (#00D9FF), Electric Purple (#8B5CF6), Neon Green (#10B981), Deep Space (#0A0E27)
- typography must support fluid scaling with clamp() function
- animationType must be one of the supported animation types
- responsiveBreakpoints must include mobile (<768px), tablet (768px-1024px), desktop (>1024px)

## Entity: Performance Configuration
**Description**: Configuration for performance optimization requirements
**Fields**:
- coreWebVitals: object (Core Web Vitals targets)
- maxLoadTime: number (maximum allowed page load time in seconds)
- imageOptimization: boolean (whether images are optimized)
- lazyLoadingEnabled: boolean (whether lazy loading is enabled for non-critical components)
- fontLoadingStrategy: string (font loading approach - 'swap', 'fallback', 'optional')

**Validation rules**:
- coreWebVitals must include LCP < 2.5s, FID < 100ms, CLS < 0.1
- maxLoadTime must be <= 3 seconds
- fontLoadingStrategy must be one of the supported strategies

## State Transitions

### Documentation Homepage States
1. **Loading** → **Loaded**: When the documentation homepage content is successfully fetched
2. **Loaded** → **Error**: When there's an error loading content
3. **Loaded** → **LocaleChanged**: When the user switches locale
4. **Loaded** → **CTAClicked**: When the primary CTA is clicked (triggers navigation)

### Component States
- **FeatureCard**: idle → hover → clicked
- **CTAButton**: enabled → loading → disabled (on click during navigation)