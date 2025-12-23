# Implementation Plan: Docusaurus UI Enhancement

**Feature ID**: 003-ui-enhancement
**Plan Version**: 1.0
**Created**: 2025-12-23
**Status**: Planning

## Executive Summary

This plan outlines the implementation approach for enhancing the AI-Native Book Docusaurus site with modern visual design, improved navigation, enhanced readability, full accessibility compliance, and optimized performance. The enhancement will transform the current default Docusaurus theme into a polished, professional educational platform while maintaining the existing content structure and framework capabilities.

## Technical Context

### Current System State

**Docusaurus Configuration**:
- Version: 3.x with v4 future flag enabled
- Theme: Classic preset with default Infima CSS framework
- Custom CSS: Basic color overrides in `src/css/custom.css`
- Navbar: 3 module links + blog + GitHub
- Sidebars: 3 configured sidebars (tutorialSidebar, isaacNavigationSidebar, module4Sidebar)
- Color modes: Light and dark with system preference respect
- Prism themes: GitHub (light), Dracula (dark)

**Current Styling**:
- Primary color: Green (#2e8555 light, #25c2a0 dark)
- Default Infima variables (minimal customization)
- No custom typography settings
- Basic responsive behavior (Docusaurus defaults)
- Minimal branding (generic "My Site" title/tagline)

**Content Structure**:
- 4 main modules (Tutorial, Isaac Navigation, Digital Twin, Module 4 VLA)
- Hierarchical sidebar navigation with categories
- 24+ documentation pages in Module 4 alone
- Blog functionality enabled
- Homepage with basic features component

### Technology Stack

**Framework & Tools**:
- Docusaurus 3.x (React-based static site generator)
- Infima CSS framework (included with Docusaurus)
- CSS Custom Properties (CSS Variables) for theming
- TypeScript configuration
- Prism React Renderer for syntax highlighting
- React 18+ components
- MDX for content authoring

**Build & Deployment**:
- npm build system
- Static site generation
- Future v4 compatibility flag enabled

**Browser Support**:
- Modern browsers (Chrome, Firefox, Safari, Edge - last 2 versions)
- Progressive enhancement approach
- Responsive design (320px - 2560px+)

### Integration Points

1. **`docusaurus.config.ts`**: Site configuration, theme settings, navbar, footer
2. **`src/css/custom.css`**: Global CSS custom properties and overrides
3. **`sidebars.ts`**: Navigation structure definition
4. **Infima CSS Framework**: Base styling system with CSS variables
5. **Theme Components**: Swizzlable React components for deep customization
6. **MDX Content**: Markdown documentation with custom styling hooks

### Known Constraints

- Must maintain Docusaurus v3.x framework (no migration)
- Cannot modify existing content files or structure
- Must preserve all current navigation paths and URLs
- Custom CSS limited to Infima variable overrides and CSS modules
- Component swizzling should be minimal (maintainability concern)
- Bundle size budget: < 500KB JavaScript
- Build time: < 30 seconds for incremental builds

## Phase 0: Research & Design System Definition

### Research Task 1: Modern Documentation Design Patterns

**Objective**: Identify best practices for technical documentation site design

**Key Questions**:
1. What color palettes work best for educational technical content?
2. What typography combinations enhance readability for code-heavy documentation?
3. How do successful documentation sites structure navigation?
4. What are common responsive breakpoints for documentation sites?

**Findings**:

**Color Palette Recommendations**:
- **Educational Tech Sites**: Prefer blue/purple hues (convey trust, intelligence, creativity)
- **Accent Colors**: Use contrasting warm colors for CTAs and highlights
- **Neutral Scale**: 7-9 shades of gray for text, borders, backgrounds
- **Semantic Colors**: Success (green), warning (yellow/orange), error (red), info (blue)

**Decision**:
- **Primary**: Deep blue (#1e40af to #3b82f6) - professional, trustworthy
- **Secondary**: Purple accent (#8b5cf6) - innovation, AI/robotics theme
- **Neutral**: Gray scale from #f9fafb (lightest) to #111827 (darkest)
- **Success**: #10b981, **Warning**: #f59e0b, **Error**: #ef4444, **Info**: #3b82f6

**Rationale**: Blue conveys professionalism and trust essential for educational content. Purple accent reinforces AI/robotics theme while providing visual interest. Comprehensive neutral scale enables proper hierarchy.

**Typography Recommendations**:
- **Body Text**: Sans-serif, 16-18px, 1.6-1.8 line-height
- **Code**: Monospace with ligature support, 14-16px
- **Headings**: Clear hierarchy with size scaling (2.5rem, 2rem, 1.5rem, 1.25rem, 1.125rem)
- **System Fonts**: Prioritize for performance (-apple-system, BlinkMacSystemFont, 'Segoe UI', etc.)

**Decision**:
- **Body**: System font stack with fallback to sans-serif
- **Headings**: Same family, bolder weights (600-700)
- **Code**: 'JetBrains Mono', 'Fira Code', or system monospace stack
- **Sizes**: 17px body (mobile), 18px body (desktop)

**Rationale**: System fonts eliminate external requests (performance). Slightly larger than default improves readability for long-form technical content.

### Research Task 2: Docusaurus Theming Architecture

**Objective**: Understand best practices for Docusaurus customization

**Key Findings**:

**Theming Approach Options**:
1. **CSS Variables Only** (Recommended for this project):
   - Override Infima variables in custom.css
   - Pros: Maintainable, update-safe, performant
   - Cons: Limited to what Infima exposes

2. **Component Swizzling**:
   - Replace React components entirely
   - Pros: Complete control
   - Cons: Update brittleness, maintenance burden

3. **CSS Modules**:
   - Scoped styles for specific components
   - Pros: No conflicts, modular
   - Cons: More files to manage

**Decision**: Primarily use CSS Variables (approach #1) with CSS Modules for specific enhancements. Avoid component swizzling except for non-core components if absolutely necessary.

**Rationale**: CSS variables provide 80% of needed customization with minimal maintenance burden. This aligns with project constraints on maintainability.

**Infima Variable Groups**:
- **Colors**: Primary, secondary, success, warning, danger, info (with shades)
- **Typography**: Font families, sizes, weights, line heights
- **Spacing**: Padding, margin scales (0.25rem increments)
- **Layout**: Container widths, sidebar widths, breakpoints
- **Effects**: Shadows, borders, transitions, animations

### Research Task 3: Responsive Design Strategy

**Objective**: Define breakpoint system and responsive behavior

**Standard Breakpoints** (Infima defaults):
- **Mobile**: < 768px
- **Tablet**: 768px - 996px
- **Desktop**: 997px - 1279px
- **Wide**: >= 1280px

**Decision**: Use Infima breakpoints with custom behavior at each level

**Responsive Behaviors**:

**Mobile (< 768px)**:
- Hamburger navigation menu
- Full-width content (no sidebar visible by default)
- Larger touch targets (min 44x44px)
- Simplified header (icon only or shorter title)
- Stacked footer columns
- Single column layouts

**Tablet (768px - 996px)**:
- Collapsible sidebar (toggleable)
- Reduced font sizes slightly from desktop
- 2-column footer
- Comfortable touch targets still maintained

**Desktop (997px+)**:
- Persistent sidebar navigation
- Optimal reading width (60-80 characters)
- 3-column footer
- Hover states for interactive elements
- Full feature set visible

**Wide (>= 1280px)**:
- Content max-width to maintain readability
- Additional whitespace on sides
- Potential for "table of contents" in right sidebar

### Research Task 4: Accessibility Best Practices

**Objective**: Ensure WCAG 2.1 AA compliance strategy

**Key Requirements**:

**Color Contrast**:
- Body text: 4.5:1 minimum
- Large text (18px+): 3:1 minimum
- UI components: 3:1 minimum
- Tool: Use WebAIM Contrast Checker

**Keyboard Navigation**:
- All interactive elements in tab order
- Visible focus indicators (outline or ring)
- Skip-to-content link
- Keyboard shortcuts documented

**Semantic HTML**:
- Proper heading hierarchy (h1→h2→h3, no skips)
- Landmark regions (header, nav, main, aside, footer)
- ARIA labels where semantic HTML insufficient
- Alt text for images

**Screen Reader Support**:
- ARIA live regions for dynamic content
- Descriptive link text (no "click here")
- Form labels properly associated
- Status messages announced

**Decision**: Implement comprehensive accessibility checklist during implementation with automated testing (axe DevTools) and manual verification.

### Research Task 5: Performance Optimization Techniques

**Objective**: Define performance budget and optimization strategy

**Performance Budget**:
- First Contentful Paint (FCP): < 1.5s
- Largest Contentful Paint (LCP): < 2.5s
- Total Blocking Time (TBT): < 300ms
- Cumulative Layout Shift (CLS): < 0.1
- Time to Interactive (TTI): < 3.5s
- Bundle size: < 500KB JS

**Optimization Techniques**:

**Asset Optimization**:
- Minify CSS/JS (Docusaurus handles automatically)
- Optimize images (WebP with fallbacks)
- Lazy load images below fold
- Use modern image formats with srcset

**CSS Optimization**:
- Minimize custom CSS (leverage Infima)
- Avoid complex selectors
- Use CSS containment where appropriate
- Prefer CSS transforms for animations

**Font Loading**:
- Use system fonts (no external font loading)
- If custom fonts needed: font-display: swap
- Subset fonts to needed characters

**Caching Strategy**:
- Static assets with long cache headers
- Service worker for offline capability (Docusaurus PWA plugin)

**Code Splitting**:
- Docusaurus automatic route-based splitting
- Lazy load non-critical components

## Phase 1: Design System & Visual Identity

### 1.1 Color Palette Definition

**Primary Colors**:
```css
:root {
  /* Primary Blue - Main brand color */
  --ifm-color-primary: #3b82f6;
  --ifm-color-primary-dark: #2563eb;
  --ifm-color-primary-darker: #1d4ed8;
  --ifm-color-primary-darkest: #1e40af;
  --ifm-color-primary-light: #60a5fa;
  --ifm-color-primary-lighter: #93c5fd;
  --ifm-color-primary-lightest: #dbeafe;

  /* Secondary Purple - Accent color */
  --ifm-color-secondary: #8b5cf6;
  --ifm-color-secondary-dark: #7c3aed;
  --ifm-color-secondary-darker: #6d28d9;
  --ifm-color-secondary-darkest: #5b21b6;
  --ifm-color-secondary-light: #a78bfa;
  --ifm-color-secondary-lighter: #c4b5fd;
  --ifm-color-secondary-lightest: #ede9fe;
}

[data-theme='dark'] {
  --ifm-color-primary: #60a5fa;
  --ifm-color-primary-dark: #3b82f6;
  --ifm-color-primary-darker: #2563eb;
  --ifm-color-primary-darkest: #1d4ed8;
  --ifm-color-primary-light: #93c5fd;
  --ifm-color-primary-lighter: #bfdbfe;
  --ifm-color-primary-lightest: #dbeafe;

  --ifm-color-secondary: #a78bfa;
  --ifm-color-secondary-dark: #8b5cf6;
  --ifm-color-secondary-darker: #7c3aed;
  --ifm-color-secondary-darkest: #6d28d9;
  --ifm-color-secondary-light: #c4b5fd;
  --ifm-color-secondary-lighter: #ddd6fe;
  --ifm-color-secondary-lightest: #ede9fe;
}
```

**Semantic Colors**:
```css
:root {
  --ifm-color-success: #10b981;
  --ifm-color-info: #3b82f6;
  --ifm-color-warning: #f59e0b;
  --ifm-color-danger: #ef4444;
}
```

**Neutral Scale** (for text, borders, backgrounds):
```css
:root {
  --ifm-color-gray-0: #ffffff;
  --ifm-color-gray-50: #f9fafb;
  --ifm-color-gray-100: #f3f4f6;
  --ifm-color-gray-200: #e5e7eb;
  --ifm-color-gray-300: #d1d5db;
  --ifm-color-gray-400: #9ca3af;
  --ifm-color-gray-500: #6b7280;
  --ifm-color-gray-600: #4b5563;
  --ifm-color-gray-700: #374151;
  --ifm-color-gray-800: #1f2937;
  --ifm-color-gray-900: #111827;
}
```

### 1.2 Typography System

**Font Stack**:
```css
:root {
  --ifm-font-family-base: -apple-system, BlinkMacSystemFont, 'Segoe UI',
    Roboto, 'Helvetica Neue', Arial, sans-serif;
  --ifm-font-family-monospace: 'SF Mono', 'Monaco', 'Inconsolata',
    'Fira Code', 'Droid Sans Mono', 'Source Code Pro', monospace;
}
```

**Font Sizes**:
```css
:root {
  --ifm-font-size-base: 17px; /* Mobile */
  --ifm-h1-font-size: 2.5rem;    /* 40px */
  --ifm-h2-font-size: 2rem;      /* 32px */
  --ifm-h3-font-size: 1.5rem;    /* 24px */
  --ifm-h4-font-size: 1.25rem;   /* 20px */
  --ifm-h5-font-size: 1.125rem;  /* 18px */
  --ifm-h6-font-size: 1rem;      /* 16px */
  --ifm-code-font-size: 0.875em; /* 14.875px at 17px base */
}

@media (min-width: 997px) {
  :root {
    --ifm-font-size-base: 18px; /* Desktop */
  }
}
```

**Line Heights**:
```css
:root {
  --ifm-line-height-base: 1.7;
  --ifm-heading-line-height: 1.3;
  --ifm-code-line-height: 1.5;
}
```

**Font Weights**:
```css
:root {
  --ifm-font-weight-light: 300;
  --ifm-font-weight-normal: 400;
  --ifm-font-weight-semibold: 600;
  --ifm-font-weight-bold: 700;

  --ifm-heading-font-weight: 600;
}
```

### 1.3 Spacing System

**Base Unit**: 0.25rem (4px)

```css
:root {
  --ifm-spacing-horizontal: 1rem;    /* 16px */
  --ifm-spacing-vertical: 1rem;      /* 16px */

  --ifm-global-spacing: 1.5rem;      /* 24px */
  --ifm-leading: 1.5rem;              /* 24px */

  /* Custom spacing scale */
  --spacing-xs: 0.5rem;   /* 8px */
  --spacing-sm: 0.75rem;  /* 12px */
  --spacing-md: 1rem;     /* 16px */
  --spacing-lg: 1.5rem;   /* 24px */
  --spacing-xl: 2rem;     /* 32px */
  --spacing-2xl: 3rem;    /* 48px */
  --spacing-3xl: 4rem;    /* 64px */
}
```

### 1.4 Layout Specifications

**Content Width**:
```css
:root {
  --ifm-container-width: 1140px;
  --ifm-container-width-xl: 1320px;

  /* Reading width (for article content) */
  --content-max-width: 800px; /* ~80 characters at 18px */
}
```

**Sidebar Width**:
```css
:root {
  --doc-sidebar-width: 300px !important;
}

@media (min-width: 1440px) {
  :root {
    --doc-sidebar-width: 320px !important;
  }
}
```

### 1.5 Visual Effects

**Shadows**:
```css
:root {
  --ifm-global-shadow-lw: 0 1px 3px 0 rgba(0, 0, 0, 0.1),
                          0 1px 2px 0 rgba(0, 0, 0, 0.06);
  --ifm-global-shadow-md: 0 4px 6px -1px rgba(0, 0, 0, 0.1),
                          0 2px 4px -1px rgba(0, 0, 0, 0.06);
  --ifm-global-shadow-tl: 0 10px 15px -3px rgba(0, 0, 0, 0.1),
                          0 4px 6px -2px rgba(0, 0, 0, 0.05);
}
```

**Border Radius**:
```css
:root {
  --ifm-global-radius: 0.5rem;      /* 8px */
  --ifm-code-border-radius: 0.375rem; /* 6px */
  --ifm-button-border-radius: 0.5rem;
}
```

**Transitions**:
```css
:root {
  --ifm-transition-fast: 150ms ease-in-out;
  --ifm-transition-slow: 300ms ease-in-out;
  --ifm-hover-overlay: rgba(0, 0, 0, 0.05);
}

[data-theme='dark'] {
  --ifm-hover-overlay: rgba(255, 255, 255, 0.05);
}
```

## Phase 2: Component Customization Plan

### 2.1 Navbar Enhancements

**Changes**:
1. Update site title and tagline
2. Add logo (if available)
3. Enhance visual hierarchy
4. Improve mobile menu UX
5. Add search prominence

**Configuration Updates** (`docusaurus.config.ts`):
```typescript
navbar: {
  title: 'AI-Native Book',
  hideOnScroll: false,
  logo: {
    alt: 'AI-Native Book Logo',
    src: 'img/logo.svg',
    srcDark: 'img/logo-dark.svg',
    width: 32,
    height: 32,
  },
  items: [
    // ... existing items
  ],
}
```

**CSS Customization** (`custom.css`):
```css
/* Navbar styling */
.navbar {
  box-shadow: var(--ifm-global-shadow-lw);
  backdrop-filter: blur(8px);
  background-color: rgba(255, 255, 255, 0.95);
}

[data-theme='dark'] .navbar {
  background-color: rgba(24, 24, 27, 0.95);
}

.navbar__title {
  font-weight: var(--ifm-font-weight-semibold);
  font-size: 1.125rem;
}

.navbar__link {
  font-weight: var(--ifm-font-weight-normal);
  transition: color var(--ifm-transition-fast);
}

.navbar__link:hover {
  color: var(--ifm-color-primary);
}

/* Mobile menu improvements */
@media (max-width: 996px) {
  .navbar-sidebar {
    background: var(--ifm-background-color);
  }

  .navbar-sidebar__brand {
    padding: var(--spacing-lg);
  }

  .menu__link {
    padding: var(--spacing-sm) var(--spacing-lg);
    font-size: 1rem;
  }
}
```

### 2.2 Sidebar Navigation Enhancements

**Changes**:
1. Improve visual hierarchy
2. Add icons for categories (optional)
3. Better active state indication
4. Smooth expand/collapse animations
5. Sticky positioning for long sidebars

**CSS Customization**:
```css
/* Sidebar container */
.theme-doc-sidebar-container {
  border-right: 1px solid var(--ifm-color-gray-200);
}

[data-theme='dark'] .theme-doc-sidebar-container {
  border-right-color: var(--ifm-color-gray-800);
}

/* Sidebar menu */
.menu {
  padding: var(--spacing-lg) var(--spacing-md);
}

.menu__link {
  border-radius: var(--ifm-global-radius);
  transition: background-color var(--ifm-transition-fast);
  font-size: 0.9375rem;
}

.menu__link:hover {
  background-color: var(--ifm-hover-overlay);
}

/* Active item */
.menu__link--active {
  background-color: var(--ifm-color-primary-lightest);
  color: var(--ifm-color-primary-darker);
  font-weight: var(--ifm-font-weight-semibold);
}

[data-theme='dark'] .menu__link--active {
  background-color: rgba(59, 130, 246, 0.1);
  color: var(--ifm-color-primary-lighter);
}

/* Category headings */
.menu__list-item-collapsible {
  font-weight: var(--ifm-font-weight-semibold);
}

.menu__caret {
  transition: transform var(--ifm-transition-fast);
}

/* Expand/collapse animation */
.menu__list {
  transition: height var(--ifm-transition-slow);
}
```

### 2.3 Content Area Enhancements

**Changes**:
1. Optimal reading width
2. Improved heading spacing
3. Better code block styling
4. Enhanced blockquote design
5. Table of contents positioning

**CSS Customization**:
```css
/* Main content wrapper */
.theme-doc-markdown {
  max-width: var(--content-max-width);
}

/* Heading improvements */
article h1 {
  font-size: var(--ifm-h1-font-size);
  margin-bottom: var(--spacing-lg);
  line-height: var(--ifm-heading-line-height);
  color: var(--ifm-color-gray-900);
}

[data-theme='dark'] article h1 {
  color: var(--ifm-color-gray-50);
}

article h2 {
  margin-top: var(--spacing-2xl);
  margin-bottom: var(--spacing-lg);
  padding-bottom: var(--spacing-sm);
  border-bottom: 1px solid var(--ifm-color-gray-200);
}

/* Paragraph spacing */
article p {
  margin-bottom: var(--spacing-lg);
  line-height: var(--ifm-line-height-base);
}

/* Code blocks */
.prism-code {
  border-radius: var(--ifm-code-border-radius);
  font-size: var(--ifm-code-font-size);
  line-height: var(--ifm-code-line-height);
}

/* Inline code */
code {
  background-color: var(--ifm-color-gray-100);
  color: var(--ifm-color-gray-900);
  padding: 0.125rem 0.375rem;
  border-radius: 0.25rem;
  font-size: 0.9em;
}

[data-theme='dark'] code {
  background-color: var(--ifm-color-gray-800);
  color: var(--ifm-color-gray-100);
}

/* Blockquotes */
blockquote {
  border-left: 4px solid var(--ifm-color-primary);
  background-color: var(--ifm-color-primary-lightest);
  padding: var(--spacing-md) var(--spacing-lg);
  border-radius: var(--ifm-global-radius);
  margin: var(--spacing-lg) 0;
}

[data-theme='dark'] blockquote {
  background-color: rgba(59, 130, 246, 0.05);
}

/* Tables */
table {
  border-collapse: collapse;
  width: 100%;
  margin: var(--spacing-lg) 0;
}

table thead tr {
  background-color: var(--ifm-color-gray-50);
}

[data-theme='dark'] table thead tr {
  background-color: var(--ifm-color-gray-900);
}

table th,
table td {
  padding: var(--spacing-sm) var(--spacing-md);
  border: 1px solid var(--ifm-color-gray-200);
}

[data-theme='dark'] table th,
[data-theme='dark'] table td {
  border-color: var(--ifm-color-gray-800);
}
```

### 2.4 Footer Customization

**Configuration Updates** (`docusaurus.config.ts`):
```typescript
footer: {
  style: 'dark',
  logo: {
    alt: 'AI-Native Book Logo',
    src: 'img/logo.svg',
    width: 160,
  },
  links: [
    {
      title: 'Learn',
      items: [
        { label: 'Tutorial', to: '/docs/intro' },
        { label: 'Isaac Navigation', to: '/docs/isaac-navigation-systems' },
        { label: 'Module 4: VLA', to: '/docs/module-4-vla-planning-capstone' },
      ],
    },
    {
      title: 'Resources',
      items: [
        { label: 'Blog', to: '/blog' },
        { label: 'GitHub', href: 'https://github.com/...' },
      ],
    },
    {
      title: 'About',
      items: [
        { label: 'About This Project', to: '/about' },
        { label: 'License', to: '/license' },
      ],
    },
  ],
  copyright: `© ${new Date().getFullYear()} AI-Native Book. Built with Docusaurus.`,
}
```

**CSS Customization**:
```css
/* Footer styling */
.footer {
  padding: var(--spacing-3xl) 0 var(--spacing-xl);
  background-color: var(--ifm-color-gray-900);
}

.footer__title {
  font-weight: var(--ifm-font-weight-semibold);
  color: var(--ifm-color-gray-100);
}

.footer__link-item {
  color: var(--ifm-color-gray-400);
  transition: color var(--ifm-transition-fast);
}

.footer__link-item:hover {
  color: var(--ifm-color-primary-light);
}

.footer__copyright {
  color: var(--ifm-color-gray-500);
  font-size: 0.875rem;
  margin-top: var(--spacing-lg);
}
```

### 2.5 Button & Interactive Elements

**CSS Customization**:
```css
/* Button base */
.button {
  border-radius: var(--ifm-button-border-radius);
  font-weight: var(--ifm-font-weight-semibold);
  transition: all var(--ifm-transition-fast);
  border: none;
}

/* Primary button */
.button--primary {
  background-color: var(--ifm-color-primary);
  color: white;
}

.button--primary:hover {
  background-color: var(--ifm-color-primary-dark);
  transform: translateY(-1px);
  box-shadow: var(--ifm-global-shadow-md);
}

/* Secondary button */
.button--secondary {
  background-color: var(--ifm-color-secondary);
  color: white;
}

.button--secondary:hover {
  background-color: var(--ifm-color-secondary-dark);
}

/* Outline button */
.button--outline {
  border: 2px solid var(--ifm-color-primary);
  color: var(--ifm-color-primary);
  background-color: transparent;
}

.button--outline:hover {
  background-color: var(--ifm-color-primary);
  color: white;
}

/* Link hover states */
a {
  text-decoration: none;
  color: var(--ifm-color-primary);
  transition: color var(--ifm-transition-fast);
}

a:hover {
  color: var(--ifm-color-primary-dark);
  text-decoration: underline;
}

/* Focus indicators for accessibility */
*:focus-visible {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 2px;
  border-radius: 0.125rem;
}
```

### 2.6 Card Components

**CSS Module** (`src/components/Card/Card.module.css`):
```css
.card {
  background: var(--ifm-card-background-color);
  border: 1px solid var(--ifm-color-gray-200);
  border-radius: var(--ifm-global-radius);
  padding: var(--spacing-lg);
  transition: all var(--ifm-transition-fast);
  box-shadow: var(--ifm-global-shadow-lw);
}

.card:hover {
  transform: translateY(-2px);
  box-shadow: var(--ifm-global-shadow-md);
  border-color: var(--ifm-color-primary-light);
}

[data-theme='dark'] .card {
  background: var(--ifm-color-gray-900);
  border-color: var(--ifm-color-gray-800);
}

.cardTitle {
  font-size: 1.25rem;
  font-weight: var(--ifm-font-weight-semibold);
  margin-bottom: var(--spacing-sm);
  color: var(--ifm-color-gray-900);
}

[data-theme='dark'] .cardTitle {
  color: var(--ifm-color-gray-100);
}

.cardDescription {
  color: var(--ifm-color-gray-600);
  line-height: 1.6;
}

[data-theme='dark'] .cardDescription {
  color: var(--ifm-color-gray-400);
}
```

## Phase 3: Responsive Implementation

### 3.1 Mobile Optimizations (< 768px)

**Key Changes**:
```css
@media (max-width: 768px) {
  :root {
    --ifm-font-size-base: 17px;
    --ifm-spacing-horizontal: 1rem;
  }

  /* Touch-friendly targets */
  .button,
  .navbar__link,
  .menu__link {
    min-height: 44px;
    min-width: 44px;
  }

  /* Simplified navigation */
  .navbar__items {
    gap: var(--spacing-sm);
  }

  /* Full-width content */
  .container {
    padding-left: var(--spacing-md);
    padding-right: var(--spacing-md);
  }

  /* Code blocks with horizontal scroll */
  .prism-code {
    font-size: 14px;
  }

  /* Simplified tables */
  table {
    font-size: 0.875rem;
  }

  table th,
  table td {
    padding: var(--spacing-xs) var(--spacing-sm);
  }
}
```

### 3.2 Tablet Optimizations (768px - 996px)

**Key Changes**:
```css
@media (min-width: 768px) and (max-width: 996px) {
  :root {
    --ifm-font-size-base: 17px;
    --doc-sidebar-width: 280px;
  }

  /* Collapsible sidebar */
  .theme-doc-sidebar-container {
    position: sticky;
    top: calc(var(--ifm-navbar-height) + 1rem);
  }

  /* Footer 2-column layout */
  .footer__links {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: var(--spacing-xl);
  }
}
```

### 3.3 Desktop Optimizations (997px+)

**Key Changes**:
```css
@media (min-width: 997px) {
  :root {
    --ifm-font-size-base: 18px;
  }

  /* Persistent sidebar */
  .theme-doc-sidebar-container {
    display: block !important;
  }

  /* Optimal content width */
  .theme-doc-markdown {
    max-width: var(--content-max-width);
  }

  /* Footer 3-column layout */
  .footer__links {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: var(--spacing-2xl);
  }

  /* Table of contents on right */
  .table-of-contents {
    position: sticky;
    top: calc(var(--ifm-navbar-height) + 2rem);
    max-height: calc(100vh - var(--ifm-navbar-height) - 4rem);
    overflow-y: auto;
  }
}
```

### 3.4 Wide Screen Optimizations (>= 1280px)

**Key Changes**:
```css
@media (min-width: 1280px) {
  :root {
    --doc-sidebar-width: 320px;
    --ifm-container-width-xl: 1320px;
  }

  /* Additional whitespace */
  .main-wrapper {
    padding: 0 var(--spacing-xl);
  }

  /* Larger headings */
  article h1 {
    font-size: 2.75rem;
  }

  article h2 {
    font-size: 2.125rem;
  }
}

@media (min-width: 1440px) {
  /* Max content width to maintain readability */
  .container {
    max-width: 1400px;
  }

  /* Optional: Show table of contents in right sidebar */
  .docMainContainer {
    display: grid;
    grid-template-columns: minmax(0, var(--content-max-width)) 240px;
    gap: var(--spacing-2xl);
  }
}
```

## Phase 4: Accessibility Implementation

### 4.1 Color Contrast Verification

**Action Items**:
1. Test all color combinations against WCAG 2.1 AA standards
2. Use WebAIM Contrast Checker for verification
3. Ensure minimum ratios:
   - Body text: 4.5:1
   - Large text (18px+/14px bold+): 3:1
   - UI components: 3:1

**Testing Checklist**:
- [ ] Primary button text on primary background
- [ ] Secondary button text on secondary background
- [ ] Body text on white/dark backgrounds
- [ ] Link colors in both themes
- [ ] Code text in code blocks
- [ ] Navbar text on navbar background
- [ ] Footer text on footer background
- [ ] Form inputs and labels
- [ ] Alert/notification text

### 4.2 Keyboard Navigation

**Enhancements**:
```css
/* Skip to content link */
.skip-to-content {
  position: absolute;
  left: -9999px;
  z-index: 999;
  padding: var(--spacing-md);
  background-color: var(--ifm-color-primary);
  color: white;
  text-decoration: none;
  border-radius: var(--ifm-global-radius);
}

.skip-to-content:focus {
  left: var(--spacing-md);
  top: var(--spacing-md);
}

/* Focus indicators */
*:focus-visible {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 2px;
}

/* Focus within for containers */
.menu__list-item:focus-within {
  background-color: var(--ifm-hover-overlay);
}
```

**Action Items**:
1. Add skip-to-content link at page start
2. Ensure all interactive elements in tab order
3. Test keyboard navigation through entire site
4. Verify focus indicators visible in both themes
5. Test with screen reader (NVDA/JAWS/VoiceOver)

### 4.3 Semantic HTML & ARIA

**Configuration** (`docusaurus.config.ts`):
```typescript
themeConfig: {
  metadata: [
    {name: 'description', content: 'AI-Native Book: Learn about humanoid robotics, VLA models, and motion planning'},
    {name: 'keywords', content: 'AI, robotics, VLA, machine learning, education'},
  ],
}
```

**HTML Structure Verification**:
- [ ] Proper heading hierarchy (no skipped levels)
- [ ] Landmark regions (header, nav, main, aside, footer)
- [ ] Alt text for all images
- [ ] Form labels properly associated
- [ ] ARIA labels for icon-only buttons
- [ ] ARIA live regions for dynamic content

### 4.4 Reduced Motion Support

**CSS**:
```css
/* Respect user preference for reduced motion */
@media (prefers-reduced-motion: reduce) {
  *,
  *::before,
  *::after {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
    scroll-behavior: auto !important;
  }
}
```

## Phase 5: Performance Optimization

### 5.1 CSS Optimization

**Action Items**:
1. Minimize custom CSS (leverage Infima variables)
2. Use CSS containment for independent sections
3. Avoid complex selectors (max 3 levels deep)
4. Use CSS transforms for animations (GPU accelerated)
5. Inline critical CSS (Docusaurus handles automatically)

**CSS Performance Rules**:
```css
/* Use containment for isolated components */
.card {
  contain: content;
}

.sidebar {
  contain: layout style;
}

/* Prefer transforms for animations */
.button:hover {
  transform: translateY(-1px); /* GPU accelerated */
  /* Avoid: margin-top: -1px; */ /* Layout thrashing */
}

/* Use will-change sparingly */
.animated-element {
  will-change: transform;
}

.animated-element.done {
  will-change: auto; /* Remove after animation */
}
```

### 5.2 Asset Optimization

**Action Items**:
1. Optimize logo/images (WebP with PNG fallback)
2. Use appropriate image sizes (srcset)
3. Lazy load images below fold
4. Compress CSS/JS (Docusaurus handles)
5. Use system fonts (no external font loading)

**Image Optimization Example**:
```html
<picture>
  <source srcSet="logo.webp" type="image/webp" />
  <img src="logo.png" alt="AI-Native Book Logo" width="160" height="40" loading="lazy" />
</picture>
```

### 5.3 Lighthouse Performance Targets

**Metrics to Monitor**:
- First Contentful Paint (FCP): < 1.5s
- Largest Contentful Paint (LCP): < 2.5s
- Total Blocking Time (TBT): < 300ms
- Cumulative Layout Shift (CLS): < 0.1
- Speed Index: < 3.0s

**Optimization Checklist**:
- [ ] Minimize unused CSS
- [ ] Defer non-critical JavaScript
- [ ] Optimize images (size, format, lazy loading)
- [ ] Use efficient cache policy
- [ ] Minimize main-thread work
- [ ] Reduce JavaScript execution time
- [ ] Avoid enormous network payloads
- [ ] Serve static assets with efficient cache policy

## Implementation Checklist

### Phase 1: Design System ✓
- [ ] Define color palette (light and dark themes)
- [ ] Establish typography system
- [ ] Create spacing scale
- [ ] Define layout specifications
- [ ] Document visual effects (shadows, borders, transitions)

### Phase 2: Component Customization
- [ ] Update navbar (title, logo, styling)
- [ ] Enhance sidebar navigation (visual hierarchy, active states)
- [ ] Improve content area (reading width, headings, code blocks)
- [ ] Customize footer (links, copyright, styling)
- [ ] Style buttons and interactive elements
- [ ] Create card components

### Phase 3: Responsive Design
- [ ] Implement mobile optimizations (< 768px)
- [ ] Implement tablet optimizations (768px - 996px)
- [ ] Implement desktop optimizations (997px+)
- [ ] Implement wide screen optimizations (>= 1280px)
- [ ] Test on real devices

### Phase 4: Accessibility
- [ ] Verify color contrast ratios
- [ ] Implement keyboard navigation enhancements
- [ ] Add skip-to-content link
- [ ] Verify semantic HTML and ARIA labels
- [ ] Add reduced motion support
- [ ] Test with screen readers

### Phase 5: Performance
- [ ] Optimize CSS (minimize, containment)
- [ ] Optimize assets (images, fonts)
- [ ] Run Lighthouse audits
- [ ] Fix performance issues
- [ ] Verify bundle size < 500KB

### Phase 6: Testing & Validation
- [ ] Cross-browser testing (Chrome, Firefox, Safari, Edge)
- [ ] Device testing (mobile, tablet, desktop)
- [ ] Accessibility testing (automated + manual)
- [ ] Performance testing (Lighthouse, WebPageTest)
- [ ] User acceptance testing

## Risk Mitigation

| Risk | Impact | Probability | Mitigation |
| ---- | ------ | ----------- | ---------- |
| CSS variable browser support issues | Low | Low | Use fallback values; target modern browsers only |
| Color contrast failures in dark mode | Medium | Medium | Test all color combinations; use contrast checker tools |
| Performance regression with custom styles | Medium | Low | Monitor bundle size; use CSS containment; run Lighthouse audits |
| Responsive design breaks on edge devices | Low | Medium | Test on real devices; use progressive enhancement |
| Custom styling conflicts with future Docusaurus updates | Medium | Low | Minimize custom CSS; use Infima variables; avoid component swizzling |

## Success Criteria

Implementation is complete when:

1. ✅ All visual design elements applied (colors, typography, spacing)
2. ✅ All components customized (navbar, sidebar, content, footer, buttons)
3. ✅ Responsive design works on all breakpoints (mobile to wide screen)
4. ✅ WCAG 2.1 AA accessibility compliance verified
5. ✅ Lighthouse scores meet targets (90+ Performance, 95+ Accessibility)
6. ✅ Bundle size under 500KB
7. ✅ Cross-browser compatibility confirmed
8. ✅ User acceptance testing passed

## Next Steps

After planning approval:
1. Run `/sp.tasks` to generate detailed implementation tasks
2. Begin Phase 1 implementation (Design System in custom.css)
3. Iteratively implement and test each component
4. Conduct accessibility audits after each phase
5. Run performance tests before final deployment

---

**Plan Status**: Ready for Implementation
**Estimated Effort**: 2-3 weeks full-time (40-60 hours)
**Dependencies**: None (all work within Docusaurus framework)
