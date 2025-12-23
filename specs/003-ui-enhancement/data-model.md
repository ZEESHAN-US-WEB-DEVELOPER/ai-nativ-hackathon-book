# Data Model: UI Enhancement

**Feature**: 003-ui-enhancement
**Date**: 2025-12-23

## Entities

### Theme Configuration

Represents the visual design system including colors, typography, spacing, and effects.

**Attributes**:
- Color palette (primary, secondary, semantic colors with shades)
- Typography settings (font families, sizes, weights, line heights)
- Spacing scale (base unit, specific values for different use cases)
- Visual effects (shadows, border radius, transitions)
- Breakpoint definitions (mobile, tablet, desktop, wide)

**Storage**: CSS Custom Properties in `:root` and `[data-theme='dark']` scopes

**Relationships**: Used by all UI components for consistent styling

**Validation Rules**:
- All colors must meet WCAG 2.1 AA contrast requirements
- Font sizes must be >= 16px for body text
- Spacing values must use rem units for accessibility
- Transition durations must be <= 300ms to feel responsive

**State Transitions**:
- Light mode ↔ Dark mode (user toggles theme)
- System preference detection (initial state)
- Theme preference persisted in localStorage

### Navigation Structure

Represents the hierarchical organization of documentation content.

**Attributes**:
- Sidebar ID (tutorialSidebar, isaacNavigationSidebar, module4Sidebar)
- Category items (nested hierarchy of sections)
- Document paths (links to content files)
- Labels (display names for navigation items)
- Active/inactive states (current page indication)
- Expand/collapse states (category visibility)

**Storage**: TypeScript configuration in `sidebars.ts`

**Relationships**:
- Referenced by navbar items (sidebar ID)
- Maps to content file structure in `docs/`
- Drives breadcrumb generation

**Validation Rules**:
- All document paths must exist in `docs/` directory
- Category hierarchies must be ≤ 3 levels deep for usability
- Labels must be clear and descriptive (no abbreviations without context)

**State Transitions**:
- Collapsed ↔ Expanded (user clicks category)
- Active page updates on navigation
- Mobile: Hidden ↔ Visible (hamburger menu toggle)

### User Preferences

Represents stored user interface settings.

**Attributes**:
- Color mode preference (light, dark, or system)
- Sidebar state (open or closed on desktop)
- Font size adjustments (if customizable)
- Reduced motion preference (from system)

**Storage**: Browser localStorage + CSS media queries

**Relationships**: Affects theme configuration rendering

**Validation Rules**:
- Color mode must be one of: 'light', 'dark', 'system'
- Preferences must persist across sessions
- Invalid values revert to defaults

**State Transitions**:
- User changes theme → Update localStorage → Apply CSS variables
- System preference changes → Detect media query → Update if mode is 'system'
- User adjusts font → Update CSS variable → Content reflows

### Content Page

Represents individual documentation pages with associated metadata.

**Attributes**:
- Page title (h1 heading)
- Breadcrumb path (hierarchical location)
- Previous/next links (sequential navigation)
- Table of contents (h2-h4 headings on page)
- Last updated timestamp (from git)
- Reading time estimate (Docusaurus calculated)

**Storage**: MDX frontmatter + auto-generated metadata

**Relationships**:
- Belongs to sidebar navigation structure
- Links to previous and next pages in sequence
- Contains content sections (headings, paragraphs, code blocks)

**Validation Rules**:
- Must have exactly one h1 heading
- Heading hierarchy must not skip levels (h1→h2→h3, not h1→h3)
- Breadcrumb path must reflect sidebar structure
- Table of contents max depth of 3 levels

## Data Flow

### Theme Switching Flow

```
User clicks theme toggle
    ↓
JavaScript updates data-theme attribute on <html>
    ↓
CSS re-evaluates [data-theme='dark'] selector
    ↓
CSS custom properties updated
    ↓
All components re-render with new theme
    ↓
Preference stored in localStorage
```

### Navigation Flow

```
User clicks sidebar item
    ↓
Route changes (React Router)
    ↓
New content page loads
    ↓
Sidebar updates active state
    ↓
Breadcrumbs regenerate
    ↓
Previous/Next links update
    ↓
Scroll to top
```

### Responsive Adaptation Flow

```
Browser window resizes
    ↓
CSS media queries re-evaluate
    ↓
Breakpoint-specific styles apply
    ↓
Layout reflows (sidebar, content, footer)
    ↓
Touch targets adjust (mobile vs desktop)
```

## Component Hierarchy

```
App
├── Navbar
│   ├── Logo
│   ├── NavItems
│   │   ├── DocSidebarLink
│   │   ├── BlogLink
│   │   └── GitHubLink
│   ├── Search
│   └── ColorModeToggle
├── Layout
│   ├── Sidebar (Desktop/Tablet)
│   │   ├── MenuCategories
│   │   │   └── MenuItems
│   │   └── SidebarCollapsible
│   ├── MainContent
│   │   ├── Breadcrumbs
│   │   ├── ArticleContent
│   │   │   ├── Headings (h1-h6)
│   │   │   ├── Paragraphs
│   │   │   ├── CodeBlocks
│   │   │   ├── Tables
│   │   │   ├── Blockquotes
│   │   │   └── Images
│   │   ├── TableOfContents (Wide screens)
│   │   └── PaginationNav (Prev/Next)
│   └── HamburgerMenu (Mobile)
└── Footer
    ├── FooterLogo
    ├── FooterLinks (3 columns)
    └── Copyright
```

## CSS Variable Structure

### Organization

```
:root {
  /* === COLOR PALETTE === */
  --ifm-color-primary-*
  --ifm-color-secondary-*
  --ifm-color-success-*
  --ifm-color-gray-*

  /* === TYPOGRAPHY === */
  --ifm-font-family-base
  --ifm-font-family-monospace
  --ifm-font-size-base
  --ifm-h*-font-size
  --ifm-line-height-base
  --ifm-font-weight-*

  /* === SPACING === */
  --ifm-spacing-horizontal
  --ifm-spacing-vertical
  --spacing-*

  /* === LAYOUT === */
  --ifm-container-width
  --doc-sidebar-width
  --content-max-width

  /* === EFFECTS === */
  --ifm-global-shadow-*
  --ifm-global-radius
  --ifm-transition-*
}

[data-theme='dark'] {
  /* Dark mode overrides */
}

@media (prefers-reduced-motion: reduce) {
  /* Reduced motion overrides */
}

@media (max-width: 768px) {
  /* Mobile overrides */
}

@media (min-width: 997px) {
  /* Desktop overrides */
}
```

## Summary

The data model defines four key entities:

1. **Theme Configuration**: Design tokens (colors, typography, spacing) stored as CSS variables
2. **Navigation Structure**: Sidebar hierarchy defined in TypeScript configuration
3. **User Preferences**: Settings stored in localStorage affecting theme rendering
4. **Content Page**: Individual documentation pages with metadata

These entities interact to create the complete user interface, with clear data flows for theme switching, navigation, and responsive adaptation.
