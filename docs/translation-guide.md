# Translation Guide - AI-Native Book

This guide explains how to translate the AI-Native Book into multiple languages and how to use the translation features.

## 🌍 Supported Languages

The AI-Native Book supports **15 languages** from around the world:

| Language | Code | Native Name | Country | Status |
|----------|------|-------------|---------|--------|
| English | `en` | English | 🇺🇸 USA | ✅ Complete |
| **Urdu** | `ur` | **اردو** | **🇵🇰 Pakistan** | **✅ Partial** |
| Spanish | `es` | Español | 🇪🇸 Spain | 🚧 In Progress |
| Chinese | `zh` | 简体中文 | 🇨🇳 China | 🚧 In Progress |
| Hindi | `hi` | हिन्दी | 🇮🇳 India | 🚧 In Progress |
| Arabic | `ar` | العربية | 🇸🇦 Saudi Arabia | 🚧 In Progress |
| French | `fr` | Français | 🇫🇷 France | 🚧 In Progress |
| German | `de` | Deutsch | 🇩🇪 Germany | 🚧 In Progress |
| Japanese | `ja` | 日本語 | 🇯🇵 Japan | 🚧 In Progress |
| Korean | `ko` | 한국어 | 🇰🇷 South Korea | 🚧 In Progress |
| Portuguese | `pt` | Português | 🇧🇷 Brazil | 🚧 In Progress |
| Russian | `ru` | Русский | 🇷🇺 Russia | 🚧 In Progress |
| Turkish | `tr` | Türkçe | 🇹🇷 Turkey | 🚧 In Progress |
| Italian | `it` | Italiano | 🇮🇹 Italy | 🚧 In Progress |
| Dutch | `nl` | Nederlands | 🇳🇱 Netherlands | 🚧 In Progress |

## 🇵🇰 Special Feature: Urdu Language Support

Urdu has **special emphasis** with a dedicated button for easy access!

### Urdu Button Features

- **🔘 Prominent Button**: Green gradient button with Pakistan flag
- **📍 Fixed Position**: Always visible in top-right corner
- **✨ Animations**: Smooth slide-in, pulse effect when active
- **🎨 Beautiful Typography**: Uses Noto Nastaliq Urdu font (authentic Nastaliq script)
- **↔️ RTL Support**: Right-to-left reading direction
- **✓ Active Indicator**: Shows checkmark when viewing Urdu content

### How to Use the Urdu Button

1. **Click the green button** with "اردو" (top-right corner)
2. **Page switches** to Urdu version automatically
3. **Navigation updates** to show Urdu menu items
4. **Content displays** in beautiful Nastaliq script

## 🎯 How to Switch Languages

### Method 1: Language Dropdown (Navbar)

1. Click the **language icon** in the navbar
2. Select from **15 available languages**
3. Page automatically reloads in selected language

### Method 2: Special Urdu Button

1. Click the **green "اردو" button** (top-right)
2. Instantly switch to Urdu
3. Button shows **✓ checkmark** when active

### Method 3: URL Direct Access

Visit any page with language code prefix:

```
English: https://example.com/docs/intro
Urdu:    https://example.com/ur/docs/intro
Spanish: https://example.com/es/docs/intro
Chinese: https://example.com/zh/docs/intro
```

## 📖 Translation Status by Content

### Fully Translated (Urdu)
- ✅ Homepage navigation
- ✅ Introduction page
- ✅ Module 4 overview
- ✅ Common UI elements (buttons, menus, breadcrumbs)

### Coming Soon
- 🚧 All VLA Models chapter
- 🚧 Planning & Control chapter
- 🚧 Capstone Project chapter
- 🚧 Other modules

## 🛠️ For Translators: How to Add Translations

### Step 1: Create Translation Files

For each language, create translation files in `i18n/[locale]/`:

```bash
# Example: Adding Spanish translations
mkdir -p i18n/es/docusaurus-plugin-content-docs/current
mkdir -p i18n/es/docusaurus-theme-classic

# Copy English content to translate
cp -r docs/* i18n/es/docusaurus-plugin-content-docs/current/
```

### Step 2: Translate UI Elements

Edit `i18n/[locale]/code.json`:

```json
{
  "theme.common.skipToMainContent": {
    "message": "Ir al contenido principal",
    "description": "Skip to main content link"
  },
  "theme.docs.paginator.next": {
    "message": "Siguiente",
    "description": "Next button"
  }
}
```

### Step 3: Translate Content Files

Edit markdown files in `i18n/[locale]/docusaurus-plugin-content-docs/current/`:

```markdown
# Bienvenido al Libro AI-Native

Este libro le enseña sobre IA y robótica...
```

### Step 4: Test Translation

```bash
# Build for specific locale
npm run build -- --locale es

# Start dev server with specific locale
npm run start -- --locale ur
```

## 🎨 RTL (Right-to-Left) Languages

**RTL Languages**: Urdu, Arabic

### Special Considerations

1. **Text Direction**: Automatically switches to RTL
2. **Layout Mirroring**: Navbar, sidebar, and navigation reverse
3. **Code Blocks**: Remain LTR (code is universal)
4. **Numbers**: Remain LTR (123 not ۱۲۳)

### CSS Automatic Adjustments

```css
html[dir="rtl"] .navbar__items {
  flex-direction: row-reverse;
}

html[dir="rtl"] article {
  text-align: right;
}
```

## 📱 Mobile Translation Experience

On mobile devices:
- Language dropdown in navbar menu
- Urdu button in dedicated section
- Responsive font sizes for all languages
- Touch-friendly language switching

## 🌟 Font Optimization

### Google Fonts Used

- **Urdu**: Noto Nastaliq Urdu (authentic Nastaliq script)
- **Arabic**: Noto Sans Arabic
- **Chinese**: Noto Sans SC (Simplified Chinese)
- **Japanese**: Noto Sans JP
- **Korean**: Noto Sans KR
- **Hindi**: Noto Sans Devanagari

### Why Noto Fonts?

- ✅ **Free and open-source**
- ✅ **Professional quality**
- ✅ **Complete character coverage**
- ✅ **Optimized for web**
- ✅ **Consistent design across languages**

## 🚀 Quick Translation Commands

```bash
# Build all locales
npm run build

# Build specific locale
npm run build -- --locale ur

# Start dev server with Urdu
npm run start -- --locale ur

# Write translations for missing strings
npm run write-translations -- --locale ur
```

## 📊 Translation Progress

### Overall Progress

- **English**: 100% (source language)
- **Urdu**: 15% (UI complete, content in progress)
- **Other languages**: 5% (config only)

### Priority Order

1. **Phase 1**: Urdu (special emphasis) - Target: 50%
2. **Phase 2**: Spanish, Hindi, Arabic - Target: 25%
3. **Phase 3**: Chinese, French, German - Target: 25%
4. **Phase 4**: All others - Target: 10%

## 🤝 Contributing Translations

Want to help translate? Here's how:

1. **Choose a language** from the supported list
2. **Fork the repository** on GitHub
3. **Translate content files** in `i18n/[locale]/`
4. **Test your translations** locally
5. **Submit a pull request**

### Translation Guidelines

- **Keep technical terms** in English (e.g., "PyTorch", "ROS", "SLAM")
- **Translate concepts** clearly and accurately
- **Maintain code examples** in original form
- **Use native speakers** to review
- **Test RTL layouts** for Arabic/Urdu

## 📖 Sample Translations

### Button Text Examples

| English | Urdu | Spanish | Chinese |
|---------|------|---------|---------|
| Start Learning | سیکھنا شروع کریں | Comenzar a Aprender | 开始学习 |
| Next Page | اگلا صفحہ | Página Siguiente | 下一页 |
| Previous | پچھلا | Anterior | 上一个 |
| Sign In | سائن ان کریں | Iniciar Sesión | 登录 |

## 🌐 SEO and Internationalization

### Language-Specific Metadata

Each language gets:
- Proper `lang` attribute in HTML
- Localized meta descriptions
- Hreflang tags for SEO
- Language-specific social cards

### URL Structure

```
example.com/           → English (default)
example.com/ur/        → Urdu
example.com/es/        → Spanish
example.com/zh/        → Chinese
example.com/ur/docs/   → Urdu documentation
```

## 🎯 Next Steps

1. **Try the Urdu translation**: Click the green اردو button!
2. **Help translate**: Contribute to your native language
3. **Report issues**: Found a translation error? Let us know!

---

**Ready to reach global audience!** 🌍

With 15 language support, the AI-Native Book can educate learners worldwide in their native languages!
