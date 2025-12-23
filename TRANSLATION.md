# 🌍 Multilingual Translation System

The AI-Native Book supports **15 languages** with special emphasis on **Urdu (اردو)**.

## 🚀 Quick Start

### View in Different Languages

```bash
# English (default)
npm start

# Urdu
npm run start -- --locale ur

# Spanish
npm run start -- --locale es

# Chinese
npm run start -- --locale zh
```

### Access via URL

- English: `http://localhost:3000/`
- Urdu: `http://localhost:3000/ur/`
- Spanish: `http://localhost:3000/es/`
- Chinese: `http://localhost:3000/zh/`

## 🇵🇰 Special Urdu Button

A **dedicated green button** appears on every page for quick Urdu access!

### Features:
- 🟢 **Green gradient** with Pakistan flag (🇵🇰)
- 📍 **Fixed position**: Top-right corner (always visible)
- ✨ **Smooth animations**: Slide-in, hover lift, pulse when active
- **Beautiful font**: Noto Nastaliq Urdu (authentic Nastaliq script)
- ↔️ **RTL support**: Automatic right-to-left layout
- ✓ **Active indicator**: Checkmark when viewing Urdu

### Location:
- **Desktop**: Top-right corner, fixed position
- **Mobile**: Below navbar, centered

## 📋 Supported Languages

| # | Language | Code | Native | Flag | Direction | Font |
|---|----------|------|--------|------|-----------|------|
| 1 | English | `en` | English | 🇺🇸 | LTR | System |
| 2 | **Urdu** | `ur` | **اردو** | **🇵🇰** | **RTL** | **Noto Nastaliq Urdu** |
| 3 | Spanish | `es` | Español | 🇪🇸 | LTR | System |
| 4 | Chinese | `zh` | 简体中文 | 🇨🇳 | LTR | Noto Sans SC |
| 5 | Hindi | `hi` | हिन्दी | 🇮🇳 | LTR | Noto Sans Devanagari |
| 6 | Arabic | `ar` | العربية | 🇸🇦 | RTL | Noto Sans Arabic |
| 7 | French | `fr` | Français | 🇫🇷 | LTR | System |
| 8 | German | `de` | Deutsch | 🇩🇪 | LTR | System |
| 9 | Japanese | `ja` | 日本語 | 🇯🇵 | LTR | Noto Sans JP |
| 10 | Korean | `ko` | 한국어 | 🇰🇷 | LTR | Noto Sans KR |
| 11 | Portuguese | `pt` | Português | 🇧🇷 | LTR | System |
| 12 | Russian | `ru` | Русский | 🇷🇺 | LTR | System |
| 13 | Turkish | `tr` | Türkçe | 🇹🇷 | LTR | System |
| 14 | Italian | `it` | Italiano | 🇮🇹 | LTR | System |
| 15 | Dutch | `nl` | Nederlands | 🇳🇱 | LTR | System |

**RTL Languages**: Urdu (🇵🇰), Arabic (🇸🇦) - Automatic right-to-left layout

## 📁 Translation Structure

```
AI-NATIVE-BOOK/
├── i18n/
│   ├── ur/                          # Urdu translations
│   │   ├── code.json               # UI strings
│   │   ├── docusaurus-theme-classic/
│   │   │   └── navbar.json         # Navbar translations
│   │   └── docusaurus-plugin-content-docs/
│   │       └── current/
│   │           ├── intro.md        # Translated intro
│   │           └── module-4-vla-planning-capstone/
│   │               └── index.md    # Translated module page
│   ├── es/                          # Spanish
│   ├── zh/                          # Chinese
│   ├── hi/                          # Hindi
│   └── ...                          # Other languages
└── docs/                            # English (source)
    └── translation-guide.md
```

## 🎯 Translation Progress

### Current Status

| Language | UI | Content | Overall |
|----------|-----|---------|---------|
| English (en) | 100% | 100% | ✅ 100% |
| **Urdu (ur)** | **100%** | **15%** | **🚧 25%** |
| Spanish (es) | 10% | 0% | 🚧 5% |
| Chinese (zh) | 10% | 0% | 🚧 5% |
| Hindi (hi) | 10% | 0% | 🚧 5% |
| Arabic (ar) | 10% | 0% | 🚧 5% |
| Others | 5% | 0% | 🚧 2% |

### Priority Roadmap

**Phase 1** (Current):
- ✅ English: 100% complete
- 🚧 Urdu: 25% complete (UI ✅, Intro ✅, Module 4 overview ✅)

**Phase 2** (Next 4 weeks):
- 🎯 Urdu: Target 75% (all major chapters)
- 🎯 Spanish: Target 50%
- 🎯 Hindi: Target 50%

**Phase 3** (2 months):
- 🎯 Complete Urdu: 100%
- 🎯 Arabic: 50%
- 🎯 Chinese: 50%

## 🛠️ How to Add New Language Translation

### Step 1: Configure Language

Already done in `docusaurus.config.ts`! 15 languages pre-configured.

### Step 2: Create Translation Directory

```bash
# For any language (example: Spanish)
mkdir -p i18n/es/docusaurus-plugin-content-docs/current
mkdir -p i18n/es/docusaurus-theme-classic
```

### Step 3: Generate Translation Files

```bash
# Write translation templates
npm run write-translations -- --locale es

# This creates JSON files with all UI strings to translate
```

### Step 4: Translate Content

#### Translate UI Elements (`i18n/es/code.json`):

```json
{
  "theme.docs.paginator.next": {
    "message": "Siguiente",
    "description": "Next button in Spanish"
  }
}
```

#### Translate Content (`i18n/es/docusaurus-plugin-content-docs/current/intro.md`):

```markdown
# Bienvenido al Libro AI-Native

Una guía completa sobre IA y robótica...
```

### Step 5: Test Translation

```bash
# Start dev server with your language
npm run start -- --locale es

# Build for production
npm run build -- --locale es
```

## 🎨 Special Features

### Urdu Button Component

**Files**:
- `src/components/UrduButton/index.tsx`
- `src/components/UrduButton/styles.module.css`
- `src/theme/Navbar/index.tsx` (integration)

**Styling**:
```css
.urduButton {
  background: linear-gradient(135deg, #01b468 0%, #0d9488 100%);
  /* Green gradient (Pakistan theme) */

  box-shadow: 0 4px 15px rgba(1, 180, 104, 0.3);
  /* Glowing effect */

  animation: pulse 2s ease-in-out infinite;
  /* Pulsing when active */
}
```

### RTL Layout Support

**Automatic for**: Urdu, Arabic

**CSS Rules**:
```css
html[dir="rtl"] .navbar__items {
  flex-direction: row-reverse;
}

html[dir="rtl"] article {
  text-align: right;
}
```

### Font Loading

**Google Fonts imported in `custom.css`**:
- Noto Nastaliq Urdu (Urdu - Nastaliq script)
- Noto Sans Arabic (Arabic)
- Noto Sans SC (Chinese Simplified)
- Noto Sans JP (Japanese)
- Noto Sans KR (Korean)
- Noto Sans Devanagari (Hindi)

## 📊 Build Commands for All Languages

### Development

```bash
# English
npm start

# Urdu (special button appears!)
npm run start -- --locale ur

# All locales (slow)
npm run start
```

### Production Build

```bash
# Build all languages
npm run build

# Build specific language
npm run build -- --locale ur

# Deploy specific language
npm run serve -- --locale ur
```

## 🌟 Translation Best Practices

### DO:
- ✅ Keep technical terms in English (PyTorch, ROS, API)
- ✅ Translate concepts clearly
- ✅ Use native fonts (Nastaliq for Urdu)
- ✅ Test RTL layout for Arabic/Urdu
- ✅ Keep code examples in original form
- ✅ Translate comments in code

### DON'T:
- ❌ Translate function/variable names in code
- ❌ Change technical accuracy for simplicity
- ❌ Mix RTL/LTR improperly
- ❌ Use machine translation without review
- ❌ Skip testing on mobile devices

## 🎓 Translation Examples

### Sample Urdu Translation

**English**:
```markdown
# Getting Started with VLA Models

Learn how Vision-Language-Action models work.
```

**Urdu**:
```markdown
# VLA ماڈلز کے ساتھ شروعات

جانیں کہ وژن-لینگویج-ایکشن ماڈلز کیسے کام کرتے ہیں۔
```

### Sample Code with Urdu Comments

```python
# اردو میں تبصرے: یہ ایک سادہ VLA ماڈل ہے
# English comments can be added for technical details

import torch
from transformers import CLIPModel

# ماڈل لوڈ کریں (Load the model)
model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")

# نتائج دیکھیں (View results)
print("Model loaded successfully!")
```

## 🔧 Troubleshooting

### Urdu Button Not Showing

1. Clear browser cache
2. Restart dev server
3. Check `src/theme/Navbar/index.tsx` exists

### RTL Layout Issues

1. Verify `direction: 'rtl'` in `docusaurus.config.ts`
2. Check CSS `html[dir="rtl"]` rules
3. Test with browser dev tools

### Font Not Loading

1. Check internet connection (Google Fonts CDN)
2. Verify `@import` in `custom.css`
3. Check browser console for font errors

## 📖 Documentation

See [`docs/translation-guide.md`](docs/translation-guide.md) for complete translation documentation.

## 🤝 Contributing

Want to help translate?

1. Choose your language from the 15 supported
2. Translate UI strings and content
3. Test thoroughly (especially RTL for Urdu/Arabic)
4. Submit pull request

**Priority**: Urdu translations most welcome! 🇵🇰

---

## ✨ Summary

Your AI-Native Book now supports:

- ✅ **15 languages** configured and ready
- ✅ **Urdu special button** with beautiful design
- ✅ **RTL support** for Urdu and Arabic
- ✅ **Professional fonts** (Noto family)
- ✅ **Sample translations** for Urdu
- ✅ **Easy switching** via button or dropdown
- ✅ **Mobile-responsive** language selection

**Ready to reach global audience!** 🌍
