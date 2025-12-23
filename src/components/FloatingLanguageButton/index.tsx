import React, {useState} from 'react';
import {useHistory, useLocation} from '@docusaurus/router';
import styles from './styles.module.css';

interface Language {
  code: string;
  name: string;
  native: string;
  flag: string;
  progress: number;
  special?: boolean;
  direction?: 'rtl' | 'ltr';
}

const languages: Language[] = [
  { code: 'en', name: 'English', native: 'English', flag: '🇺🇸', progress: 100, direction: 'ltr' },
  { code: 'ur', name: 'Urdu', native: 'اردو', flag: '🇵🇰', progress: 25, special: true, direction: 'rtl' },
  { code: 'es', name: 'Spanish', native: 'Español', flag: '🇪🇸', progress: 5, direction: 'ltr' },
  { code: 'zh', name: 'Chinese', native: '简体中文', flag: '🇨🇳', progress: 5, direction: 'ltr' },
  { code: 'hi', name: 'Hindi', native: 'हिन्दी', flag: '🇮🇳', progress: 5, direction: 'ltr' },
  { code: 'ar', name: 'Arabic', native: 'العربية', flag: '🇸🇦', progress: 5, direction: 'rtl' },
  { code: 'fr', name: 'French', native: 'Français', flag: '🇫🇷', progress: 2, direction: 'ltr' },
  { code: 'de', name: 'German', native: 'Deutsch', flag: '🇩🇪', progress: 2, direction: 'ltr' },
  { code: 'ja', name: 'Japanese', native: '日本語', flag: '🇯🇵', progress: 2, direction: 'ltr' },
  { code: 'ko', name: 'Korean', native: '한국어', flag: '🇰🇷', progress: 2, direction: 'ltr' },
  { code: 'pt', name: 'Portuguese', native: 'Português', flag: '🇧🇷', progress: 2, direction: 'ltr' },
  { code: 'ru', name: 'Russian', native: 'Русский', flag: '🇷🇺', progress: 2, direction: 'ltr' },
  { code: 'tr', name: 'Turkish', native: 'Türkçe', flag: '🇹🇷', progress: 2, direction: 'ltr' },
  { code: 'it', name: 'Italian', native: 'Italiano', flag: '🇮🇹', progress: 2, direction: 'ltr' },
  { code: 'nl', name: 'Dutch', native: 'Nederlands', flag: '🇳🇱', progress: 2, direction: 'ltr' },
];

export default function FloatingLanguageButton(): React.ReactElement {
  const [isOpen, setIsOpen] = useState(false);
  const history = useHistory();
  const location = useLocation();

  const getCurrentLanguage = (): Language => {
    const pathLang = location.pathname.split('/')[1];
    return languages.find(lang => lang.code === pathLang) || languages[0];
  };

  const currentLang = getCurrentLanguage();

  const switchLanguage = (langCode: string) => {
    const currentPath = location.pathname;
    let newPath: string;

    // Remove current language prefix if exists
    const pathWithoutLang = currentPath.replace(/^\/(en|ur|es|zh|hi|ar|fr|de|ja|ko|pt|ru|tr|it|nl)\//, '/');

    if (langCode === 'en') {
      // English is default, no prefix needed
      newPath = pathWithoutLang === '/' ? '/' : pathWithoutLang;
    } else {
      // Add language prefix
      newPath = `/${langCode}${pathWithoutLang === '/' ? '/' : pathWithoutLang}`;
    }

    history.push(newPath);
    setIsOpen(false);
  };

  return (
    <>
      <div className={styles.floatingButton}>
        <button
          className={styles.mainButton}
          onClick={() => setIsOpen(!isOpen)}
          aria-label="Change language"
          title="Change language"
        >
          <span className={styles.globeIcon}>🌍</span>
          <span className={styles.currentLang}>{currentLang.flag}</span>
          <span className={styles.chevron}>{isOpen ? '✕' : '▼'}</span>
        </button>
      </div>

      {isOpen && (
        <>
          <div className={styles.overlay} onClick={() => setIsOpen(false)} />
          <div className={styles.popup}>
            <div className={styles.popupHeader}>
              <h3 className={styles.popupTitle}>
                <span className={styles.popupTitleIcon}>🌍</span>
                Choose Your Language
              </h3>
              <p className={styles.popupSubtitle}>
                Select a language to read the AI-Native Book
              </p>
            </div>

            <div className={styles.languageGrid}>
              {/* Special Urdu Card - Always First */}
              {languages.filter(lang => lang.special).map((lang) => (
                <button
                  key={lang.code}
                  className={`${styles.languageCard} ${styles.specialCard} ${currentLang.code === lang.code ? styles.active : ''}`}
                  onClick={() => switchLanguage(lang.code)}
                >
                  <div className={styles.specialBadge}>
                    <span>⭐ Featured</span>
                  </div>
                  <div className={styles.cardFlag}>{lang.flag}</div>
                  <div className={styles.cardContent}>
                    <div className={styles.cardNative}>{lang.native}</div>
                    <div className={styles.cardName}>{lang.name}</div>
                    <div className={styles.progressBar}>
                      <div
                        className={styles.progressFill}
                        style={{width: `${lang.progress}%`}}
                      />
                    </div>
                    <div className={styles.progressText}>{lang.progress}% Complete</div>
                  </div>
                  {currentLang.code === lang.code && (
                    <div className={styles.activeCheck}>✓</div>
                  )}
                </button>
              ))}

              {/* Other Languages */}
              {languages.filter(lang => !lang.special).map((lang) => (
                <button
                  key={lang.code}
                  className={`${styles.languageCard} ${currentLang.code === lang.code ? styles.active : ''}`}
                  onClick={() => switchLanguage(lang.code)}
                >
                  <div className={styles.cardFlag}>{lang.flag}</div>
                  <div className={styles.cardContent}>
                    <div className={styles.cardNative}>{lang.native}</div>
                    <div className={styles.cardName}>{lang.name}</div>
                    <div className={styles.progressBar}>
                      <div
                        className={styles.progressFill}
                        style={{width: `${lang.progress}%`}}
                      />
                    </div>
                    <div className={styles.progressText}>{lang.progress}% Complete</div>
                  </div>
                  {currentLang.code === lang.code && (
                    <div className={styles.activeCheck}>✓</div>
                  )}
                </button>
              ))}
            </div>

            <div className={styles.popupFooter}>
              <p className={styles.footerNote}>
                🌟 More languages coming soon! Translation progress varies by language.
              </p>
            </div>
          </div>
        </>
      )}
    </>
  );
}
