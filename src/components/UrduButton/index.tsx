import React, {useState} from 'react';
import {useHistory, useLocation} from '@docusaurus/router';
import styles from './styles.module.css';

export default function UrduButton(): React.ReactElement {
  const history = useHistory();
  const location = useLocation();
  const [isHovered, setIsHovered] = useState(false);

  const switchToUrdu = () => {
    const currentPath = location.pathname;
    let urduPath: string;

    // If already on /ur/ path, stay there
    if (currentPath.startsWith('/ur/')) {
      urduPath = currentPath;
    }
    // If on English path, switch to Urdu
    else if (currentPath === '/' || currentPath === '') {
      urduPath = '/ur/';
    }
    else {
      // Replace /docs/ with /ur/docs/
      urduPath = currentPath.replace(/^\//, '/ur/');
    }

    history.push(urduPath);
  };

  // Check if currently viewing Urdu
  const isUrdu = location.pathname.startsWith('/ur/');

  return (
    <div className={styles.buttonWrapper}>
      <button
        onClick={switchToUrdu}
        onMouseEnter={() => setIsHovered(true)}
        onMouseLeave={() => setIsHovered(false)}
        className={`${styles.urduButton} ${isUrdu ? styles.active : ''}`}
        aria-label="Switch to Urdu language - اردو میں پڑھیں"
        title="Read in Urdu - اردو میں پڑھیں"
      >
        <span className={styles.flagIcon}>🇵🇰</span>
        <div className={styles.textContent}>
          <span className={styles.urduText}>اردو</span>
          <span className={styles.subText}>Urdu</span>
        </div>
        {isUrdu && (
          <span className={styles.activeIndicator}>
            <span className={styles.checkmark}>✓</span>
          </span>
        )}
      </button>

      {/* Tooltip on hover */}
      {isHovered && !isUrdu && (
        <div className={styles.tooltip}>
          <p className={styles.tooltipUrdu}>اردو میں پڑھیں</p>
          <p className={styles.tooltipEnglish}>Read in Urdu</p>
        </div>
      )}

      {/* Active badge */}
      {isUrdu && (
        <div className={styles.activeBadge}>
          <span>Reading in Urdu</span>
        </div>
      )}
    </div>
  );
}
