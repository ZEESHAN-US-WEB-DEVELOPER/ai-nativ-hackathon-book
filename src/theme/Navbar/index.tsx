import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import UrduButton from '@site/src/components/UrduButton';
import styles from './styles.module.css';

export default function Navbar(props) {
  return (
    <div className={styles.navbarWrapper}>
      <OriginalNavbar {...props} />
      <div className={styles.urduButtonContainer}>
        <UrduButton />
      </div>
    </div>
  );
}
