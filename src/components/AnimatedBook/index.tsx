import React from 'react';
import styles from './styles.module.css';

export default function AnimatedBook(): React.ReactElement {
  return (
    <div className={styles.bookContainer}>
      <div className={styles.book}>
        {/* Book Cover */}
        <div className={styles.bookCover}>
          <div className={styles.bookSpine}></div>
          <div className={styles.coverFront}>
            <div className={styles.coverTitle}>
              <span className={styles.coverTitleMain}>AI-Native</span>
              <span className={styles.coverTitleSub}>Book</span>
            </div>
            <div className={styles.coverIcon}>🤖</div>
          </div>
        </div>

        {/* Left Page */}
        <div className={styles.bookPage} data-page="left">
          <div className={styles.pageContent}>
            <h3 className={styles.pageTitle}>Vision-Language-Action</h3>
            <div className={styles.pageText}>
              <p>Master RT-1 & RT-2 architectures</p>
              <p>Learn multimodal integration</p>
              <p>Build intelligent robots</p>
            </div>
            <div className={styles.pageIcon}>🧠</div>
          </div>
        </div>

        {/* Right Page */}
        <div className={styles.bookPage} data-page="right">
          <div className={styles.pageContent}>
            <h3 className={styles.pageTitle}>Planning & Control</h3>
            <div className={styles.pageText}>
              <p>Path planning algorithms</p>
              <p>Trajectory optimization</p>
              <p>Reactive planning systems</p>
            </div>
            <div className={styles.pageIcon}>🎯</div>
          </div>
        </div>

        {/* Floating Elements */}
        <div className={styles.floatingElement} data-position="1">
          <span>RT-1</span>
        </div>
        <div className={styles.floatingElement} data-position="2">
          <span>PaLM-E</span>
        </div>
        <div className={styles.floatingElement} data-position="3">
          <span>SLAM</span>
        </div>
      </div>

      {/* Book Shadow */}
      <div className={styles.bookShadow}></div>
    </div>
  );
}
