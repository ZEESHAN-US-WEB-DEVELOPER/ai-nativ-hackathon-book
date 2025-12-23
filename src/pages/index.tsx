import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import AnimatedBook from '@site/src/components/AnimatedBook';
import FloatingLanguageButton from '@site/src/components/FloatingLanguageButton';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className={styles.heroBackground}>
        <div className={styles.heroCircle}></div>
        <div className={styles.heroCircle}></div>
        <div className={styles.heroCircle}></div>
      </div>
      <div className="container">
        <div className={styles.heroLayout}>
          {/* Left Side - Text Content */}
          <div className={styles.heroContent}>
            <span className={styles.heroBadge}>🤖 AI & Robotics Education</span>
            <Heading as="h1" className={styles.heroTitle}>
              Master the Future of
              <span className={styles.heroGradient}> AI Humanoids</span>
            </Heading>
            <p className={styles.heroSubtitle}>
              A comprehensive guide to Vision-Language-Action models, robotic systems,
              and cutting-edge AI technologies. Learn from industry experts and build
              production-ready humanoid robots.
            </p>
            <div className={styles.heroButtons}>
              <Link
                className={clsx('button button--primary button--lg', styles.heroButton)}
                to="/docs/intro">
                Start Learning 🚀
              </Link>
              <Link
                className={clsx('button button--outline button--lg', styles.heroButtonSecondary)}
                to="/docs/module-4-vla-planning-capstone/">
                Explore VLA Models →
              </Link>
            </div>
            <div className={styles.heroStats}>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>4</div>
                <div className={styles.statLabel}>Modules</div>
              </div>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>50+</div>
                <div className={styles.statLabel}>Lessons</div>
              </div>
              <div className={styles.statItem}>
                <div className={styles.statNumber}>100%</div>
                <div className={styles.statLabel}>Free</div>
              </div>
            </div>
          </div>

          {/* Right Side - Animated Book */}
          <div className={styles.heroVisual}>
            <AnimatedBook />
          </div>
        </div>
      </div>
    </header>
  );
}

function BookFeatures() {
  const features = [
    {
      title: '🧠 Vision-Language-Action Models',
      description: 'Deep dive into RT-1, RT-2, and PaLM-E architectures. Learn how VLA models enable robots to understand natural language, perceive their environment, and take intelligent actions.',
      link: '/docs/module-4-vla-planning-capstone/vla-models/',
      gradient: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    },
    {
      title: '🎯 Motion Planning & Control',
      description: 'Master path planning algorithms, trajectory optimization, and reactive control systems. Implement real-world navigation with A*, RRT, and modern planning frameworks.',
      link: '/docs/module-4-vla-planning-capstone/planning-control/',
      gradient: 'linear-gradient(135deg, #f093fb 0%, #f5576c 100%)',
    },
    {
      title: '🤝 Isaac Navigation Systems',
      description: 'Explore NVIDIA Isaac Sim for realistic robot simulation. Learn sensor integration, SLAM algorithms, and autonomous navigation in complex environments.',
      link: '/docs/isaac-navigation-systems/',
      gradient: 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)',
    },
    {
      title: '🏗️ Digital Twin Technology',
      description: 'Build high-fidelity digital twins using Unity and ROS. Create virtual replicas for testing, simulation, and validation before physical deployment.',
      link: '/docs/digital-twin-robots/',
      gradient: 'linear-gradient(135deg, #43e97b 0%, #38f9d7 100%)',
    },
    {
      title: '🎓 Hands-On Capstone Project',
      description: 'Apply your knowledge in a comprehensive capstone project. Design, implement, and present a complete robotic system from concept to deployment.',
      link: '/docs/module-4-vla-planning-capstone/capstone/',
      gradient: 'linear-gradient(135deg, #fa709a 0%, #fee140 100%)',
    },
    {
      title: '⚡ Production-Ready Code',
      description: 'Learn industry best practices with production-grade code examples, optimization techniques, and real-world deployment strategies.',
      link: '/docs/intro',
      gradient: 'linear-gradient(135deg, #30cfd0 0%, #330867 100%)',
    },
  ];

  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            What You'll Learn
          </Heading>
          <p className={styles.sectionSubtitle}>
            Comprehensive curriculum designed for robotics engineers, AI researchers, and tech enthusiasts
          </p>
        </div>
        <div className={styles.featuresGrid}>
          {features.map((feature, idx) => (
            <Link key={idx} to={feature.link} className={styles.featureCard}>
              <div className={styles.featureCardInner} style={{background: feature.gradient}}>
                <div className={styles.featureContent}>
                  <h3 className={styles.featureTitle}>{feature.title}</h3>
                  <p className={styles.featureDescription}>{feature.description}</p>
                  <div className={styles.featureArrow}>→</div>
                </div>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

function LearningPath() {
  return (
    <section className={styles.learningPathSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Your Learning Journey
          </Heading>
          <p className={styles.sectionSubtitle}>
            Structured path from fundamentals to advanced robotics
          </p>
        </div>
        <div className={styles.pathContainer}>
          <div className={styles.pathItem}>
            <div className={styles.pathNumber}>01</div>
            <div className={styles.pathContent}>
              <h3>Foundations</h3>
              <p>Start with core concepts in AI, computer vision, and robotics fundamentals</p>
            </div>
          </div>
          <div className={styles.pathConnector}></div>
          <div className={styles.pathItem}>
            <div className={styles.pathNumber}>02</div>
            <div className={styles.pathContent}>
              <h3>VLA Models</h3>
              <p>Deep dive into Vision-Language-Action architectures and multimodal learning</p>
            </div>
          </div>
          <div className={styles.pathConnector}></div>
          <div className={styles.pathItem}>
            <div className={styles.pathNumber}>03</div>
            <div className={styles.pathContent}>
              <h3>Planning & Control</h3>
              <p>Master motion planning, trajectory optimization, and reactive systems</p>
            </div>
          </div>
          <div className={styles.pathConnector}></div>
          <div className={styles.pathItem}>
            <div className={styles.pathNumber}>04</div>
            <div className={styles.pathContent}>
              <h3>Capstone Project</h3>
              <p>Build and deploy a complete robotic system with real-world applications</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2" className={styles.ctaTitle}>
            Ready to Build the Future?
          </Heading>
          <p className={styles.ctaSubtitle}>
            Join thousands of engineers and researchers learning to build intelligent robotic systems
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className={clsx('button button--primary button--lg', styles.ctaButton)}
              to="/docs/intro">
              Get Started Now
            </Link>
            <Link
              className={clsx('button button--outline button--lg', styles.ctaButtonOutline)}
              to="/blog">
              Read the Blog
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

function LanguageShowcase() {
  const languages = [
    { code: 'en', name: 'English', native: 'English', flag: '🇺🇸', progress: 100 },
    { code: 'ur', name: 'Urdu', native: 'اردو', flag: '🇵🇰', progress: 25, special: true },
    { code: 'es', name: 'Spanish', native: 'Español', flag: '🇪🇸', progress: 5 },
    { code: 'zh', name: 'Chinese', native: '简体中文', flag: '🇨🇳', progress: 5 },
    { code: 'hi', name: 'Hindi', native: 'हिन्दी', flag: '🇮🇳', progress: 5 },
    { code: 'ar', name: 'Arabic', native: 'العربية', flag: '🇸🇦', progress: 5 },
    { code: 'fr', name: 'French', native: 'Français', flag: '🇫🇷', progress: 2 },
    { code: 'de', name: 'German', native: 'Deutsch', flag: '🇩🇪', progress: 2 },
    { code: 'ja', name: 'Japanese', native: '日本語', flag: '🇯🇵', progress: 2 },
    { code: 'ko', name: 'Korean', native: '한국어', flag: '🇰🇷', progress: 2 },
    { code: 'pt', name: 'Portuguese', native: 'Português', flag: '🇧🇷', progress: 2 },
    { code: 'ru', name: 'Russian', native: 'Русский', flag: '🇷🇺', progress: 2 },
  ];

  return (
    <section className={styles.languageSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Available in 15 Languages
          </Heading>
          <p className={styles.sectionSubtitle}>
            Learn AI and robotics in your native language
          </p>
        </div>
        <div className={styles.languageGrid}>
          {languages.map((lang, idx) => (
            <Link
              key={idx}
              to={lang.code === 'en' ? '/' : `/${lang.code}/`}
              className={`${styles.languageCard} ${lang.special ? styles.specialCard : ''}`}
            >
              <div className={styles.languageFlag}>{lang.flag}</div>
              <div className={styles.languageInfo}>
                <div className={styles.languageNative}>{lang.native}</div>
                <div className={styles.languageName}>{lang.name}</div>
                <div className={styles.progressBar}>
                  <div
                    className={styles.progressFill}
                    style={{width: `${lang.progress}%`}}
                  ></div>
                </div>
                <div className={styles.progressText}>{lang.progress}% Complete</div>
              </div>
              {lang.special && (
                <div className={styles.specialBadge}>Featured ⭐</div>
              )}
            </Link>
          ))}
        </div>
        <div className={styles.languageNote}>
          <p>🌍 More languages coming soon! Want to help translate? <Link to="/docs/translation-guide">Learn more →</Link></p>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="Master AI Humanoids, Vision-Language-Action models, and robotic systems with comprehensive tutorials and hands-on projects">
      <HomepageHeader />
      <main>
        <BookFeatures />
        <LanguageShowcase />
        <LearningPath />
        <CTASection />
      </main>
      <FloatingLanguageButton />
    </Layout>
  );
}
