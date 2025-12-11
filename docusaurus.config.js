// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A University-Level Textbook on Physical AI, ROS 2, and Humanoid Systems',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-docusaurus-test-site.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',

  // GitHub pages deployment config (optional)
  organizationName: 'physical-ai', // Usually your GitHub org/user name.
  projectName: 'humanoid-robotics-textbook', // Usually your repo name.

  onBrokenLinks: 'throw',
  // Moved to markdown.hooks
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang.
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      ur: {
        direction: 'rtl',
        label: 'Urdu',
      },
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Path to chapters directory (Docusaurus docs root)
          path: 'chapters',
          routeBasePath: 'chapters',
          // Enable math support
          remarkPlugins: [require('remark-math')],
          rehypePlugins: [require('rehype-katex')],
        },
        theme: {
        },
        gtag: undefined,
        googleTagManager: undefined,
        sitemap: undefined,
      }),
    ],
  ],
  // KaTeX stylesheet for LaTeX math rendering
  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css',
      type: 'text/css',
      integrity:
        'sha384-n8MVd4RsNIU0tAv4ct0nTaAbDJwPJzDEaqSD1odI+WdtXRGWt2kTvGFasHpSy3SV',
      crossorigin: 'anonymous',
    },
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Textbook',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
          href: '/chapters/c1-foundations-physical-ai', // Link to first chapter instead of root
          target: '_self',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Chapters',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Textbook',
            items: [
              {
                label: 'Chapter 1: Foundations of Physical AI',
                to: '/chapters/c1-foundations-physical-ai',
              },
              {
                label: 'Chapter 2: ROS 2 Architecture',
                to: '/chapters/c2-ros2-architecture',
              },
              {
                label: 'Chapter 3: ROS 2 Actions and Services',
                to: '/chapters/c3-ros2-actions',
              },
              {
                label: 'Chapter 4: URDF Robot Description',
                to: '/chapters/c4-urdf-robot-description',
              },
              {
                label: 'Chapter 5: Gazebo Simulation',
                to: '/chapters/c5-gazebo-simulation',
              },
              {
                label: 'Chapter 6: NVIDIA Isaac Sim',
                to: '/chapters/c6-isaac-sim',
              },
              {
                label: 'Chapter 7: Unity Simulation for Humanoid Robotics',
                to: '/chapters/c7-unity-simulation',
              },
              {
                label: 'Chapter 8: Advanced Simulation Techniques',
                to: '/chapters/c8-advanced-simulation',
              },
              {
                label: 'Chapter 9: Real-Time Control Systems and Embedded Hardware',
                to: '/chapters/c9-real-time-control',
              },
              {
                label: 'Chapter 10: Real-Time Control Algorithms for Humanoid Robotics',
                to: '/chapters/c10-real-time-algorithms',
              },
              {
                label: 'Chapter 11: Sensor Fusion for Humanoid Robotics',
                to: '/chapters/c11-sensor-fusion',
              },
              {
                label: 'Chapter 12: Whole-Body Control for Humanoid Robotics',
                to: '/chapters/c12-whole-body-control',
              },
              {
                label: 'Chapter 13: ZMP Walking and Balance Control for Humanoid Robotics',
                to: '/chapters/c13-zmp-walking',
              },
              {
                label: 'Chapter 14: Humanoid Integration and Deployment',
                to: '/chapters/c14-humanoid-integration',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Humble Docs',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'Gazebo',
                href: 'https://gazebosim.org/',
              },
              {
                label: 'NVIDIA Isaac Sim',
                href: 'https://docs.omniverse.nvidia.com/isaacsim/latest/',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
        additionalLanguages: ['python', 'bash', 'markup', 'yaml'],
      },
    }),
};

module.exports = config;
