import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Textbook',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://Viky11992.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Viky11992', // Usually your GitHub org/user name.
  projectName: 'humanoid-robotics-docusource', // Usually your repo name.

  onBrokenLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  themes: ['@docusaurus/theme-live-codeblock'],

  stylesheets: [
    'https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&family=Manrope:wght@400;500;600;700&display=swap',
  ],
  scripts: [
    { src: '/js/rag-chatbot.js', defer: true },
    { src: '/js/mobile-utils.js', defer: true }
  ],
  headTags: [
    {
      tagName: 'meta',
      attributes: {
        name: 'api-key',
        content: 'OcGog7FUPfnhAYinrxeoeOjhVWn412ZONxcHzG2AVlU', // Update this with your actual API key for production
      },
    },
    {
      tagName: 'meta',
      attributes: {
        name: 'api-base-url',
        content: process.env.API_BASE_URL || 'https://your-railway-backend-production.up.railway.app', // Railway backend URL for production
      },
    },
  ],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Viky11992/humanoid-robotics-docusource/tree/main/my-website/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Tutorial',
        },
        {
          to: '/auth',
          label: 'Sign In',
          position: 'right',
        },
        {
          href: 'https://github.com/Viky11992/humanoid-robotics-docusource',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Tutorial',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Follow Me',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Viky11992',
            },
            {
              label: 'LinkedIn',
              href: 'https://www.linkedin.com/in/shoaibarshad92/',
            },
            {
              label: 'Twitter/X',
              href: 'https://x.com/ShoaibVickey',
            },
          ],
        },
        {
          title: 'Contact',
          items: [
            {
              label: 'Email',
              href: 'mailto:Shoaibarshad470@gmail.com',
            },
            {
              label: 'Facebook',
              href: 'https://www.facebook.com/profile.php?id=100012378756441',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} My Project, Inc. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
