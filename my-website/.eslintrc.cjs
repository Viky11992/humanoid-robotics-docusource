module.exports = {
  extends: ['eslint:recommended', 'plugin:@typescript-eslint/recommended', 'plugin:react/recommended', 'plugin:react-hooks/recommended', 'plugin:jsx-a11y/recommended'],
  parser: '@typescript-eslint/parser',
  parserOptions: {
    ecmaVersion: 'latest',
    sourceType: 'module',
    project: ['./tsconfig.json'],
    tsconfigRootDir: __dirname,
    ecmaFeatures: {
      jsx: true,
    },
  },
  plugins: ['@typescript-eslint', 'react', 'react-hooks', 'jsx-a11y'],
  settings: {
    react: {
      version: 'detect',
    },
  },
  rules: {
    // Add custom rules here
    'react/react-in-jsx-scope': 'off', // Not needed for React 17+ with new JSX transform
  },
  env: {
    browser: true,
    node: true,
    es2021: true,
  },
  ignorePatterns: ['build/', 'dist/', '.docusaurus/', 'node_modules/'],
};