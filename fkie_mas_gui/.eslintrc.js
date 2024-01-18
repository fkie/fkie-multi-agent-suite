module.exports = {
  extends: 'erb',
  plugins: ['@typescript-eslint'],
  rules: {
    // A temporary hack related to IDE not resolving correct package.json
    'import/no-extraneous-dependencies': 'off',
    'import/no-unresolved': 'off',
    'import/no-import-module-exports': 'off',
    'import/extensions': 'off',
    // Since React 17 and typescript 4.1 you can safely disable the rule
    'react/react-in-jsx-scope': 'off',
    // carbon uses commonly props propagation
    'react/jsx-props-no-spreading': 'off',
    // Sometimes is required to change the names to keep consistency (ex. nested promises)
    'promise/param-names': 'off',
    // sometimes the type of an incoming list is not known in advance
    'react/forbid-prop-types': 'off',
    // some class methods do not require this
    'class-methods-use-this': 'off',
    'react/sort-comp': 'off',
    // annoying but necessary?
    'no-console': 'off',

    'react/jsx-filename-extension': 'off',
    'no-shadow': 'off',
    '@typescript-eslint/no-shadow': 'error',
    'no-unused-vars': 'off',
    '@typescript-eslint/no-unused-vars': 'error',
  },
  parserOptions: {
    ecmaVersion: 2020,
    sourceType: 'module',
    project: './tsconfig.json',
    tsconfigRootDir: __dirname,
    createDefaultProgram: true,
  },
  settings: {
    'import/resolver': {
      // See https://github.com/benmosher/eslint-plugin-import/issues/1396#issuecomment-575727774 for line below
      node: {},
      webpack: {
        config: require.resolve('./.erb/configs/webpack.config.eslint.ts'),
      },
      typescript: {},
    },
    'import/parsers': {
      '@typescript-eslint/parser': ['.ts', '.tsx'],
    },
  },
};
