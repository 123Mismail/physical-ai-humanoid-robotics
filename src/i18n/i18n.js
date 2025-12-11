// i18n configuration for the textbook platform
import i18n from 'i18next';
import { initReactI18next } from 'react-i18next';
import LanguageDetector from 'i18next-browser-languagedetector';
import enTranslation from './locales/en/translation.json';
import urTranslation from './locales/ur/translation.json';

// Configuration for i18n
const i18nConfig = {
  // Resources (translations) for different languages
  resources: {
    en: {
      translation: enTranslation,
    },
    ur: {
      translation: urTranslation,
    },
  },

  // Default language fallback
  fallbackLng: 'en',

  // Detection options for user's language
  detection: {
    // Order of language detection
    order: ['queryString', 'cookie', 'localStorage', 'sessionStorage', 'navigator', 'htmlTag', 'path', 'subdomain'],

    // Cache user language on
    caches: ['localStorage', 'cookie'],
    excludeCacheFor: ['cimode'], // Don't cache the cimode

    // Cookie options
    cookieOptions: {
      path: '/',
      sameSite: 'strict',
    },
  },

  // Interpolation settings
  interpolation: {
    escapeValue: false, // React already escapes values
  },

  // React specific options
  react: {
    useSuspense: false, // Don't use suspense for loading translations
    transSupportBasicHtmlNodes: true, // Enable translation of basic HTML nodes
    transKeepBasicHtmlNodesFor: ['br', 'strong', 'i', 'p'], // Keep basic HTML nodes in translations
  },

  // Debug mode (set to true to see console logs about translation loading)
  debug: process.env.NODE_ENV === 'development',
};

// Initialize i18n with the configuration
i18n
  .use(LanguageDetector) // Use language detector to detect user's language
  .use(initReactI18next) // Passes i18n down to react-i18next
  .init(i18nConfig);

// Function to get the direction (ltr/rtl) for the current language
export const getDirection = () => {
  return i18n.language === 'ur' ? 'rtl' : 'ltr';
};

// Function to check if current language is RTL
export const isRTL = () => {
  return i18n.language === 'ur';
};

// Function to change the language
export const changeLanguage = (lng) => {
  // Set the language in i18n
  i18n.changeLanguage(lng);

  // Update the document direction based on the language
  document.documentElement.lang = lng;
  document.documentElement.dir = getDirection();

  // Store the selected language in localStorage
  localStorage.setItem('selectedLanguage', lng);
};

// Function to get the current language
export const getCurrentLanguage = () => {
  return i18n.language;
};

// Function to get available languages
export const getAvailableLanguages = () => {
  return ['en', 'ur'];
};

// Function to get language information
export const getLanguageInfo = (lng = getCurrentLanguage()) => {
  const languages = {
    en: { code: 'en', name: 'English', nativeName: 'English', direction: 'ltr' },
    ur: { code: 'ur', name: 'Urdu', nativeName: 'اردو', direction: 'rtl' }
  };

  return languages[lng] || languages.en;
};

// Export the configured i18n instance
export default i18n;