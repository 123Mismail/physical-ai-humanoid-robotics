/**
 * RTL Accessibility Utilities
 * Provides accessibility support for RTL languages like Urdu
 */

/**
 * Updates ARIA attributes for RTL layout
 * @param {string} language - Current language code
 */
export const updateAriaForRTL = (language) => {
  const isRTL = language === 'ur';

  // Update document attributes
  document.documentElement.setAttribute('lang', language);
  document.documentElement.setAttribute('dir', isRTL ? 'rtl' : 'ltr');

  // Update ARIA attributes for better screen reader support
  if (isRTL) {
    // Add RTL-specific ARIA labels where needed
    document.documentElement.setAttribute('data-rtl', 'true');
  } else {
    document.documentElement.removeAttribute('data-rtl');
  }
};

/**
 * Gets appropriate ARIA label based on text direction
 * @param {string} label - Original label
 * @param {string} language - Current language
 * @returns {string} Appropriately formatted label for RTL/LTR
 */
export const getAriaLabelForDirection = (label, language) => {
  const isRTL = language === 'ur';
  return isRTL ? `(${label})` : label;
};

/**
 * Sets up RTL-aware keyboard navigation
 * @param {HTMLElement} element - Element to set up navigation for
 * @param {string} language - Current language
 */
export const setupRTLKeyboardNavigation = (element, language) => {
  const isRTL = language === 'ur';

  if (element) {
    // For RTL languages, swap left/right arrow key functionality
    element.addEventListener('keydown', (e) => {
      if (isRTL) {
        if (e.key === 'ArrowLeft') {
          // In RTL, left arrow should move right (next)
          e.target.dispatchEvent(new KeyboardEvent('keydown', { key: 'ArrowRight', bubbles: true }));
          e.preventDefault();
        } else if (e.key === 'ArrowRight') {
          // In RTL, right arrow should move left (previous)
          e.target.dispatchEvent(new KeyboardEvent('keydown', { key: 'ArrowLeft', bubbles: true }));
          e.preventDefault();
        }
      }
    });
  }
};

/**
 * Gets the correct ARIA role for RTL elements
 * @param {string} baseRole - Base ARIA role
 * @param {string} language - Current language
 * @returns {string} Appropriate ARIA role for the language direction
 */
export const getAriaRoleForLanguage = (baseRole, language) => {
  const isRTL = language === 'ur';

  // For some roles, we may need to adjust behavior for RTL
  switch (baseRole) {
    case 'menu':
    case 'menubar':
      return isRTL ? `${baseRole} rtl` : baseRole;
    default:
      return baseRole;
  }
};

/**
 * Initializes RTL accessibility features
 * @param {string} language - Current language
 */
export const initRTLAccessibility = (language) => {
  updateAriaForRTL(language);

  // Listen for language changes and update accessibility attributes
  document.addEventListener('languageChange', (e) => {
    updateAriaForRTL(e.detail.language);
  });
};

// Export default function to initialize RTL accessibility
export default initRTLAccessibility;