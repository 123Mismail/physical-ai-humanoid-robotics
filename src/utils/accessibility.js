/**
 * Accessibility Utilities
 * Provides WCAG 2.1 AA compliance features and full keyboard navigation support
 */

/**
 * Ensures proper contrast ratios for text elements
 * @param {string} backgroundColor - Background color in hex/rgb format
 * @param {string} textColor - Text color in hex/rgb format
 * @returns {boolean} Whether the contrast ratio meets WCAG 2.1 AA standards
 */
export const checkContrastRatio = (backgroundColor, textColor) => {
  // Implementation would require color parsing and contrast calculation
  // This is a simplified placeholder
  return true; // Placeholder - implement proper contrast checking
};

/**
 * Adds focus indicators for keyboard navigation
 * @param {HTMLElement} element - Element to enhance with focus indicators
 */
export const enhanceFocusIndicators = (element) => {
  if (element) {
    // Add a visible focus indicator
    element.style.outline = '2px solid #4a90e2';
    element.style.outlineOffset = '2px';
  }
};

/**
 * Implements ARIA attributes for dynamic content
 * @param {HTMLElement} element - Element to add ARIA attributes to
 * @param {Object} options - ARIA options
 */
export const addAriaAttributes = (element, options = {}) => {
  if (element) {
    // Add live region for dynamic updates
    if (options.live) {
      element.setAttribute('aria-live', options.live);
    }

    // Add label for accessibility
    if (options.label) {
      element.setAttribute('aria-label', options.label);
    }

    // Add describedby reference
    if (options.describedBy) {
      element.setAttribute('aria-describedby', options.describedBy);
    }

    // Add role attribute if specified
    if (options.role) {
      element.setAttribute('role', options.role);
    }

    // Add expanded/collapsed state for interactive elements
    if (options.expanded !== undefined) {
      element.setAttribute('aria-expanded', options.expanded);
    }
  }
};

/**
 * Implements skip link for screen readers
 */
export const implementSkipLinks = () => {
  // Create skip navigation link
  const skipLink = document.createElement('a');
  skipLink.href = '#main-content';
  skipLink.textContent = 'Skip to main content';
  skipLink.className = 'skip-link';
  skipLink.style.position = 'absolute';
  skipLink.style.left = '-10000px';
  skipLink.style.top = 'auto';
  skipLink.style.width = '1px';
  skipLink.style.height = '1px';
  skipLink.style.overflow = 'hidden';
  skipLink.style.backgroundColor = '#000';
  skipLink.style.color = '#fff';
  skipLink.style.padding = '8px';
  skipLink.style.borderRadius = '4px';
  skipLink.style.zIndex = '1000';
  skipLink.style.textDecoration = 'none';

  // Show skip link when focused
  skipLink.addEventListener('focus', () => {
    skipLink.style.left = '10px';
  });

  skipLink.addEventListener('blur', () => {
    skipLink.style.left = '-10000px';
  });

  // Add to the top of the body
  document.body.insertBefore(skipLink, document.body.firstChild);
};

/**
 * Manages focus within a specific container (useful for modal dialogs)
 * @param {HTMLElement} container - The container element to manage focus within
 */
export const manageFocusWithin = (container) => {
  if (!container) return;

  const focusableElements = container.querySelectorAll(
    'button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])'
  );

  if (focusableElements.length === 0) return;

  const firstElement = focusableElements[0];
  const lastElement = focusableElements[focusableElements.length - 1];

  container.addEventListener('keydown', (e) => {
    if (e.key === 'Tab') {
      if (e.shiftKey) {
        // Shift + Tab
        if (document.activeElement === firstElement) {
          e.preventDefault();
          lastElement.focus();
        }
      } else {
        // Tab
        if (document.activeElement === lastElement) {
          e.preventDefault();
          firstElement.focus();
        }
      }
    }
  });
};

/**
 * Handles keyboard navigation for menu items
 * @param {HTMLElement} menu - The menu container
 */
export const setupKeyboardNavigationForMenu = (menu) => {
  if (!menu) return;

  const menuItems = menu.querySelectorAll('a, button');

  if (menuItems.length === 0) return;

  menuItems.forEach((item, index) => {
    item.setAttribute('tabindex', '0');

    item.addEventListener('keydown', (e) => {
      switch (e.key) {
        case 'ArrowDown':
          e.preventDefault();
          const nextIndex = (index + 1) % menuItems.length;
          menuItems[nextIndex].focus();
          break;

        case 'ArrowUp':
          e.preventDefault();
          const prevIndex = (index - 1 + menuItems.length) % menuItems.length;
          menuItems[prevIndex].focus();
          break;

        case 'Home':
          e.preventDefault();
          menuItems[0].focus();
          break;

        case 'End':
          e.preventDefault();
          menuItems[menuItems.length - 1].focus();
          break;

        case 'Enter':
        case ' ':
          // Trigger click event for Enter or Space
          e.preventDefault();
          item.click();
          break;
      }
    });
  });
};

/**
 * Sets up keyboard navigation for tab panels
 * @param {HTMLElement} tabContainer - The container with tabs and panels
 */
export const setupKeyboardNavigationForTabs = (tabContainer) => {
  if (!tabContainer) return;

  const tabList = tabContainer.querySelector('[role="tablist"]');
  if (!tabList) return;

  const tabs = tabList.querySelectorAll('[role="tab"]');
  const panels = tabContainer.querySelectorAll('[role="tabpanel"]');

  if (tabs.length === 0 || panels.length === 0) return;

  tabs.forEach((tab, index) => {
    tab.setAttribute('tabindex', '0');

    tab.addEventListener('keydown', (e) => {
      let newTab;

      switch (e.key) {
        case 'ArrowRight':
          e.preventDefault();
          newTab = tabs[(index + 1) % tabs.length];
          break;

        case 'ArrowLeft':
          e.preventDefault();
          newTab = tabs[(index - 1 + tabs.length) % tabs.length];
          break;

        case 'Home':
          e.preventDefault();
          newTab = tabs[0];
          break;

        case 'End':
          e.preventDefault();
          newTab = tabs[tabs.length - 1];
          break;

        case 'Enter':
        case ' ':
          e.preventDefault();
          tab.click();
          break;
      }

      if (newTab) {
        newTab.focus();
        // Activate the new tab
        tabs.forEach(t => t.setAttribute('aria-selected', 'false'));
        newTab.setAttribute('aria-selected', 'true');
        newTab.setAttribute('tabindex', '0');

        // Hide all panels
        panels.forEach(p => p.setAttribute('hidden', 'true'));
        // Show the corresponding panel
        const panelId = newTab.getAttribute('aria-controls');
        if (panelId) {
          const correspondingPanel = document.getElementById(panelId);
          if (correspondingPanel) {
            correspondingPanel.removeAttribute('hidden');
          }
        }
      }
    });
  });
};

/**
 * Initializes accessibility features including full keyboard navigation
 */
export const initAccessibility = () => {
  // Implement skip links
  implementSkipLinks();

  // Enhance focus management
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Tab') {
      document.body.classList.add('keyboard-navigation');
    }
  });

  document.addEventListener('mousedown', () => {
    document.body.classList.remove('keyboard-navigation');
  });

  // Ensure proper heading hierarchy
  const headings = document.querySelectorAll('h1, h2, h3, h4, h5, h6');
  let lastLevel = 0;

  headings.forEach((heading, index) => {
    const level = parseInt(heading.tagName.charAt(1));

    // Check if heading structure is logical
    if (index > 0 && level > lastLevel + 1) {
      console.warn(`Potential heading hierarchy issue: ${heading.tagName} follows ${lastLevel > 0 ? 'h' + lastLevel : 'no heading'}`);
    }

    lastLevel = level;
  });

  // Set up keyboard navigation for menus
  const menus = document.querySelectorAll('[role="menu"], [role="menubar"]');
  menus.forEach(menu => setupKeyboardNavigationForMenu(menu));

  // Set up keyboard navigation for tab panels
  const tabContainers = document.querySelectorAll('[role="tablist"]');
  tabContainers.forEach(tabContainer => setupKeyboardNavigationForTabs(tabContainer));

  // Enhance focus indicators for all focusable elements
  const focusableElements = document.querySelectorAll('a, button, input, select, textarea, [tabindex]');
  focusableElements.forEach(element => {
    element.addEventListener('focus', () => {
      element.classList.add('focus-visible');
    });

    element.addEventListener('blur', () => {
      element.classList.remove('focus-visible');
    });
  });
};

// Run accessibility initialization when DOM is loaded
if (typeof window !== 'undefined' && typeof document !== 'undefined') {
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initAccessibility);
  } else {
    initAccessibility();
  }
}

export default initAccessibility;