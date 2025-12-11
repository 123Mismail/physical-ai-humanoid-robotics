/**
 * Sidebar Parser Utility
 * Parses the Docusaurus sidebar configuration to extract module information
 * Used for generating the modular footer navigation as required by the constitution
 */

// Import the sidebar configuration
import sidebarConfig from '@site/sidebars.js';

/**
 * Parses the sidebar configuration to extract module information
 * @param {Object} sidebar - The sidebar configuration object
 * @returns {Array} Array of module objects with id, title, items, and firstChapter
 */
export const parseModulesFromSidebar = (sidebar = sidebarConfig) => {
  try {
    const modules = [];

    // Check if sidebar is valid
    if (!sidebar || typeof sidebar !== 'object') {
      console.warn('Sidebar configuration is invalid, returning empty modules');
      return [];
    }

    // Iterate through each category in the sidebar
    for (const [categoryId, categoryConfig] of Object.entries(sidebar)) {
      // Check if this category represents a module
      if (categoryConfig && categoryConfig.type === 'category') {
        const module = {
          id: categoryId,
          title: categoryConfig.label || categoryId,
          items: [],
          firstChapter: null,
          order: modules.length + 1 // Simple ordering based on appearance
        };

        // Extract items from the category
        if (categoryConfig.items && Array.isArray(categoryConfig.items)) {
          module.items = extractChapterItems(categoryConfig.items);

          // Find the first chapter in this module
          if (module.items.length > 0) {
            // Look for the first valid chapter link
            const firstItem = module.items.find(item =>
              item.type === 'doc' && item.id && item.id.startsWith('c')
            );

            if (firstItem) {
              module.firstChapter = `/chapters/${firstItem.id}`;
            } else if (module.items.length > 0) {
              // Fallback: use the first item if it has a valid path
              const firstItem = module.items[0];
              if (firstItem.type === 'doc' && firstItem.id) {
                module.firstChapter = `/chapters/${firstItem.id}`;
              } else if (firstItem.type === 'link' && firstItem.href) {
                module.firstChapter = firstItem.href;
              }
            }
          }
        }

        // Only add modules that have a valid title and at least one item
        if (module.title && (module.items.length > 0 || module.firstChapter)) {
          modules.push(module);
        }
      } else if (Array.isArray(categoryConfig)) {
        // Handle the case where the sidebar is an array directly
        const module = {
          id: `module-${modules.length + 1}`,
          title: `Module ${modules.length + 1}`,
          items: [],
          firstChapter: null,
          order: modules.length + 1
        };

        module.items = extractChapterItems(categoryConfig);

        if (module.items.length > 0) {
          const firstItem = module.items.find(item =>
            item.type === 'doc' && item.id && item.id.startsWith('c')
          );

          if (firstItem) {
            module.firstChapter = `/chapters/${firstItem.id}`;
          }
        }

        if (module.title && (module.items.length > 0 || module.firstChapter)) {
          modules.push(module);
        }
      }
    }

    return modules;
  } catch (error) {
    console.error('Error parsing sidebar configuration:', error);
    // Return empty array in case of error to prevent breaking the application
    return [];
  }
};

/**
 * Recursively extracts chapter items from sidebar items
 * @param {Array} items - Array of sidebar items
 * @param {Array} extractedItems - Accumulator for extracted items (internal use)
 * @returns {Array} Array of extracted chapter items
 */
const extractChapterItems = (items, extractedItems = []) => {
  for (const item of items) {
    if (item.type === 'doc') {
      // Direct document reference
      extractedItems.push({
        type: 'doc',
        id: item.id,
        label: item.label || item.id,
        ...(item.href && { href: item.href })
      });
    } else if (item.type === 'link') {
      // External or internal link
      extractedItems.push({
        type: 'link',
        href: item.href,
        label: item.label || item.href
      });
    } else if (item.type === 'category' && item.items && Array.isArray(item.items)) {
      // Nested category - recursively extract items
      extractChapterItems(item.items, extractedItems);
    } else if (Array.isArray(item)) {
      // Nested array of items
      extractChapterItems(item, extractedItems);
    }
  }

  return extractedItems;
};

/**
 * Gets module information by module ID
 * @param {string} moduleId - The ID of the module to retrieve
 * @param {Object} sidebar - Optional sidebar config (defaults to imported config)
 * @returns {Object|null} Module object or null if not found
 */
export const getModuleById = (moduleId, sidebar = sidebarConfig) => {
  const modules = parseModulesFromSidebar(sidebar);
  return modules.find(module => module.id === moduleId) || null;
};

/**
 * Gets module information by title
 * @param {string} title - The title of the module to retrieve
 * @param {Object} sidebar - Optional sidebar config (defaults to imported config)
 * @returns {Object|null} Module object or null if not found
 */
export const getModuleByTitle = (title, sidebar = sidebarConfig) => {
  const modules = parseModulesFromSidebar(sidebar);
  return modules.find(module => module.title === title) || null;
};

/**
 * Gets all module titles
 * @param {Object} sidebar - Optional sidebar config (defaults to imported config)
 * @returns {Array} Array of module titles
 */
export const getModuleTitles = (sidebar = sidebarConfig) => {
  const modules = parseModulesFromSidebar(sidebar);
  return modules.map(module => module.title);
};

/**
 * Validates that modules are parsed correctly
 * @param {Object} sidebar - Optional sidebar config (defaults to imported config)
 * @returns {Object} Validation result with errors array
 */
export const validateModules = (sidebar = sidebarConfig) => {
  const modules = parseModulesFromSidebar(sidebar);
  const errors = [];

  modules.forEach((module, index) => {
    if (!module.id) {
      errors.push(`Module at index ${index} is missing an ID`);
    }

    if (!module.title) {
      errors.push(`Module with ID ${module.id || `at index ${index}`} is missing a title`);
    }

    if (!module.firstChapter) {
      errors.push(`Module "${module.title}" is missing a firstChapter reference`);
    }
  });

  return {
    isValid: errors.length === 0,
    errors,
    modulesCount: modules.length
  };
};

/**
 * Gets a limited number of modules (for footer display)
 * @param {number} limit - Maximum number of modules to return
 * @param {Object} sidebar - Optional sidebar config (defaults to imported config)
 * @returns {Array} Limited array of modules
 */
export const getLimitedModules = (limit = 3, sidebar = sidebarConfig) => {
  const modules = parseModulesFromSidebar(sidebar);
  return modules.slice(0, limit);
};

// Export the default parser function
export default parseModulesFromSidebar;