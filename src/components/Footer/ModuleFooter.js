import React, { useEffect, useState } from 'react';
import { parseModulesFromSidebar, getLimitedModules } from '@site/src/utils/sidebar-parser';
import Link from '@docusaurus/Link';
import Translate, { translate } from '@docusaurus/Translate';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

/**
 * ModuleFooter Component
 * Displays module titles in the footer instead of chapter-level links
 * Reads module structure from sidebar metadata
 *
 * Constitution Compliance: Principle VIII - Textbook Delivery Platform Requirements
 * Spec: specs/005-delivery-platform/spec.md (M-05, AC-8.4-A)
 */
function ModuleFooter() {
  const [modules, setModules] = useState([]);
  const { i18n } = useDocusaurusContext();

  useEffect(() => {
    try {
      // Parse modules from sidebar configuration
      const parsedModules = getLimitedModules(3); // Limit to 3 modules as specified
      setModules(parsedModules);
    } catch (error) {
      console.error('Error parsing modules for footer:', error);
      // Set empty array to prevent rendering issues
      setModules([]);
    }
  }, []);

  if (!modules || modules.length === 0) {
    return null; // Don't render if no modules are available
  }

  return (
    <div className="footer__module-section">
      <h3 className="footer__title">
        <Translate id="footer.modulesTitle" description="Footer modules section title">
          Course Modules
        </Translate>
      </h3>
      <div className="footer__items">
        {modules.map((module, index) => (
          <div key={module.id || index} className="footer__item">
            <Link
              to={module.firstChapter || '#'}
              className="footer__link"
            >
              {module.title}
            </Link>
          </div>
        ))}
      </div>
    </div>
  );
}

export default ModuleFooter;