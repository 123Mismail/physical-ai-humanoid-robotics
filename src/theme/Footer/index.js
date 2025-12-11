import React from 'react';
import OriginalFooter from '@theme-original/Footer';
import ModuleFooter from '@site/src/components/Footer/ModuleFooter';

/**
 * Custom Footer Component
 * Extends the original Docusaurus footer with modular navigation
 *
 * Constitution Compliance: Principle VIII - Textbook Delivery Platform Requirements
 * Spec: specs/005-delivery-platform/spec.md (M-05, AC-8.4-A)
 */
export default function Footer(props) {
  return (
    <>
      <OriginalFooter {...props} />
      <ModuleFooter />
    </>
  );
}