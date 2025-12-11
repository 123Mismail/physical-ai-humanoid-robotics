import React from 'react';
import { SessionProvider } from 'next-auth/react';
import { AuthProvider } from '../Auth/AuthProvider';

/**
 * Root component wrapper for the Docusaurus application
 * Provides authentication context to the entire application
 *
 * @param {Object} props - Component properties
 * @param {React.ReactNode} props.children - Child components to be wrapped
 */
export default function Root({ children }) {
  return (
    <SessionProvider>
      <AuthProvider>
        {children}
      </AuthProvider>
    </SessionProvider>
  );
}