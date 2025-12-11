// Authentication utilities for the textbook platform

/**
 * Checks if a user is authenticated
 * @param {Object} user - The user object
 * @returns {boolean} True if user is authenticated, false otherwise
 */
export const isAuthenticated = (user) => {
  return user && user.id && user.email;
};

/**
 * Checks if a route requires authentication
 * @param {string} route - The route to check
 * @returns {boolean} True if the route requires authentication
 */
export const requiresAuth = (route) => {
  return route.startsWith('/chapters/') || route.includes('/protected/');
};

/**
 * Gets the redirect URL after login
 * @param {string} currentPath - Current path the user was trying to access
 * @returns {string} The redirect URL
 */
export const getRedirectUrl = (currentPath = '') => {
  if (currentPath && requiresAuth(currentPath)) {
    return currentPath;
  }
  return '/chapters/c1-foundations';
};

/**
 * Handles token refresh
 * @returns {Promise<Object>} New tokens
 */
export const refreshToken = async () => {
  try {
    const response = await fetch('/api/auth/refresh', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include', // Include cookies for authentication
    });

    if (!response.ok) {
      throw new Error('Token refresh failed');
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Token refresh error:', error);
    throw error;
  }
};

/**
 * Validates an access token
 * @param {string} token - The access token to validate
 * @returns {Promise<boolean>} True if token is valid
 */
export const validateToken = async (token) => {
  try {
    const response = await fetch('/api/auth/session', {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${token}`,
      },
    });

    return response.ok;
  } catch (error) {
    console.error('Token validation error:', error);
    return false;
  }
};

/**
 * Securely stores authentication tokens
 * @param {Object} tokens - The tokens to store
 */
export const storeTokens = (tokens) => {
  // Store tokens in memory (not in localStorage for security reasons)
  if (typeof window !== 'undefined') {
    window.__AUTH_TOKENS__ = tokens;
  }
};

/**
 * Retrieves authentication tokens
 * @returns {Object|null} The stored tokens or null
 */
export const getStoredTokens = () => {
  if (typeof window !== 'undefined' && window.__AUTH_TOKENS__) {
    return window.__AUTH_TOKENS__;
  }
  return null;
};

/**
 * Clears stored authentication tokens
 */
export const clearTokens = () => {
  if (typeof window !== 'undefined') {
    window.__AUTH_TOKENS__ = null;
    delete window.__AUTH_TOKENS__;
  }
};

/**
 * Checks if user has required permissions for a route
 * @param {Object} user - The user object
 * @param {string} route - The route to check permissions for
 * @returns {boolean} True if user has permissions
 */
export const hasRoutePermissions = (user, route) => {
  // For now, all authenticated users can access all chapter routes
  // This can be extended to check roles/permissions in the future
  if (route.startsWith('/chapters/')) {
    return isAuthenticated(user);
  }

  // Add more route-specific permission checks here as needed
  return true;
};