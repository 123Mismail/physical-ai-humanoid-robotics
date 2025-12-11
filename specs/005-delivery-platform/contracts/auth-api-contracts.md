# Authentication API Contracts

## Session Management

### GET /api/auth/session
**Description**: Retrieve current user session information
**Authentication**: Required (valid session token)
**Response**:
- 200: OK - Returns session object
  ```json
  {
    "user": {
      "id": "string",
      "name": "string",
      "email": "string"
    },
    "expires": "timestamp",
    "accessToken": "string"
  }
  ```
- 401: Unauthorized - No valid session

### POST /api/auth/login
**Description**: Initiate login flow
**Response**:
- 302: Found - Redirect to authentication provider
- 200: OK - Login form for credentials (if applicable)

### POST /api/auth/logout
**Description**: End current session
**Response**:
- 200: OK - Session terminated
  ```json
  {
    "status": "success",
    "message": "Session terminated"
  }
  ```

## Token Management

### POST /api/auth/refresh
**Description**: Refresh access token using refresh token
**Request**:
```json
{
  "refreshToken": "string"
}
```
**Response**:
- 200: OK - New tokens issued
  ```json
  {
    "accessToken": "string",
    "expiresIn": "number"
  }
  ```
- 401: Unauthorized - Invalid or expired refresh token

## Protected Content Access

### GET /api/chapters/{chapterId}
**Description**: Access protected chapter content
**Authentication**: Required
**Response**:
- 200: OK - Chapter content
- 401: Unauthorized - No valid session
- 403: Forbidden - Insufficient permissions
- 404: Not Found - Chapter does not exist

### GET /api/chapters/search
**Description**: Search protected content
**Authentication**: Required
**Query Parameters**:
- q: search query
- locale: locale filter (optional)
**Response**:
- 200: OK - Search results
  ```json
  {
    "results": [
      {
        "id": "string",
        "title": "string",
        "route": "string",
        "contentPreview": "string",
        "locale": "string"
      }
    ],
    "total": "number"
  }
  ```