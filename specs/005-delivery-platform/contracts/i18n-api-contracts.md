# Internationalization API Contracts

## Locale Management

### GET /api/i18n/locales
**Description**: Get available locales
**Response**:
- 200: OK - Available locales
  ```json
  {
    "locales": [
      {
        "code": "string",
        "name": "string",
        "isRTL": "boolean"
      }
    ],
    "defaultLocale": "string"
  }
  ```

### GET /api/i18n/translations/{locale}
**Description**: Get translations for specific locale
**Response**:
- 200: OK - Translation resources
  ```json
  {
    "locale": "string",
    "resources": {
      "key": "translated value"
    },
    "direction": "string"
  }
  ```
- 404: Not Found - Locale not available

## Content Localization

### GET /api/content/{id}/localized
**Description**: Get localized version of content
**Query Parameters**:
- locale: target locale code
**Response**:
- 200: OK - Localized content
  ```json
  {
    "id": "string",
    "locale": "string",
    "title": "string",
    "content": "string",
    "direction": "string",
    "localizedAt": "timestamp"
  }
  ```
- 404: Not Found - Content or locale not available

## RTL Layout Configuration

### GET /api/i18n/rtl-config
**Description**: Get RTL layout configuration
**Response**:
- 200: OK - RTL configuration
  ```json
  {
    "enabled": "boolean",
    "locales": ["string"],
    "rtlElements": ["string"],
    "mirroredComponents": ["string"]
  }
  ```