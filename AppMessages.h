#pragma once

// App-wide custom window messages (keep in WM_APP range to avoid conflicts).
// Broadcast after importing settings so any open UI can refresh from profile.
constexpr UINT WM_APP_SETTINGS_IMPORTED = WM_APP + 100;


