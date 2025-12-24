#pragma once

#include <string>

namespace SettingsIo
{
	struct Result
	{
		bool ok = false;
		std::wstring error;
	};

	// Export current persisted settings (stored via CWinApp profile APIs) into a sharable INI file.
	Result ExportToIni(const std::wstring& iniPath);

	// Import settings from an INI file into CWinApp profile storage.
	// Missing keys will fall back to existing profile values (or defaults where appropriate).
	Result ImportFromIni(const std::wstring& iniPath);
}


