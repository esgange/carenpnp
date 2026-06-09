#pragma once

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <sstream>
#include <string>

#include "dobot_common/workspace_paths.hpp"

namespace dobot_common
{
namespace robot_identity
{

inline std::string trimCopy(const std::string &value)
{
  const auto not_space = [](unsigned char c) { return !std::isspace(c); };
  const auto first = std::find_if(value.begin(), value.end(), not_space);
  const auto last = std::find_if(value.rbegin(), value.rend(), not_space).base();
  if (first >= last)
  {
    return {};
  }
  return std::string(first, last);
}

inline std::string unquoteConfigValue(std::string value)
{
  value = trimCopy(value);
  if (value.size() >= 2 && value.front() == value.back() &&
      (value.front() == '"' || value.front() == '\''))
  {
    value = value.substr(1, value.size() - 2);
  }
  return trimCopy(value);
}

inline std::string readStationConfigValue(
  std::initializer_list<const char *> keys,
  const std::filesystem::path &source_hint = {})
{
  const auto station_config_path = paths::workspacePath({"station_config"}, source_hint);
  std::ifstream stream(station_config_path);
  if (!stream.good())
  {
    return {};
  }

  std::string raw_line;
  while (std::getline(stream, raw_line))
  {
    std::string line = trimCopy(raw_line);
    if (line.empty() || line.front() == '#')
    {
      continue;
    }
    const std::string export_prefix = "export ";
    if (line.rfind(export_prefix, 0) == 0)
    {
      line = trimCopy(line.substr(export_prefix.size()));
    }
    const auto equals = line.find('=');
    if (equals == std::string::npos)
    {
      continue;
    }

    const std::string key = trimCopy(line.substr(0, equals));
    for (const char *requested_key : keys)
    {
      if (key == requested_key)
      {
        const std::string value = unquoteConfigValue(line.substr(equals + 1));
        if (!value.empty())
        {
          return value;
        }
      }
    }
  }
  return {};
}

inline std::string resolveRobotIpAddress(
  const std::string &requested = {},
  const std::filesystem::path &source_hint = {})
{
  const std::string requested_ip = trimCopy(requested);
  if (!requested_ip.empty())
  {
    return requested_ip;
  }
  if (const char *env_ip = std::getenv("ROBOT_IP_ADDRESS"); env_ip != nullptr && *env_ip != '\0')
  {
    return trimCopy(env_ip);
  }
  return readStationConfigValue({"ROBOT_IP_ADDRESS", "ip_address"}, source_hint);
}

inline std::string sanitizeFilenameToken(const std::string &value)
{
  std::string token;
  bool previous_was_underscore = false;
  for (unsigned char c : value)
  {
    if (std::isalnum(c) || c == '.' || c == '-' || c == '_')
    {
      token.push_back(static_cast<char>(c));
      previous_was_underscore = false;
    }
    else if (!previous_was_underscore)
    {
      token.push_back('_');
      previous_was_underscore = true;
    }
  }

  while (!token.empty() && token.front() == '_')
  {
    token.erase(token.begin());
  }
  while (!token.empty() && token.back() == '_')
  {
    token.pop_back();
  }
  return token;
}

inline std::string currentDateStamp()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &now_time);
#else
  localtime_r(&now_time, &tm);
#endif
  std::ostringstream stream;
  stream << std::put_time(&tm, "%d%m%Y");
  return stream.str();
}

inline bool endsWith(const std::string &value, const std::string &suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

inline bool looksLikeIpToken(const std::string &token)
{
  if (token.empty() || token.find('.') == std::string::npos)
  {
    return false;
  }
  return std::all_of(
    token.begin(),
    token.end(),
    [](unsigned char c) { return std::isdigit(c) || c == '.'; });
}

enum class RobotFileMatch
{
  kDifferentRobot,
  kLegacyOrUnknown,
  kExactRobot,
};

inline RobotFileMatch classifyFilenameForRobot(
  const std::filesystem::path &path,
  const std::string &robot_ip_address)
{
  const std::string robot_ip_token = sanitizeFilenameToken(robot_ip_address);
  if (robot_ip_token.empty())
  {
    return RobotFileMatch::kLegacyOrUnknown;
  }

  const std::string stem = path.stem().string();
  if (endsWith(stem, "_" + robot_ip_token))
  {
    return RobotFileMatch::kExactRobot;
  }

  const auto underscore = stem.find_last_of('_');
  const std::string last_token =
    underscore == std::string::npos ? stem : stem.substr(underscore + 1);
  if (looksLikeIpToken(last_token))
  {
    return RobotFileMatch::kDifferentRobot;
  }
  return RobotFileMatch::kLegacyOrUnknown;
}

inline bool filenameMatchesExactRobot(
  const std::filesystem::path &path,
  const std::string &robot_ip_address)
{
  return !trimCopy(robot_ip_address).empty() &&
         classifyFilenameForRobot(path, robot_ip_address) == RobotFileMatch::kExactRobot;
}

struct LatestRobotFileSelection
{
  std::filesystem::path exact_path;
  std::filesystem::file_time_type exact_time{};

  void consider(
    const std::filesystem::path &path,
    const std::filesystem::file_time_type &time,
    const std::string &robot_ip_address)
  {
    switch (classifyFilenameForRobot(path, robot_ip_address))
    {
      case RobotFileMatch::kExactRobot:
        if (exact_path.empty() || time > exact_time)
        {
          exact_path = path;
          exact_time = time;
        }
        break;
      case RobotFileMatch::kLegacyOrUnknown:
      case RobotFileMatch::kDifferentRobot:
        break;
    }
  }

  std::filesystem::path selected() const
  {
    return exact_path;
  }
};

}  // namespace robot_identity
}  // namespace dobot_common
