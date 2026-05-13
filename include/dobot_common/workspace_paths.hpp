#pragma once

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

namespace dobot_common
{
namespace paths
{

inline bool isWorkspaceRoot(const std::filesystem::path &path)
{
  std::error_code ec;
  return std::filesystem::exists(path / "src", ec) &&
         (std::filesystem::exists(path / "README.md", ec) ||
          std::filesystem::exists(path / "docker-compose.yml", ec) ||
          std::filesystem::exists(path / "src" / "dobot_msgs_v4", ec));
}

inline std::filesystem::path findWorkspaceRootFrom(std::filesystem::path path)
{
  std::error_code ec;
  if (path.empty())
  {
    return {};
  }
  if (std::filesystem::is_regular_file(path, ec))
  {
    path = path.parent_path();
  }
  path = path.lexically_normal();
  while (!path.empty())
  {
    if (isWorkspaceRoot(path))
    {
      return path;
    }
    const auto parent = path.parent_path();
    if (parent == path)
    {
      break;
    }
    path = parent;
  }
  return {};
}

inline std::vector<std::filesystem::path> splitPathList(const char *value)
{
  std::vector<std::filesystem::path> paths;
  if (value == nullptr || *value == '\0')
  {
    return paths;
  }

  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ':'))
  {
    if (!token.empty())
    {
      paths.emplace_back(token);
    }
  }
  return paths;
}

inline std::filesystem::path workspaceRoot(const std::filesystem::path &source_hint = {})
{
  for (const char *env_name : {"DOBOT_PICKN_PLACE_ROOT", "DOBOT_WORKSPACE_ROOT"})
  {
    if (const char *value = std::getenv(env_name); value != nullptr && *value != '\0')
    {
      const auto root = findWorkspaceRootFrom(value);
      if (!root.empty())
      {
        return root;
      }
      return std::filesystem::path(value).lexically_normal();
    }
  }

  std::vector<std::filesystem::path> candidates;
  std::error_code ec;
  candidates.emplace_back(std::filesystem::current_path(ec));
  if (!source_hint.empty())
  {
    candidates.emplace_back(source_hint);
  }

  for (const char *env_name : {"COLCON_PREFIX_PATH", "AMENT_PREFIX_PATH"})
  {
    for (const auto &prefix : splitPathList(std::getenv(env_name)))
    {
      candidates.emplace_back(prefix);
      auto install_pos = prefix.end();
      for (auto it = prefix.begin(); it != prefix.end(); ++it)
      {
        if (*it == "install")
        {
          install_pos = it;
          break;
        }
      }
      if (install_pos != prefix.end())
      {
        std::filesystem::path root_candidate;
        for (auto it = prefix.begin(); it != install_pos; ++it)
        {
          root_candidate /= *it;
        }
        candidates.emplace_back(root_candidate);
      }
    }
  }

  for (const auto &candidate : candidates)
  {
    const auto root = findWorkspaceRootFrom(candidate);
    if (!root.empty())
    {
      return root;
    }
  }

  return std::filesystem::current_path(ec);
}

inline std::filesystem::path workspacePath(
  std::initializer_list<const char *> parts,
  const std::filesystem::path &source_hint = {})
{
  auto path = workspaceRoot(source_hint);
  for (const char *part : parts)
  {
    path /= part;
  }
  return path.lexically_normal();
}

}  // namespace paths
}  // namespace dobot_common
