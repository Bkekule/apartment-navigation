#pragma once
#include <gz/common/Console.hh>
#include <string_view>

class Logger {
  public:
    // The passed name string must be a literal or outlive this Logger instance
    explicit Logger(std::string_view p_name) : m_name(p_name) {}

    [[nodiscard]] std::ostream &debug() const { return gzdbg << "[" << m_name << "] "; }
    [[nodiscard]] std::ostream &info() const { return gzmsg << "[" << m_name << "] "; }
    [[nodiscard]] std::ostream &warn() const { return gzwarn << "[" << m_name << "] "; }
    [[nodiscard]] std::ostream &error() const { return gzerr << "[" << m_name << "] "; }

  private:
    std::string_view m_name; // Non-owning reference to logger identifier
};
