#ifndef FOUNDATION_COMMON_SYSTEM_MONITOR_HPP_
#define FOUNDATION_COMMON_SYSTEM_MONITOR_HPP_

#include <chrono>
#include <string>

namespace foundation_common
{

struct SystemHealth
{
  double cpu_usage;     // CPU usage percentage (0-100)
  double memory_usage;  // Memory usage percentage (0-100)
  double disk_usage;    // Disk usage percentage (0-100)
  std::chrono::system_clock::time_point timestamp;
};

class SystemMonitor
{
public:
  SystemMonitor();
  
  /**
   * @brief Get current system health metrics
   * @return SystemHealth struct with current metrics
   */
  SystemHealth getSystemHealth();
  
  /**
   * @brief Get CPU usage percentage
   * @return CPU usage as percentage (0-100), -1 on error
   */
  double getCpuUsage();
  
  /**
   * @brief Get memory usage percentage
   * @return Memory usage as percentage (0-100), -1 on error
   */
  double getMemoryUsage();
  
  /**
   * @brief Get disk usage percentage
   * @return Disk usage as percentage (0-100), -1 on error
   */
  double getDiskUsage();
  
  /**
   * @brief Get health status string based on metrics
   * @param health System health metrics
   * @return Status string: "OK", "WARNING", or "CRITICAL"
   */
  std::string getHealthStatus(const SystemHealth& health);

private:
  long last_cpu_idle_;
  long last_cpu_total_;
};

}  // namespace foundation_common

#endif  // FOUNDATION_COMMON_SYSTEM_MONITOR_HPP_
