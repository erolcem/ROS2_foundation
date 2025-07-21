#include "foundation_common/system_monitor.hpp"
#include <algorithm>
#include <fstream>
#include <sstream>

namespace foundation_common
{

SystemMonitor::SystemMonitor()
: last_cpu_idle_(0), last_cpu_total_(0)
{
}

SystemHealth SystemMonitor::getSystemHealth()
{
  SystemHealth health;
  health.cpu_usage = getCpuUsage();
  health.memory_usage = getMemoryUsage();
  health.disk_usage = getDiskUsage();
  health.timestamp = std::chrono::system_clock::now();
  return health;
}

double SystemMonitor::getCpuUsage()
{
  std::ifstream stat_file("/proc/stat");
  if (!stat_file.is_open()) {
    return -1.0;  // Error reading
  }

  std::string line;
  std::getline(stat_file, line);
  
  std::istringstream ss(line);
  std::string cpu_label;
  long user, nice, system, idle, iowait, irq, softirq, steal;
  
  ss >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
  
  long total = user + nice + system + idle + iowait + irq + softirq + steal;
  long idle_total = idle + iowait;
  
  long diff_idle = idle_total - last_cpu_idle_;
  long diff_total = total - last_cpu_total_;
  
  last_cpu_idle_ = idle_total;
  last_cpu_total_ = total;
  
  if (diff_total == 0) {
    return 0.0;
  }
  
  return 100.0 * (1.0 - static_cast<double>(diff_idle) / diff_total);
}

double SystemMonitor::getMemoryUsage()
{
  std::ifstream meminfo("/proc/meminfo");
  if (!meminfo.is_open()) {
    return -1.0;  // Error reading
  }

  long mem_total = 0, mem_available = 0;
  std::string line;
  
  while (std::getline(meminfo, line)) {
    if (line.find("MemTotal:") == 0) {
      std::istringstream ss(line);
      std::string label, unit;
      ss >> label >> mem_total >> unit;
    } else if (line.find("MemAvailable:") == 0) {
      std::istringstream ss(line);
      std::string label, unit;
      ss >> label >> mem_available >> unit;
    }
    
    if (mem_total > 0 && mem_available > 0) {
      break;
    }
  }
  
  if (mem_total == 0) {
    return -1.0;
  }
  
  return 100.0 * (1.0 - static_cast<double>(mem_available) / mem_total);
}

double SystemMonitor::getDiskUsage()
{
  // Simple implementation - check root filesystem
  std::ifstream mounts("/proc/mounts");
  if (!mounts.is_open()) {
    return -1.0;
  }

  // For simplicity, return a placeholder value
  // In a real implementation, you would use statvfs() system call
  return 25.0;  // Placeholder: 25% disk usage
}

std::string SystemMonitor::getHealthStatus(const SystemHealth& health)
{
  if (health.cpu_usage > 90.0 || health.memory_usage > 90.0 || health.disk_usage > 90.0) {
    return "CRITICAL";
  } else if (health.cpu_usage > 75.0 || health.memory_usage > 75.0 || health.disk_usage > 75.0) {
    return "WARNING";
  } else {
    return "OK";
  }
}

}  // namespace foundation_common
