#ifndef READ_SYSTEM_INFORMATION_HPP
#define READ_SYSTEM_INFORMATION_HPP
#include <string>
#include <sstream>
#include <map>
#include <fstream>
#include <boost/regex.hpp>

#include "sys/types.h"
#include "sys/sysinfo.h"
#include "sys/stat.h"

namespace csapex {
namespace system_information {

inline void split(const std::string &str,
                  const char delimiter,
                  std::vector<std::string> &tokens)
{
  std::string buffer;
  for(std::size_t i = 0 ; i < str.size() ; ++i) {
      char c = str[i];
      if(c == delimiter) {
         if(!buffer.empty()) {
             tokens.emplace_back(buffer);
             buffer = "";
         }
      } else {
          buffer.push_back(c);
      }
  }
  if(!buffer.empty())
      tokens.emplace_back(buffer);
}

template<typename T>
inline T as(const std::string &str)
{
    std::stringstream ss;
    ss << str;
    T value;
    ss >> value;
    return value;
}


struct cpu_info {
    std::size_t user;       // normal processes executing in user mode
    std::size_t nice;       // niced processes executing in user mode
    std::size_t system;     // processes executing in kernel mode
    std::size_t idle;       // twiddling thumbs
    std::size_t iowait;     // waiting for I/O to complete
    std::size_t irq;        // servicing interrupts
    std::size_t softirq;    // servicing softirqs
};

struct ram_info {
    double total_size;      // MB
    double free;            // MB
};

class system_info {
public:
    typedef std::shared_ptr<system_info> Ptr;
    system_info() :
        stat_("/proc/stat"),
        reg_cpu_("cpu[0-9]*")
    {
        if(!stat_.is_open())
            throw std::runtime_error("Cannot open '/proc/stat'!");

        updateCPUInfo();
    }

    virtual ~system_info()
    {
        stat_.close();
    }

    inline void getRAMInfo(ram_info &info)
    {
        updateRAMInfo();
        info = ram_;
    }

    inline void getCPUUsage(std::map<std::string, double> &info)
    {
        updateCPUInfo();
        if(!cpu_prev_.empty()) {
            for(auto &cpu_entry : cpu_) {
                cpu_info &cpu = cpu_entry.second;
                cpu_info &cpu_prev = cpu_prev_[cpu_entry.first];
                info[cpu_entry.first] = usage(cpu, cpu_prev);
            }
        }
    }

    inline void getCPUIds(std::vector<std::string> &ids)
    {
        for(auto &cpu_entry : cpu_) {
            ids.emplace_back(cpu_entry.first);
        }
    }

private:
    std::map<std::string, cpu_info> cpu_;
    std::map<std::string, cpu_info> cpu_prev_;
    ram_info                 ram_;
    std::ifstream            stat_;
    boost::regex             reg_cpu_;

    void updateCPUInfo()
    {
       cpu_prev_ = cpu_;
       readCPUInfo(cpu_);
    }

    void updateRAMInfo()
    {
        struct sysinfo si;
        sysinfo(&si);
        ram_.total_size = si.totalram / (1024.0 * 1024.0);
        ram_.free       = si.freeram  / (1024.0 * 1024.0);
    }

    void readCPUInfo(std::map<std::string, cpu_info> &infos)
    {
        stat_.clear();
        stat_.seekg(0, std::ios::beg);
        std::string line;
        while(std::getline(stat_, line)) {
            std::vector<std::string> tokens;
            split(line, ' ', tokens);
            if(boost::regex_match(tokens.front(), reg_cpu_)) {
                cpu_info info;
                info.user    = as<std::size_t>(tokens.at(1));
                info.nice    = as<std::size_t>(tokens.at(2));
                info.system  = as<std::size_t>(tokens.at(3));
                info.idle    = as<std::size_t>(tokens.at(4));
                info.iowait  = as<std::size_t>(tokens.at(5));
                info.irq     = as<std::size_t>(tokens.at(6));
                info.softirq = as<std::size_t>(tokens.at(7));
                infos[tokens.at(0)] = info;
            }
        }
    }


    inline double usage(const cpu_info &info,
                        const cpu_info &info_prev)
    {
        double active = (info.user - info_prev.user)
                      + (info.nice - info_prev.nice)
                      + (info.system - info_prev.system);
        double idle = info.idle - info_prev.idle;
        return active / (active + idle);
    }



};
}
}


#endif // READ_SYSTEM_INFORMATION_HPP
