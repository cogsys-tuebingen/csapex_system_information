#ifndef READ_SYSTEM_INFORMATION_HPP
#define READ_SYSTEM_INFORMATION_HPP
#include <string>
#include <sstream>
#include <map>
#include <fstream>
#include <boost/regex.hpp>

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
    std::size_t total;      // KB
    std::size_t available;  // KB
    std::size_t used;

    inline static double toMB(std::size_t mem)
    {
        return mem / 1024.0;
    }
};

class system_info {
public:
    typedef std::shared_ptr<system_info> Ptr;
    system_info() :
        stat_("/proc/stat"),
        meminfo_("/proc/meminfo"),
        reg_cpu_("(cpu)([0-9]*)"),
        reg_mem_total_("MemTotal.*"),
        reg_mem_available_("MemAvailable.*"),
        reg_mem_buffers_("Buffers.*"),
        reg_mem_cached_("Cached.*"),
        reg_mem_free_("MemFree.*")
    {
        if(!stat_.is_open())
            throw std::runtime_error("Cannot open '/proc/stat'!");
        if(!meminfo_.is_open())
            throw std::runtime_error("Cannot open '/proc/meminfo'!");

        updateCPUInfo();
    }

    virtual ~system_info()
    {
        stat_.close();
        meminfo_.close();
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
    std::ifstream            meminfo_;
    boost::regex             reg_cpu_;
    boost::regex             reg_mem_total_;
    boost::regex             reg_mem_available_;
    boost::regex             reg_mem_buffers_;
    boost::regex             reg_mem_cached_;
    boost::regex             reg_mem_free_;

    void updateCPUInfo()
    {
       cpu_prev_ = cpu_;
       readCPUInfo(cpu_);
    }

    void updateRAMInfo()
    {
        meminfo_.clear();
        meminfo_.seekg(0, std::ios::beg);
        std::string line;
        std::size_t mem_total(0);
        std::size_t mem_available(0);
        std::size_t mem_buffers(0);
        std::size_t mem_cached(0);
        std::size_t mem_free(0);
        while(std::getline(meminfo_, line)) {
            std::vector<std::string> tokens;
            split(line, ' ', tokens);
            if(tokens.empty()) {
                continue;
            }

            if(boost::regex_match(tokens[0], reg_mem_total_)) {
                mem_total = as<std::size_t>(tokens[1]);
            } else if(boost::regex_match(tokens[0], reg_mem_available_)) {
                mem_available = as<std::size_t>(tokens[1]);
            } else if(boost::regex_match(tokens[0], reg_mem_buffers_)) {
                mem_buffers = as<std::size_t>(tokens[1]);
            } else if(boost::regex_match(tokens[0], reg_mem_cached_)) {
                mem_cached = as<std::size_t>(tokens[1]);
            } else if(boost::regex_match(tokens[0], reg_mem_free_)) {
                mem_free = as<std::size_t>(tokens[1]);
            }
            if(mem_total != 0 &&
                    (mem_available != 0 ||
                        (mem_buffers != 0 && mem_cached != 0 && mem_free != 0)))
                break;
        }

        if(mem_available != 0) {
            ram_.available = mem_available;
            ram_.total = mem_total;
            ram_.used = mem_total - mem_available;
        } else {
            ram_.available = mem_buffers + mem_cached + mem_free;
            ram_.total = mem_total;
            ram_.used = mem_total - ram_.available;
        }

    }

    void readCPUInfo(std::map<std::string, cpu_info> &infos)
    {
        stat_.clear();
        stat_.seekg(0, std::ios::beg);
        std::string line;
        while(std::getline(stat_, line)) {
            std::vector<std::string> tokens;
            split(line, ' ', tokens);
            if(tokens.empty()) {
                continue;
            }

            boost::smatch matches;
            if(boost::regex_match(tokens.front(), matches, reg_cpu_)) {
                cpu_info info;
                info.user    = as<std::size_t>(tokens.at(1));
                info.nice    = as<std::size_t>(tokens.at(2));
                info.system  = as<std::size_t>(tokens.at(3));
                info.idle    = as<std::size_t>(tokens.at(4));
                info.iowait  = as<std::size_t>(tokens.at(5));
                info.irq     = as<std::size_t>(tokens.at(6));
                info.softirq = as<std::size_t>(tokens.at(7));
                std::string id;
                if(matches[2] == "") {
                    id = "overall";
                } else {
                    id = matches[2];
                    if(id.size() == 1)
                        id = "0" + id;
                }
                infos["cpu_" + id] = info;
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
