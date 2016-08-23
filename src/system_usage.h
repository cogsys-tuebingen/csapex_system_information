#ifndef SYSTEMUSAGE_H
#define SYSTEMUSAGE_H

/// PROJECT
#include <csapex/model/tickable_node.h>
#include <csapex/model/connector_type.h>
#include <csapex/param/output_progress_parameter.h>
#include <chrono>
#include "read_system_information.hpp"

namespace csapex {
class SystemUsage : public csapex::TickableNode
{
public:
    SystemUsage();

    virtual void setup(NodeModifier &node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;
    virtual void tick() override;

private:
    using clock         = std::chrono::system_clock;
    using time_point    = clock::time_point;
    using ms            = std::chrono::duration<double, std::milli>;

    std::map<std::string, param::OutputProgressParameter::Ptr> cpu_loads_;
    param::OutputProgressParameter::Ptr                        ram_usage_;

    double                               update_interval_;
    time_point                           last_update_;
    system_information::system_info::Ptr system_info_;


};
}

#endif // SYSTEMUSAGE_H
