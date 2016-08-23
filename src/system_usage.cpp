/// HEADER
#include "system_usage.h"

/// PROJECT
#include <csapex/model/connector_type.h>
#include <csapex/model/node_modifier.h>
#include <csapex/signal/event.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <iomanip>

using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(csapex::SystemUsage, csapex::Node)

#include "read_system_information.hpp"


SystemUsage::SystemUsage() :
    last_update_(clock::now())
{
}

void SystemUsage::setup(NodeModifier &node_modifier)
{
}

void SystemUsage::setupParameters(Parameterizable &parameters)
{
    if(!system_info_) {
        system_info_.reset(new system_information::system_info());
    }

    std::vector<std::string> cpuids;
    system_info_->getCPUIds(cpuids);
    for(const std::string &id : cpuids) {
        param::Parameter::Ptr p = param::ParameterFactory::declareOutputProgress(id,
                                                                                 param::ParameterDescription("load of " + id));
        param::OutputProgressParameter::Ptr o = std::dynamic_pointer_cast<param::OutputProgressParameter>(p);
        parameters.addParameter(o);
        cpu_loads_.insert(std::make_pair(id, o));
    }

    param::Parameter::Ptr p = param::ParameterFactory::declareOutputProgress("RAM usage",
                                                                             param::ParameterDescription("memory usage"));
    ram_usage_ = std::dynamic_pointer_cast<param::OutputProgressParameter>(p);
    parameters.addParameter(ram_usage_);

    system_information::ram_info ram_info;
    system_info_->getRAMInfo(ram_info);
    std::stringstream mem_info;
    mem_info << "RAM size: ";
    mem_info << std::setprecision(6) << system_information::ram_info::toMB(ram_info.total);
    mem_info << " [MB]";
    mem_info << std::endl;
    parameters.addParameter(param::ParameterFactory::declareOutputText("RAM size"));
    parameters.setParameter<std::string>("RAM size", mem_info.str());

    parameters.addParameter(param::ParameterFactory::declareRange("update interval",
                                                                   0.01, 10.0, 0.5, 0.01),
                            update_interval_);


}

void SystemUsage::process()
{
}

void SystemUsage::tick()
{
    if(clock::now() > (last_update_ + ms(update_interval_ * 1000.0))) {
        std::map<std::string, double> loads;
        system_info_->getCPUUsage(loads);
        system_information::ram_info ram_info;
        system_info_->getRAMInfo(ram_info);


        for(auto &load : loads) {
            cpu_loads_.at(load.first)->setProgress(load.second * 100.0, 100.0);
        }

        ram_usage_->setProgress(system_information::ram_info::toMB(ram_info.used),
                                system_information::ram_info::toMB(ram_info.total));
        last_update_ = clock::now();
    }
}
