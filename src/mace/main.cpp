#include <iostream>
#include <thread>

#include "mace_core/mace_core.h"

#include "configuration_reader_xml.h"


#include "data_interpolation.h"

#include "module_collection.h"


int main(int argc, char *argv[])
{

    //generate the factory that can make module instances
    MaceCore::ModuleFactory* factory = ModuleCollection::GenerateFactory();

    //Initialize core and configure data object
    MaceCore::MaceCore core;
    std::shared_ptr<MaceCore::MaceData> data = std::make_shared<DataInterpolation>();
    core.AddDataFusion(data);

    std::string filename = "MaceSetup.xml";

    std::cout << "Reading MACE configuration file from: " << filename << std::endl;

    ConfigurationReader_XML parser(factory);
    ConfigurationParseResult parseResult = parser.Parse(filename);
    if(parseResult.success == false)
    {
        std::cerr << "Error parsing configuration file: " << std::endl;
        std::cerr << parseResult.error << std::endl;
        return 1;
    }

    if(parseResult.warnings.size() > 0)
    {
        std::cout << "Configuration Parse Warnings:" << std::endl;
        for(auto it = parseResult.warnings.cbegin() ; it != parseResult.warnings.cend() ; ++it)
            std::cout << *it << std::endl;
    }


    bool addedPathPlanning = false;
    bool addedRTA = false;
    int numVehicles = 1;
    std::map<std::shared_ptr<MaceCore::ModuleBase>, std::string > modules = parser.GetCreatedModules();
    std::vector<std::thread*> threads;

    for(auto it = modules.cbegin() ; it != modules.cend() ; ++it)
    {
        std::shared_ptr<MaceCore::ModuleBase> module = it->first;
        std::string moduleType = it->second;
        std::cout << "Creating a " << MaceCore::ModuleBase::ModuleTypeToString(module->ModuleClass()) << " module of type: " << moduleType << std::endl;

        //set data object of module
        module->setDataObject(data);

        //configure module
        module->ConfigureModule(parser.GetModuleConfiguration(module));

        //start thread
        std::thread *thread = new std::thread([module]()
        {
            module->start();
        });
        threads.push_back(thread);

        //add to core (and check if too many have been added)
        MaceCore::ModuleBase::Classes moduleClass = module->ModuleClass();
        switch (moduleClass) {
        case MaceCore::ModuleBase::PATH_PLANNING:
        {
            if(addedPathPlanning == true)
            {
                std::cerr << "Only one path planning module can be added" << std::endl;
                return 1;
            }
            core.AddPathPlanningModule(std::dynamic_pointer_cast<MaceCore::IModuleCommandPathPlanning>(module));
            addedPathPlanning = true;
            break;
        }
        case MaceCore::ModuleBase::RTA:
        {
            if(addedRTA == true)
            {
                std::cerr << "Only one RTA module can be added" << std::endl;
                return 1;
            }
            core.AddRTAModule(std::dynamic_pointer_cast<MaceCore::IModuleCommandRTA>(module));
            addedRTA = true;
            break;
            break;
        }
        case MaceCore::ModuleBase::VEHICLE_COMMS:
        {
            core.AddVehicle(std::to_string(numVehicles), std::dynamic_pointer_cast<MaceCore::IModuleCommandVehicle>(module));
            numVehicles++;
            break;
        }
        default:
        {
            std::cerr << "Unknown module parsed" << std::endl;
            return 1;
        }
        }

    }


    //wait for all threads to complete
    for(std::thread* thread: threads)
    {
        thread->join();
    }


    return 0;
}