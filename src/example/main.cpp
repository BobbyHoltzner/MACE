#include <iostream>
#include <thread>

#include "mace_core/mace_core.h"


#include "example_module.h"
#include "data_interpolation.h"

using namespace std;

int main(int argc, char *argv[])
{
    MaceCore::MaceCore core;
    std::shared_ptr<MaceCore::MaceData> data = std::make_shared<DataInterpolation>();
    core.AddDataFusion(data);


    // Create Example Module
    ExampleMetaData metaData;
    std::shared_ptr<ExampleModule> exampleModule = std::make_shared<ExampleModule>(metaData);
    exampleModule->setDataObject(data);



    //configure example module
    std::shared_ptr<MaceCore::ModuleParameterValue> configuration = std::make_shared<MaceCore::ModuleParameterValue>();
    configuration->AddTerminalValue("Int1", 5);

    std::shared_ptr<MaceCore::ModuleParameterValue> nest1 = std::make_shared<MaceCore::ModuleParameterValue>();
    nest1->AddTerminalValue("Nest1_Int1", 1);
    nest1->AddTerminalValue("Nest1_Int2", 2);
    nest1->AddTerminalValue("Nest1_Double1", 0.25);
    configuration->AddNonTerminal("Nest1", nest1);

    exampleModule->ConfigureModule(configuration);


    //normally would add module to core
    //core.AddRTAModule(rtaModule);





}
