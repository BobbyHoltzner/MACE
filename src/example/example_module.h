#ifndef EXAMPLE_MODULE_H
#define EXAMPLE_MODULE_H

#include <iostream>

#include "mace_core/abstract_module_event_listeners.h"


/*
 * This file creates an example module for demonstration of Module functionality.
 *
 */

class ExampleMetaData
{

};


//!
//! \brief Interface defining the events that module is to generate.
//!
//! For modules used by mace this object will be in MaceCore as one of the "i_module_events_<moduleName>.h"
//!
class ExampleModuleEvents
{
    virtual void Event1() = 0;

    virtual void Event2(const int&) = 0;
};

enum class ExampleCommands
{
    Command1
};


//!
//! \brief Command module that is to impliment the module's event loop and general behavior.
//!
//! The moudle shall contain a set of public methods that MaceCore::MaceCore will use to command behavior on the module.//!
//! These methods are specific to the particular module and defined the interfaces: "i_module_command_<moduleName>.h"
//! These commands may not be invoked on the modules thread, the module will be responsible for marshaling any computation onto its event loop.
//! For this example the command methods will be declaired inline
//!
//! Data from other modules that is needed may be stored in the MaceCore::MaceData object that exists globally.
//! The reason for a seperate "global" data container over simply passing data in command methods was done for two reasons:
//!     1) Some modules may require the same data, so a centrailized container is simplier over distrubtiting idential data.
//!     2) Some data may be updated at a high rate, so changing one and calling a notify modules of new data is prefered over sending new data
//! Each module has read access to the MaceData object, but not write access.
//!
class ExampleModule : public MaceCore::AbstractModule_EventListeners<ExampleMetaData, ExampleModuleEvents, ExampleCommands>
{

public:
    ExampleModule() :
        MaceCore::AbstractModule_EventListeners<ExampleMetaData, ExampleModuleEvents, ExampleCommands>(),
        CommandIssue(false)
    {

    }

    virtual MaceCore::ModuleBase::Classes ModuleClass() const
    {
        return VEHICLE_COMMS;
    }



    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const
    {
        MaceCore::ModuleParameterStructure structure;

        MaceCore::ModuleParameterStructure nest1;
        nest1.AddTerminalParameters("Nest1_Int1", MaceCore::ModuleParameterTerminalTypes::INT);
        nest1.AddTerminalParameters("Nest1_Int2", MaceCore::ModuleParameterTerminalTypes::INT);
        nest1.AddTerminalParameters("Nest1_Double1", MaceCore::ModuleParameterTerminalTypes::DOUBLE);

        structure.AddNonTerminal("Nest1", std::make_shared<MaceCore::ModuleParameterStructure>(nest1));

        structure.AddTerminalParameters("Int1", MaceCore::ModuleParameterTerminalTypes::INT);

        return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
    }


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
    {
        int int1 = params->GetTerminalValue<int>("Int1");
        std::cout << "Int1 " << int1 << std::endl;

        std::shared_ptr<MaceCore::ModuleParameterValue> nest1 = params->GetNonTerminalValue("Nest1");
        std::cout << "Nest1 Int1 " << nest1->GetTerminalValue<int>("Nest1_Int1") << std::endl;
        std::cout << "Nest1 Int2 " << nest1->GetTerminalValue<int>("Nest1_Int2") << std::endl;
        std::cout << "Nest1 Double1 " << nest1->GetTerminalValue<double>("Nest1_Double1") << std::endl;
    }


    //!
    //! \brief function that is to kick off the path planning event loop
    //!
    virtual void start()
    {
        while(true)
        {
            //check if a new command was introduced
            if(CommandIssue == true)
            {

                CommandIssue = false;
            }


            std::this_thread::sleep_for (std::chrono::milliseconds(10));
        }
    }



    void Command1()
    {
        CommandIssue = true;
    }

private:

    bool CommandIssue;
};

#endif // EXAMPLE_MODULE_H
