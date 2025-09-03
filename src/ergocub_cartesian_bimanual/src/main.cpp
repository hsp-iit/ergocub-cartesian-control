#include <cstdlib>
#include <iostream>

#include <module.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

int main(int argc, char** argv)
{
    const std::string module_name = "mc-ergocub-cartesian-bimanual";

    /* Check YARP network. */
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << module_name + "::main(). Error: YARP network is not available.";
        return EXIT_FAILURE;
    }

    /* Load configuration file. */
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext(module_name);
    rf.setDefaultConfigFile("sim/config.ini");
    rf.configure(argc, argv);

    /* Initialize the module. */
    Module module;
    return module.runModule(rf);
}
