#include <cstdlib>
#include <iostream>

#include <module.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>


int main(int argc, char** argv)
{
    const std::string module_name = "gb-ergocub-gaze-controller";

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
    rf.setDefaultContext("mc-ergocub-head-controller");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    /* Initialize the module. */
    Module module;
    module.runModule(rf);

    return EXIT_SUCCESS;
}
