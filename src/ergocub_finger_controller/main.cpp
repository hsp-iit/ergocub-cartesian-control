#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include "module.h"

using namespace yarp::os;
using namespace ergocub::controller;

int main(int argc, char* argv[])
{
    // Initialize YARP network
    Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "YARP network is not available. Please check your YARP configuration.";
        return EXIT_FAILURE;
    }

    // Create and configure the resource finder
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("ergocub_finger_controller.ini");
    rf.setDefaultContext("ergocub_finger_controller");
    rf.configure(argc, argv);

    // Create the module
    FingerControllerModule module;

    // Configure and run the module
    if (!module.configure(rf)) {
        yError() << "Failed to configure the finger controller module";
        return EXIT_FAILURE;
    }

    yInfo() << "Finger controller module configured successfully";
    yInfo() << "Starting finger controller module...";

    // Run the module
    int result = module.runModule(rf);

    yInfo() << "Finger controller module terminated";
    return result;
}
