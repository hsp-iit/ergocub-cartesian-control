#ifndef ERGOCUB_FINGER_CONTROLLER_MODULE_H
#define ERGOCUB_FINGER_CONTROLLER_MODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IControlMode.h>
#include <string>
#include <vector>

namespace ergocub {
namespace controller {

class FingerControllerModule : public yarp::os::RFModule
{
public:
    FingerControllerModule();
    virtual ~FingerControllerModule();

    // RFModule overrides
    bool configure(yarp::os::ResourceFinder& rf) override;
    bool updateModule() override;
    bool close() override;
    double getPeriod() override;

private:
    bool openDevices();
    bool closeDevices();
    bool readFingerCommands();
    bool applyFingerPositions();

    // YARP devices
    yarp::dev::PolyDriver m_leftArmDevice;
    yarp::dev::PolyDriver m_rightArmDevice;
    yarp::dev::IEncoders* m_leftEncoders;
    yarp::dev::IEncoders* m_rightEncoders;
    yarp::dev::IPositionControl* m_leftPositionControl;
    yarp::dev::IPositionControl* m_rightPositionControl;
    yarp::dev::IPositionDirect* m_leftPositionDirect;
    yarp::dev::IPositionDirect* m_rightPositionDirect;
    yarp::dev::IControlMode* m_leftControlMode;
    yarp::dev::IControlMode* m_rightControlMode;

    // YARP ports
    yarp::os::BufferedPort<yarp::os::Bottle> m_fingerCommandPort;

    // Configuration
    std::string m_robotName;
    std::string m_portPrefix;
    double m_period;
    
    // Joint control
    std::vector<int> m_fingerJoints;
    std::vector<double> m_targetPositions;
    std::vector<double> m_currentPositions;
    int m_numJoints;

    // Control flags
    bool m_isActive;
    bool m_useLeftHand;
    bool m_useRightHand;

    // Speed configuration
    double m_fingerSpeed;
    std::vector<double> m_lastSentPositions;
};

} // namespace controller
} // namespace ergocub

#endif // ERGOCUB_FINGER_CONTROLLER_MODULE_H
