#include "module.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace yarp::os;
using namespace yarp::dev;
using namespace ergocub::controller;

FingerControllerModule::FingerControllerModule()
    : m_leftEncoders(nullptr)
    , m_rightEncoders(nullptr)
    , m_leftPositionControl(nullptr)
    , m_rightPositionControl(nullptr)
    , m_leftControlMode(nullptr)
    , m_rightControlMode(nullptr)
    , m_robotName("ergoCub")
    , m_portPrefix("/ergocub_finger_controller")
    , m_period(0.01)
    , m_numJoints(0)
    , m_isActive(false)
    , m_useLeftHand(true)
    , m_useRightHand(true)
{
}

FingerControllerModule::~FingerControllerModule()
{
    close();
}

bool FingerControllerModule::configure(ResourceFinder& rf)
{
    yInfo() << "Configuring finger controller module...";

    // Read configuration parameters
    m_robotName = rf.check("robot", Value("ergoCub")).asString();
    m_portPrefix = rf.check("name", Value("/ergocub_finger_controller")).asString();
    m_period = rf.check("period", Value(0.01)).asFloat64();

    // Read hand selection configuration
    m_useLeftHand = rf.check("use_left_hand", Value(true)).asBool();
    m_useRightHand = rf.check("use_right_hand", Value(true)).asBool();

    // Validate hand selection
    if (!m_useLeftHand && !m_useRightHand) {
        yError() << "At least one hand must be enabled (use_left_hand or use_right_hand)";
        return false;
    }

    yInfo() << "Robot name:" << m_robotName;
    yInfo() << "Port prefix:" << m_portPrefix;
    yInfo() << "Control period:" << m_period << "s";
    yInfo() << "Use left hand:" << (m_useLeftHand ? "YES" : "NO");
    yInfo() << "Use right hand:" << (m_useRightHand ? "YES" : "NO");

    // Open robot devices
    if (!openDevices()) {
        yError() << "Failed to open robot devices";
        return false;
    }

    // Read finger joints from configuration or use defaults
    if (rf.check("finger_joints") && rf.find("finger_joints").isList()) {
        Bottle* fingerJointsList = rf.find("finger_joints").asList();
        m_fingerJoints.clear();
        for (int i = 0; i < fingerJointsList->size(); ++i) {
            m_fingerJoints.push_back(fingerJointsList->get(i).asInt32());
        }
        yInfo() << "Read" << m_fingerJoints.size() << "finger joints from configuration";
    } else {
        // Use default finger joint indices based on enabled hands
        m_fingerJoints.clear();
        if (m_useLeftHand) {
            // Left hand fingers (6 joints)
            for (int i = 0; i < 6; ++i) {
                m_fingerJoints.push_back(i);
            }
        }
        if (m_useRightHand) {
            // Right hand fingers (6 joints) - offset by 6 if left hand is enabled
            int offset = m_useLeftHand ? 6 : 0;
            for (int i = 0; i < 6; ++i) {
                m_fingerJoints.push_back(offset + i);
            }
        }
        yInfo() << "Using default finger joint configuration";
    }
    
    m_numJoints = m_fingerJoints.size();
    m_targetPositions.resize(m_numJoints, 0.0);
    m_currentPositions.resize(m_numJoints, 0.0);

    yInfo() << "Controlling" << m_numJoints << "finger joints";

    // Open command port
    std::string commandPortName = m_portPrefix + "/finger_commands:i";
    if (!m_fingerCommandPort.open(commandPortName)) {
        yError() << "Failed to open command port" << commandPortName;
        return false;
    }
    yInfo() << "Opened command port:" << commandPortName;
    yInfo() << "Send finger commands as bottles with" << m_numJoints << "values";

    // Initialize target and current positions
    m_targetPositions.resize(m_numJoints, 0.0);
    m_currentPositions.resize(m_numJoints, 0.0);

    m_isActive = true;
    yInfo() << "Finger controller module configured successfully";
    
    return true;
}

bool FingerControllerModule::updateModule()
{
    if (!m_isActive) {
        return false;
    }

    // Read finger commands from port
    readFingerCommands();

    // Apply finger positions
    applyFingerPositions();

    return true;
}

bool FingerControllerModule::close()
{
    yInfo() << "Closing finger controller module...";

    m_isActive = false;

    // Close ports
    m_fingerCommandPort.close();

    // Close devices
    closeDevices();

    yInfo() << "Finger controller module closed";
    return true;
}

double FingerControllerModule::getPeriod()
{
    return m_period;
}

bool FingerControllerModule::openDevices()
{
    // Check if devices are already open
    bool leftAlreadyOpen = m_leftArmDevice.isValid();
    bool rightAlreadyOpen = m_rightArmDevice.isValid();
    
    if ((m_useLeftHand == leftAlreadyOpen || !m_useLeftHand) && 
        (m_useRightHand == rightAlreadyOpen || !m_useRightHand)) {
        yInfo() << "Robot devices already configured correctly, skipping...";
        return true;
    }

    // Close any partially open devices first
    closeDevices();

    // FINGERS - Configure only enabled hands ////////////////////////////////////////////////////////////////////////////////////////
    
    // Configure left hand device if enabled
    if (m_useLeftHand) {
        Property leftHandOptions;
        leftHandOptions.put("device", "remotecontrolboardremapper");
        leftHandOptions.put("localPortPrefix", m_portPrefix + "/left_hand/client");
        leftHandOptions.addGroup("axesNames");
        Bottle& l_axes_names_bot = leftHandOptions.findGroup("axesNames").addList();
        l_axes_names_bot.addString("l_thumb_add");
        l_axes_names_bot.addString("l_thumb_oc");
        l_axes_names_bot.addString("l_index_add");
        l_axes_names_bot.addString("l_index_oc");
        l_axes_names_bot.addString("l_middle_oc");
        l_axes_names_bot.addString("l_ring_pinky_oc");
        leftHandOptions.addGroup("remoteControlBoards");
        Bottle& l_rcb_bot = leftHandOptions.findGroup("remoteControlBoards").addList();
        l_rcb_bot.addString("/" + m_robotName + "/left_arm");
        
        if (!m_leftArmDevice.open(leftHandOptions) || 
            !m_leftArmDevice.view(m_leftPositionControl) || 
            !m_leftArmDevice.view(m_leftControlMode)) {
            yError() << "[FingerController] Failed to open left hand drivers.";
            return false;
        }
        
        int leftHandAxes = 0;
        m_leftPositionControl->getAxes(&leftHandAxes);
        for (int i = 0; i < leftHandAxes; ++i) {
            m_leftControlMode->setControlMode(i, VOCAB_CM_POSITION);
            m_leftPositionControl->setRefSpeed(i, 180.0); // Adjust as needed
        }
        yInfo() << "[FingerController] Left hand configured successfully.";
    } else {
        yInfo() << "[FingerController] Left hand disabled by configuration.";
    }

    // Configure right hand device if enabled
    if (m_useRightHand) {
        Property rightHandOptions;
        rightHandOptions.put("device", "remotecontrolboardremapper");
        rightHandOptions.put("localPortPrefix", m_portPrefix + "/right_hand/client");
        rightHandOptions.addGroup("axesNames");
        Bottle& r_axes_names_bot = rightHandOptions.findGroup("axesNames").addList();
        r_axes_names_bot.addString("r_thumb_add");
        r_axes_names_bot.addString("r_thumb_oc");
        r_axes_names_bot.addString("r_index_add");
        r_axes_names_bot.addString("r_index_oc");
        r_axes_names_bot.addString("r_middle_oc");
        r_axes_names_bot.addString("r_ring_pinky_oc");
        rightHandOptions.addGroup("remoteControlBoards");
        Bottle& r_rcb_bot = rightHandOptions.findGroup("remoteControlBoards").addList();
        r_rcb_bot.addString("/" + m_robotName + "/right_arm");
        
        if (!m_rightArmDevice.open(rightHandOptions) || 
            !m_rightArmDevice.view(m_rightPositionControl) || 
            !m_rightArmDevice.view(m_rightControlMode)) {
            yError() << "[FingerController] Failed to open right hand drivers.";
            return false;
        }
        
        int rightHandAxes = 0;
        m_rightPositionControl->getAxes(&rightHandAxes);
        for (int i = 0; i < rightHandAxes; ++i) {
            m_rightControlMode->setControlMode(i, VOCAB_CM_POSITION);
            m_rightPositionControl->setRefSpeed(i, 180.0); // Adjust as needed
        }
        yInfo() << "[FingerController] Right hand configured successfully.";
    } else {
        yInfo() << "[FingerController] Right hand disabled by configuration.";
    }

    yInfo() << "[FingerController] Configuration completed successfully.";
    return true;
}

bool FingerControllerModule::closeDevices()
{
    if (m_useLeftHand) {
        m_leftEncoders = nullptr;
        m_leftPositionControl = nullptr;
        m_leftControlMode = nullptr;
        
        if (m_leftArmDevice.isValid()) {
            m_leftArmDevice.close();
        }
    }
    
    if (m_useRightHand) {
        m_rightEncoders = nullptr;
        m_rightPositionControl = nullptr;
        m_rightControlMode = nullptr;
        
        if (m_rightArmDevice.isValid()) {
            m_rightArmDevice.close();
        }
    }

    return true;
}

bool FingerControllerModule::readFingerCommands()
{
    Bottle* command = m_fingerCommandPort.read(false);
    if (command == nullptr) {
        return true; // No new command available
    }

    if (command->size() != m_numJoints) {
        yWarning() << "Received command with" << command->size() 
                   << "values, expected" << m_numJoints;
        return false;
    }

    // Read target positions from command
    for (int i = 0; i < m_numJoints; ++i) {
        m_targetPositions[i] = command->get(i).asFloat64();
    }

    yDebug() << "Received new finger command with" << m_numJoints << "joint targets";
    return true;
}

bool FingerControllerModule::applyFingerPositions()
{
    // Apply positions based on enabled hands
    if (m_useLeftHand && m_useRightHand) {
        // Both hands enabled - expect 12 values
        if (m_targetPositions.size() < 12) {
            yWarning() << "Expected 12 target positions for both hands, got" << m_targetPositions.size();
            return false;
        }
        
        // Left hand joints and references
        const int leftJoints[] = {0, 1, 2, 3, 4, 5};
        const double leftRefs[] = {
            m_targetPositions[0], m_targetPositions[1], m_targetPositions[2],
            m_targetPositions[3], m_targetPositions[4], m_targetPositions[5]
        };
        
        // Right hand joints and references
        const int rightJoints[] = {0, 1, 2, 3, 4, 5};
        const double rightRefs[] = {
            m_targetPositions[6], m_targetPositions[7], m_targetPositions[8],
            m_targetPositions[9], m_targetPositions[10], m_targetPositions[11]
        };
        
        // Move both hands
        if (m_leftPositionControl)
            m_leftPositionControl->positionMove(6, leftJoints, leftRefs);
        if (m_rightPositionControl)
            m_rightPositionControl->positionMove(6, rightJoints, rightRefs);
            
    } else if (m_useLeftHand) {
        // Only left hand enabled - expect 6 values
        if (m_targetPositions.size() < 6) {
            yWarning() << "Expected 6 target positions for left hand, got" << m_targetPositions.size();
            return false;
        }
        
        const int leftJoints[] = {0, 1, 2, 3, 4, 5};
        const double leftRefs[] = {
            m_targetPositions[0], m_targetPositions[1], m_targetPositions[2],
            m_targetPositions[3], m_targetPositions[4], m_targetPositions[5]
        };
        
        if (m_leftPositionControl)
            m_leftPositionControl->positionMove(6, leftJoints, leftRefs);
            
    } else if (m_useRightHand) {
        // Only right hand enabled - expect 6 values
        if (m_targetPositions.size() < 6) {
            yWarning() << "Expected 6 target positions for right hand, got" << m_targetPositions.size();
            return false;
        }
        
        const int rightJoints[] = {0, 1, 2, 3, 4, 5};
        const double rightRefs[] = {
            m_targetPositions[0], m_targetPositions[1], m_targetPositions[2],
            m_targetPositions[3], m_targetPositions[4], m_targetPositions[5]
        };
        
        if (m_rightPositionControl)
            m_rightPositionControl->positionMove(6, rightJoints, rightRefs);
    }

    return true;
}
