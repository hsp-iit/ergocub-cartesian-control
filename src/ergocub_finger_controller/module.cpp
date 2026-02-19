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
    , m_leftPositionDirect(nullptr)
    , m_rightPositionDirect(nullptr)
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
    m_fingerSpeed = rf.check("finger_speed", Value(60)).asFloat64();

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
    yInfo() << "Finger speed:" << m_fingerSpeed << "deg/s";

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
    m_lastSentPositions.resize(m_numJoints, 0.0);

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
    // Chiudi eventuali device aperti per evitare conflitti
    closeDevices();

    // --- CONFIGURAZIONE MANO SINISTRA ---
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
            !m_leftArmDevice.view(m_leftPositionDirect) || // View per Direct
            !m_leftArmDevice.view(m_leftControlMode)) {
            yError() << "[FingerController] Failed to open left hand drivers.";
            return false;
        }

        int leftHandAxes = 0;
        m_leftPositionControl->getAxes(&leftHandAxes);
        for (int i = 0; i < leftHandAxes; ++i) {
            // Passiamo in POSITION_DIRECT per evitare i movimenti a scatti
            m_leftControlMode->setControlMode(i, VOCAB_CM_POSITION_DIRECT);
        }
        yInfo() << "[FingerController] Left hand configured in POSITION_DIRECT.";
    }

    // --- CONFIGURAZIONE MANO DESTRA ---
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
            !m_rightArmDevice.view(m_rightPositionDirect) || // View per Direct
            !m_rightArmDevice.view(m_rightControlMode)) {
            yError() << "[FingerController] Failed to open right hand drivers.";
            return false;
        }

        int rightHandAxes = 0;
        m_rightPositionControl->getAxes(&rightHandAxes);
        for (int i = 0; i < rightHandAxes; ++i) {
            m_rightControlMode->setControlMode(i, VOCAB_CM_POSITION_DIRECT);
        }
        yInfo() << "[FingerController] Right hand configured in POSITION_DIRECT.";
    }

    yInfo() << "[FingerController] Configuration completed successfully.";
    return true;
}

bool FingerControllerModule::closeDevices()
{
    if (m_useLeftHand) {
        m_leftEncoders = nullptr;
        m_leftPositionDirect = nullptr;
        m_leftControlMode = nullptr;
        m_leftPositionControl = nullptr;
        
        if (m_leftArmDevice.isValid()) {
            m_leftArmDevice.close();
        }
    }
    
    if (m_useRightHand) {
        m_rightEncoders = nullptr;
        m_rightPositionDirect = nullptr;
        m_rightControlMode = nullptr;
        m_rightPositionControl = nullptr;
        
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

    return true;
}

bool FingerControllerModule::applyFingerPositions()
{
    if (!m_isActive) return false;

    // Calcoliamo lo spostamento massimo ammesso in questo ciclo (delta_theta = omega * delta_t)
    // m_fingerSpeed deve essere in gradi/s se i target sono in gradi, o rad/s se i target sono in rad
    double maxStep = m_fingerSpeed * m_period;

    // Vettore temporaneo per i comandi filtrati
    std::vector<double> filteredRefs(m_numJoints);

    for (int i = 0; i < m_numJoints; ++i) {
        double target = m_targetPositions[i];
        double current = m_lastSentPositions[i];
        double error = target - current;

        // Se l'errore è più grande del passo massimo, ci muoviamo solo di maxStep
        if (std::abs(error) > maxStep) {
            filteredRefs[i] = current + std::copysign(maxStep, error);
        } else {
            filteredRefs[i] = target;
        }
        
        // Aggiorniamo la memoria per il prossimo ciclo
        m_lastSentPositions[i] = filteredRefs[i];
    }

    // --- INVIO COMANDI ---
    const int fingerIndices[] = {0, 1, 2, 3, 4, 5};

    if (m_useLeftHand && m_useRightHand) {
        if (m_leftPositionDirect)
            m_leftPositionDirect->setPositions(6, fingerIndices, filteredRefs.data());
        if (m_rightPositionDirect)
            m_rightPositionDirect->setPositions(6, fingerIndices, filteredRefs.data() + 6);
    } 
    else if (m_useLeftHand) {
        if (m_leftPositionDirect)
            m_leftPositionDirect->setPositions(6, fingerIndices, filteredRefs.data());
    } 
    else if (m_useRightHand) {
        if (m_rightPositionDirect)
            m_rightPositionDirect->setPositions(6, fingerIndices, filteredRefs.data());
    }

    return true;
}
