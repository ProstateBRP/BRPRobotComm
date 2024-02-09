#include "OpenIGTLink.hpp"

template <typename TRobot>
OpenIGTLink<TRobot>::OpenIGTLink(shared_ptr<TRobot> robot, int port)
{
    this->robot = robot;
    // Port used for communication
    this->port = port;
    // On new connections retransmit robot state
    retransmit = false;
    // Does the client have a keep alive functionality ?
    keepAlive = false;
}

template <>
int OpenIGTLink<ProstateRobot>::Session()
{
    // Set socket and robot status
    std::vector<shared_ptr<ProstateRobotPhaseBase>>::iterator iter;
    for (iter = prostate_workphase_list.begin(); iter != prostate_workphase_list.end(); iter++)
    {
        (*iter)->SetSocket(socket);
        (*iter)->connect = true;
        (*iter)->SetRobot(robot);
        (*iter)->SetRobotStatus(make_shared<ProstateRobotStatus>(robot));
    }
    // Set undefined phase as the current phase;
    std::vector<shared_ptr<ProstateRobotPhaseBase>>::iterator currentPhase = prostate_workphase_list.begin();
    // Update robot state
    robot->current_state = (*currentPhase)->Name();
    while ((*currentPhase)->connect)
    {
        // Statement will change the state upon request of a new state.
        if ((*currentPhase)->Process())
        {
            std::string requestedWorkphase = (*currentPhase)->GetNextWorkPhase();
            std::string queryID = (*currentPhase)->GetQueryID();
            // Find the requested workphase
            std::vector<shared_ptr<ProstateRobotPhaseBase>>::iterator iter;
            for (iter = prostate_workphase_list.begin(); iter != prostate_workphase_list.end(); iter++)
            {
                // Check if the requested state is in the list AND if the current state allows transition into it.
                if (strcmp((*iter)->Name(), requestedWorkphase.c_str()) == 0 && (*currentPhase)->IsTransitionAllowed(requestedWorkphase))
                {
                    // Perform state-specific cleanup
                    (*currentPhase)->OnExit();
                    std::string previous_phase((*currentPhase)->Name());
                    // Change the current phase
                    currentPhase = iter;
                    (*currentPhase)->SetPreviousWorkPhase(previous_phase);
                    (*currentPhase)->Enter(queryID.c_str()); // Initialization process
                    // Update robot current state
                    robot->current_state = (*currentPhase)->Name();
                    break;
                }
            }
        }
    }
    robot->Reset();
    return 1;
}

template <>
void *OpenIGTLink<ProstateRobot>::ThreadIGT(void *igt)
{
    // Setup robot-specific states
    OpenIGTLink<ProstateRobot> *igtModule = (OpenIGTLink<ProstateRobot> *)igt;
    igtModule->prostate_workphase_list.push_back(make_shared<ProstateRobotUndefinedPhase>());
    igtModule->prostate_workphase_list.push_back(make_shared<ProstateRobotStartUpPhase>());
    igtModule->prostate_workphase_list.push_back(make_shared<ProstateRobotPlanningPhase>());
    igtModule->prostate_workphase_list.push_back(make_shared<ProstateRobotCalibrationPhase>());
    igtModule->prostate_workphase_list.push_back(make_shared<ProstateRobotTargetingPhase>());
    igtModule->prostate_workphase_list.push_back(make_shared<ProstateRobotMoveToTargetPhase>());
    igtModule->prostate_workphase_list.push_back(make_shared<ProstateRobotManualPhase>());
    igtModule->prostate_workphase_list.push_back(make_shared<ProstateRobotStopPhase>());
    igtModule->prostate_workphase_list.push_back(make_shared<ProstateRobotEmergencyPhase>());

    igtl::ServerSocket::Pointer serverSocket;
    serverSocket = igtl::ServerSocket::New();
    int r = serverSocket->CreateServer(igtModule->port);

    if (r < 0)
    {
        std::cerr << "ERROR: Cannot create a server socket." << std::endl;
        exit(0);
    }

    // While we are listening on this port
    while (1)
    {
        // Waiting for Connection
        igtModule->socket = serverSocket->WaitForConnection(2000);
        std::cerr << "WaitForConnection" << std::endl;
        // Connection specific variables state -- Not connected, This will show up in the UI console
        igtModule->robot->_socketIGTConnection = "Listening";
        igtModule->clientSocketConnected = 0;

        if (igtModule->socket.IsNotNull()) // if client connected
        {
            // Connection Specific Variables State -- Connected
            igtModule->robot->_socketIGTConnection = "Connected";
            igtModule->clientSocketConnected = -1;

            std::cerr << "MESSAGE: Client connected. Starting a session..." << std::endl;
            igtModule->Session();
        }
        // Socket closed change connect to listening
        igtModule->robot->_socketIGTConnection = "Listening";
        igtModule->clientSocketConnected = 0;
    }
    // Close connection
    igtModule->socket->CloseSocket();
    return NULL;
}


template <typename TRobot>
void OpenIGTLink<TRobot>::DisconnectSocket()
{
    socket->CloseSocket();
}

template class OpenIGTLink<ProstateRobot>;
// template class OpenIGTLink<NeuroRobot>;