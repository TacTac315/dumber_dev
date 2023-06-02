/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 19
#define PRIORITY_TBATTERYLEVEL 19
#define PRIORITY_TRUNWATCHDOG 22
#define PRIORITY_TCOMPTEUR 20
#define PRIORITY_TOPENCAMERA 19
#define PRIORITY_TGRABCAMERA 17
#define PRIORITY_TCLOSECAMERA 19
#define PRIORITY_TFINDARENA 25
#define PRIORITY_TGETROBOTPOSITION 19

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 *
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 *
 * 3- Data flow is probably not optimal
 *
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 *
 * 5- Same behavior existe for ComMonitor::Write !
 *
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 *
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex,
 * semaphore, etc.)
 */
void Tasks::Init()
{
    int status;
    int err;

    camera = new Camera(sm, 5);
    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL))
    {
        cerr << "Error mutex create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL))
    {
        cerr << "Error mutex create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL))
    {
        cerr << "Error mutex create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL))
    {
        cerr << "Error mutex create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL))
    {
        cerr << "Error mutex create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL))
    {
        cerr << "Error mutex create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_bool_arena, NULL))
    {
        cerr << "Error mutex create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Mutexes created successfully" << endl
         << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO))
    {
        cerr << "Error semaphore create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO))
    {
        cerr << "Error semaphore create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO))
    {
        cerr << "Error semaphore create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO))
    {
        cerr << "Error semaphore create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_RunWtchg, NULL, 0, S_FIFO))
    {
        cerr << "Error semaphore create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_OpenCamera, NULL, 0, S_FIFO))
    {
        cerr << "Error semaphore create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_CloseCamera, NULL, 0, S_FIFO))
    {
        cerr << "Error semaphore create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_arena, NULL, 0, S_FIFO))
    {
        cerr << "Error semaphore create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_testArena, NULL, 0, S_FIFO))
    {
        cerr << "Error semaphore create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_position, NULL, 0, S_FIFO))
    {
        cerr << "Error semaphore create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Semaphores created successfully" << endl
         << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_BatteryLevel, "th_BatteryLevel", 0, PRIORITY_TBATTERYLEVEL, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_RunWatchdog, "th_RunWatchdog", 0, PRIORITY_TRUNWATCHDOG, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_create(&th_OpenCamera, "th_OpenCamera", 0, PRIORITY_TOPENCAMERA, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_create(&th_GrabCamera, "th_GrabCamera", 0, PRIORITY_TGRABCAMERA, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_create(&th_CloseCamera, "th_CloseCamera", 0, PRIORITY_TCLOSECAMERA, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_findArena, "th_findArena,", 0, PRIORITY_TFINDARENA, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_getRobotPosition, "th_getRobotPosition", 0, PRIORITY_TGETROBOTPOSITION, 0))
    {
        cerr << "Error task create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks created successfully" << endl
         << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof(Message *) * 50, Q_UNLIMITED, Q_FIFO)) < 0)
    {
        cerr << "Error msg queue create: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl
         << flush;
}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run()
{
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void (*)(void *)) & Tasks::ServerTask, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void (*)(void *)) & Tasks::SendToMonTask, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void (*)(void *)) & Tasks::ReceiveFromMonTask, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void (*)(void *)) & Tasks::OpenComRobot, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void (*)(void *)) & Tasks::StartRobotTask, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void (*)(void *)) & Tasks::MoveTask, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_BatteryLevel, (void (*)(void *)) & Tasks::BatteryLevel, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_RunWatchdog, (void (*)(void *)) & Tasks::RunWatchdog, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_OpenCamera, (void (*)(void *)) & Tasks::OpenCamera, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_start(&th_GrabCamera, (void (*)(void *)) & Tasks::GrabCamera, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_start(&th_CloseCamera, (void (*)(void *)) & Tasks::CloseCamera, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_start(&th_findArena, (void (*)(void *)) & Tasks::FindArena, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_start(&th_getRobotPosition, (void (*)(void *)) & Tasks::GetRobotPosition, this))
    {
        cerr << "Error task start: " << strerror(-err) << endl
             << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl
         << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop()
{
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join()
{
    cout << "Tasks synchronized" << endl
         << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg)
{
    int status;

    cout << "Start " << __PRETTY_FUNCTION__ << endl
         << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0)
        throw std::runtime_error{
            "Unable to start server on port " + std::to_string(SERVER_PORT)};
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl
         << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void *arg)
{
    Message *msg;

    cout << "Start " << __PRETTY_FUNCTION__ << endl
         << flush;

    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1)
    {
        cout << "wait msg to send" << endl
             << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl
             << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg)
{
    Message *msgRcv;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl
         << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/

    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl
         << flush;

    while (1)
    {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl
             << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST))
        {
            robot.Stop();
            monitor.Close();
            robot.Close();
            rt_sem_v(&sem_CloseCamera);
            cout << "Monitor is lost" << endl
                 << flush;
            delete (msgRcv);
            exit(-1);
        }
        else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN))
        {
            rt_sem_v(&sem_openComRobot);
        }
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD))
        {
            rt_sem_v(&sem_startRobot);
        }
        else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                 msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                 msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                 msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                 msgRcv->CompareID(MESSAGE_ROBOT_STOP))
        {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN))
        {
            rt_sem_v(&sem_OpenCamera);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE))
        {
            rt_sem_v(&sem_CloseCamera);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA))
        {
            rt_sem_v(&sem_arena);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM))
        {
            rt_sem_v(&sem_testArena);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM))
        {
            arena = Arena();
            rt_sem_v(&sem_testArena);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START))
        {
            rt_sem_v(&sem_position);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP))
        {
            positionActivated = false;
        }

        delete (msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg)
{
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl
         << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1)
    {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl
             << flush;

        Message *msgSend;
        if (status < 0)
        {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        }
        else
        {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg)
{
    cout << "Start " << __PRETTY_FUNCTION__ << endl
         << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1)
    {
        int rs;
        Message *msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot with watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);

        msgSend = robot.Write(robot.StartWithWD()); // On démarre le robot avec le watchdog
        rt_mutex_release(&mutex_robot);
        rt_sem_v(&sem_RunWtchg);
        // Creation du reload du watchdog
        if (msgSend->GetID() == MESSAGE_ANSWER_ACK)
        {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl
             << flush;
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg)
{
    int rs;
    int cpMove;

    cout << "Start " << __PRETTY_FUNCTION__ << endl
         << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1)
    {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);

        rs = robotStarted;

        rt_mutex_release(&mutex_robotStarted);

        if (rs == 1)
        {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);

            cout << " move: " << cpMove;

            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl
             << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg)
{
    int err;
    if ((err = rt_queue_write(queue, (const void *)&msg, sizeof((const void *)&msg), Q_NORMAL)) < 0)
    {
        cerr << "Write in queue failed: " << strerror(-err) << endl
             << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue)
{
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof((void *)&msg), TM_INFINITE)) < 0)
    {
        cout << "Read in queue failed: " << strerror(-err) << endl
             << flush;
        throw std::runtime_error{"Error in read in queue"};
    } /** else {
         cout << "@msg :" << msg << endl << flush;
     } /**/

    return msg;
}

/**
 * Return battery level
 * @param none
 * @return nothing, battery level is send to the queue
 */
void Tasks::BatteryLevel()
{
    int rs;
    Message *msg;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    while (1)
    {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1)
        {
            cout << "Periodic battery update" << __PRETTY_FUNCTION__ << endl
                 << flush;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = robot.Write(robot.GetBattery());
            Compteur(msg);
            rt_mutex_release(&mutex_robot);
            WriteInQueue(&q_messageToMon, msg);
        }
    }
}

void Tasks::RunWatchdog()
{
    cout << "Demarrage avec Watchdog " << __PRETTY_FUNCTION__ << endl
         << flush;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_sem_p(&sem_RunWtchg, TM_INFINITE);
    int rs;
    rt_task_set_periodic(NULL, TM_NOW, 1000000000); // 0.5s
    while (1)
    {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1)
        {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(robot.ReloadWD());
            cout << "Redemmarage Watchdog" << __PRETTY_FUNCTION__ << endl
                 << flush;
            rt_mutex_release(&mutex_robot);
        }
    }
}

void Tasks::Compteur(Message *msg)
{
    if ((msg->CompareID(MESSAGE_ANSWER_COM_ERROR)) || (msg->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)))
    {
        cout << "###Erreur detecte : incrementation compteur" << endl
             << flush;
        NbErreur++;
        if (NbErreur > 3)
        {
            cout << "################################" << endl;
            cout << "Erreur detecte : arret du robot" << endl
                 << flush;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Stop();
            robot.Close();
            rt_mutex_release(&mutex_robot);
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            rt_mutex_release(&mutex_robotStarted);
            NbErreur = 0;
        }
    }
}
// Fonctionnalité 14
void Tasks::OpenCamera()
{
    cout << "Start " << __PRETTY_FUNCTION__ << endl;

    while (1)
    {
        rt_sem_p(&sem_OpenCamera, TM_INFINITE);
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        if (!camera->Open())
        {
            cerr << "La camera ne s'ouvre pas" << endl
                 << flush;
        }
        rt_mutex_release(&mutex_camera);
    }
}

void Tasks::GrabCamera()
{
    bool arene = false;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1)
    {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_bool_arena, TM_INFINITE);
        arene = ArenaFound;
        rt_mutex_release(&mutex_bool_arena);
        if (camera->IsOpen())
        {
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            Img img = camera->Grab();
            rt_mutex_release(&mutex_camera); // limite la zone critique pour opti donc on met des qu'on peut le release
            if (arene)
            {
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                img.DrawArena(arena);
                rt_mutex_release(&mutex_arena);
            }
            MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, &img);
            cerr << endl
                 << flush;
            WriteInQueue(&q_messageToMon, msgImg);
        }
    }
}

// Fin Fonctionnalité 15

// Fonctionnalité 16
void Tasks::CloseCamera()
{
    while (1)
    {
        rt_sem_p(&sem_CloseCamera, TM_INFINITE);
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        camera->Close();
        rt_mutex_release(&mutex_camera);
    }
}

// Fin Fonctionnalité 16

void Tasks::FindArena(void *arg)
{
    cout << "Start " << __PRETTY_FUNCTION__ << endl
         << flush;

    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/

    while (1)
    {
        rt_sem_p(&sem_arena, TM_INFINITE);

        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        Img *img = new Img(camera->Grab());

        rt_mutex_acquire(&mutex_arena, TM_INFINITE);
        arena = img->SearchArena();
        rt_mutex_release(&mutex_arena);

        if (arena.IsEmpty())
        {
            Message *msg = new Message(MESSAGE_ANSWER_NACK);
            WriteInQueue(&q_messageToMon, msg);
            rt_mutex_release(&mutex_camera);
        }
        else
        {
            img->DrawArena(arena);
            rt_mutex_acquire(&mutex_bool_arena, TM_INFINITE);
            ArenaFound = true;
            rt_mutex_release(&mutex_bool_arena);
            MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
            WriteInQueue(&q_messageToMon, msgImg);
            cout << "reponse arene" << endl
                 << flush;
            rt_sem_p(&sem_testArena, TM_INFINITE);
            rt_mutex_release(&mutex_camera);
        }

        cout << "finito" << endl
             << flush;
    }
}

void Tasks::GetRobotPosition(void *arg)
{
    bool arene = false;
    MessagePosition *msgPos;
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1)
    {
        rt_sem_p(&sem_position, TM_INFINITE);

        rt_mutex_acquire(&mutex_camera, TM_INFINITE);

        positionActivated = true;
        rt_mutex_acquire(&mutex_bool_arena, TM_INFINITE);
        arene = ArenaFound;
        rt_mutex_release(&mutex_bool_arena);
        while (positionActivated)
        {
            Img *img = new Img(camera->Grab());
            std::list<Position> positions = img->SearchRobot(arena);
            if (positions.empty())
            {

                msgPos = new MessagePosition(MESSAGE_CAM_POSITION, positions.front());
                cout << "Positions Not Found" << endl
                     << flush;
            }
            else
            {
                img->DrawRobot(positions.front());
                if (arene)
                {
                    rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                    img->DrawArena(arena);
                    rt_mutex_release(&mutex_arena);
                }
                msgPos = new MessagePosition(MESSAGE_CAM_POSITION, positions.front());
                cout << "Positions Found" << endl
                     << flush;
            }

            MessageImg *msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
            WriteInQueue(&q_messageToMon, msgImg);
            WriteInQueue(&q_messageToMon, msgPos);
        }

        rt_mutex_release(&mutex_camera);
    }
}
