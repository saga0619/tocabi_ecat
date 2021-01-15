#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>

#include "ethercat.h"
#include "ecat_settings.h"
#include <stdarg.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

namespace EtherCAT_Elmo
{
    enum MODE_OF_OPERATION
    {
        ProfilePositionmode = 1,
        ProfileVelocitymode = 3,
        ProfileTorquemode = 4,
        Homingmode = 6,
        InterpolatedPositionmode = 7,
        CyclicSynchronousPositionmode = 8,
        CyclicSynchronousVelocitymode = 9,
        CyclicSynchronousTorquemode = 10,
        CyclicSynchronousTorquewithCommutationAngle = 11
    };

    struct ElmoGoldDevice
    {
        struct elmo_gold_tx
        {
            int32_t targetPosition;
            int32_t targetVelocity;
            int16_t targetTorque;
            uint16_t maxTorque;
            uint16_t controlWord;
            int8_t modeOfOperation;
        };
        struct elmo_gold_rx
        {
            int32_t positionActualValue;
            //int32_t positionFollowingErrrorValue;
            uint32_t hommingSensor;
            uint16_t statusWord;
            //int8_t modeOfOperationDisplay;
            int32_t velocityActualValue;
            int16_t torqueActualValue;
            //int16_t torqueDemandValue;
            int32_t positionExternal;
        };
    };
} // namespace EtherCAT_Elmo

const int FAULT_BIT = 3;
const int OPERATION_ENABLE_BIT = 2;
const int SWITCHED_ON_BIT = 1;
const int READY_TO_SWITCH_ON_BIT = 0;

enum
{
    CW_SHUTDOWN = 6,
    CW_SWITCHON = 7,
    CW_ENABLEOP = 15,
    CW_DISABLEOP = 7,
};

namespace ElmoHommingStatus
{
    enum FZResult
    {
        SUCCESS = 11,
        FAILURE = 22
    };
}; // namespace ElmoHommingStatus

struct ElmoHomming
{
    bool hommingElmo;
    bool hommingElmo_before;
    bool startFound = false;
    bool endFound = false;
    int findZeroSequence = 0;
    double initTime;
    double initPos;
    double posStart;
    double posEnd;
    double req_length = 0.2;
    double firstPos;
    double init_direction = 1;
    int result;
};

enum ElmoJointState
{
    MOTOR_COMMUTATION = 1,
    MOTOR_OPERATION_READY = 2,
    MOTOR_SEARCHING_ZP = 3,
    MOTOR_SEARCHING_MANUAL = 4,
    MOTOR_SEARCHING_COMPLETE = 5,
    MOTOR_SAFETY_LOCK = 6,
    MOTOR_SAFETY_DISABLED = 7,
};

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

int stateElmo[ELMO_DOF];
int stateElmo_before[ELMO_DOF];

bool torqueCCEnable;
double torqueCC_recvt;
double torqueCC_comt;

double control_time_real_;

bool hommingElmo[ELMO_DOF];
bool hommingElmo_before[ELMO_DOF];

int ElmoSafteyMode[ELMO_DOF];

EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *rxPDO[ELMO_DOF];
EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *txPDO[ELMO_DOF];

bool ElmoConnected = false;
bool ElmoTerminate = false;

std::vector<int> fz_group1;
bool fz_group1_check = false;
std::vector<int> fz_group2;
bool fz_group2_check = false;
std::vector<int> fz_group3;
bool fz_group3_check = false;
int fz_group = 0;

bool ConnectionUnstableBeforeStart = false;

int bootseq = 0;
//int bootseq
const int firstbootseq[5] = {0, 33, 35, 8, 64};
const int secondbootseq[4] = {0, 33, 35, 39};

bool ecat_connection_ok = false;

bool ecat_number_ok = false;
bool ecat_WKC_ok = false;
bool commutation_check = true;
bool commutation_ok = false;
bool commutation_fail = false;

bool zp_waiting_low_switch = false;
bool zp_waiting_upper_switch = false;

bool zp_init_check = true;
bool zp_low_check = false;
bool zp_upper_check = false;
bool zp_ok = false;
bool zp_fail = false;
bool zp_load_ok = true;

atomic<bool> de_operation_ready{false};
atomic<bool> de_emergency_off{false};
atomic<bool> de_shutdown{false};
atomic<bool> de_ecat_lost{false};
atomic<bool> de_ecat_lost_before{false};
atomic<bool> de_ecat_recovered{false};
atomic<bool> de_controlword{false};

array<atomic<double>, ELMO_DOF> q_elmo_;
array<atomic<double>, ELMO_DOF> q_dot_elmo_;
array<atomic<double>, ELMO_DOF> torque_elmo_;
array<atomic<double>, ELMO_DOF> q_ext_elmo_;
array<atomic<double>, ELMO_DOF> q_ext_mod_elmo_;
array<atomic<double>, ELMO_DOF> q_desired_;
array<atomic<double>, ELMO_DOF> torque_desired_;

void ethercatThread1();
void ethercatThread2();
void ethercatCheck();

double elmoJointMove(double init, double angle, double start_time, double traj_time);
bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord);

void checkJointSafety();
void checkJointStatus();
void sendJointStatus();
void getJointCommand();

void saveCommutationLog();
void loadCommutationLog();

void saveZeroPoint();
void loadZeroPoint();

void emergencyOff();

int kbhit(void);