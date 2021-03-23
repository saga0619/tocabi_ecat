#include <pthread.h>
#include <atomic>

//per link
//Jac * 4 
//33 * 6 * 39 * 4

#define MODEL_DOF 33

typedef struct SHMmsgs
{
    pthread_mutex_t mutex;
    pthread_mutexattr_t mutexAttr;

    int status[MODEL_DOF];

    std::atomic<bool> statusWriting;
    float torqueActual[MODEL_DOF];
    float vel[MODEL_DOF];
    float pos[MODEL_DOF];
    float posExt[MODEL_DOF];

    //command val

    std::atomic<bool> commanding;
    int commandMode[MODEL_DOF];
    float torqueCommand[MODEL_DOF];
    float positionCommand[MODEL_DOF];


    std::atomic<int> t_cnt;
    std::atomic<bool> controllerReady;
    std::atomic<bool> reading;


    float lat_avg, lat_min, lat_max, lat_dev;
    float send_avg, send_min, send_max, send_dev;

} SHMmsgs;

SHMmsgs *shm_msgs_;
int shm_msg_id;
key_t shm_msg_key = 7056;


// Joint state
// 0 : ELMO_ERROR,
// 1 : OPERATION_READY,
// 2 : COMMUTATION_INITIALIZE,
// 3 : COMMUTATION_DONE,
// 4 : ZP_SEARCHING_ZP,
// 5 : ZP_SEARCH_COMPLETE,
// 6 : ZP_MANUAL_REQUIRED,
// 7 : ZP_NOT_ENOUGH_HOMMING,
// 8 : ZP_GOTO_ZERO,
// 9 : ZP_SUCCESS,
// 10 : SAFETY_VELOCITY_LIMIT,
// 11 : SAFETY_JOINT_LIMIT,
// 12 : SAFETY_TORQUE_LIMIT,
enum ESTATE{
ERROR,
OPERATION_READY,
COMMUTATION_INITIALIZE,
COMMUTATION_DONE,
ZP_SEARCHING_ZP,
ZP_SEARCH_COMPLETE,
ZP_MANUAL_REQUIRED,
ZP_NOT_ENOUGH_HOMMING,
ZP_GOTO_ZERO,
ZP_SUCCESS,
SAFETY_VELOCITY_LIMIT,
SAFETY_JOINT_LIMIT,
SAFETY_TORQUE_LIMIT,
};