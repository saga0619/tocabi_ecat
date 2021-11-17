#include "tocabi_ecat/ecat_master.h"
// #include "bitset"

pthread_cond_t rcv_cond_;
pthread_mutex_t rcv_mtx_;

const int FAULT_BIT = 3;
const int OPERATION_ENABLE_BIT = 2;
const int SWITCHED_ON_BIT = 1;
const int READY_TO_SWITCH_ON_BIT = 0;

ElmoHomming elmofz[ELMO_DOF];
ElmoState elmost[ELMO_DOF];
int ElmoMode[ELMO_DOF];
char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

int START_N = -1;
int Q_START = -1;       //g_init_args.q_start_;
int PART_ELMO_DOF = -1; //g_init_args.ecat_slave_num;

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

bool diag_switch = false;

int fz_group1[18] = {
    Neck_Joint, Head_Joint,
    R_Shoulder1_Joint, R_Shoulder2_Joint, R_Shoulder3_Joint, R_Armlink_Joint, R_Elbow_Joint, R_Forearm_Joint, R_Wrist1_Joint, R_Wrist2_Joint,
    L_Shoulder1_Joint, L_Shoulder2_Joint, L_Shoulder3_Joint, L_Armlink_Joint, L_Elbow_Joint, L_Forearm_Joint, L_Wrist1_Joint, L_Wrist2_Joint};

int fz_group2[3] = {
    Upperbody_Joint, Waist1_Joint, Waist2_Joint};

int fz_group3[12] = {
    R_HipYaw_Joint, R_HipRoll_Joint, R_HipPitch_Joint, R_Knee_Joint, R_AnklePitch_Joint, R_AnkleRoll_Joint,
    L_HipYaw_Joint, L_HipRoll_Joint, L_HipPitch_Joint, L_Knee_Joint, L_AnklePitch_Joint, L_AnkleRoll_Joint};

bool fz_group1_check = false;
bool fz_group2_check = false;
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

bool wait_kill_switch = true;
bool wait_time_over = false;
bool check_commutation = true;
bool check_commutation_first = true;
bool query_check_state = false;
bool zp_lower_calc = true;

volatile bool control_mode = false;

volatile int rcv_cnt;

int wait_cnt = 0;

int commutation_joint = 0;
long cycle_count = 0;

volatile bool de_operation_ready;
volatile bool de_emergency_off;
volatile bool de_shutdown;
volatile bool de_ecat_lost;
volatile bool de_ecat_lost_before;
volatile bool de_ecat_recovered;
volatile bool de_initialize;
volatile bool de_commutation_done;
volatile bool de_zp_sequence;
volatile bool de_zp_upper_switch;
volatile bool de_zp_lower_switch;
volatile int de_debug_level;
volatile bool de_zp_upper;
volatile bool de_zp_lower;

int8_t state_elmo_[ELMO_DOF];
int8_t state_zp_[ELMO_DOF];
int8_t state_safety_[ELMO_DOF];

bool force_control_mode = false;
bool pos_hold_switch = false;
int soem_freq = 0;
int expected_counter = 0;
int period_ns = 0;

int joint_state_elmo_[ELMO_DOF]; //sendstate
int joint_state_[ELMO_DOF];      //sendstate

float q_elmo_[ELMO_DOF];      //sendstate
float q_dot_elmo_[ELMO_DOF];  //sendstate
float torque_elmo_[ELMO_DOF]; //sendstate
float q_ext_elmo_[ELMO_DOF];

float q_[ELMO_DOF];      //sendstate
float q_dot_[ELMO_DOF];  //sendstate
float torque_[ELMO_DOF]; //sendstate
float q_ext_[ELMO_DOF];

int command_mode_[ELMO_DOF];
float torque_desired_elmo_[ELMO_DOF]; //get torque command
float q_desired_elmo_[ELMO_DOF];      //get joint command
float torque_desired_[ELMO_DOF];      //get torque command
float q_desired_[ELMO_DOF];           //get joint command

double q_zero_point[ELMO_DOF];

double q_zero_elmo_[ELMO_DOF];
double q_zero_mod_elmo_[ELMO_DOF];

int maxTorque = 0;

int shm_id_;
SHMmsgs *shm_msgs_;

bool status_log = false;
const char cred[] = "\033[0;31m";
const char creset[] = "\033[0m";
const char cblue[] = "\033[0;34m";
const char cgreen[] = "\033[0;32m";
const char cyellow[] = "\033[0;33m";

int lat_avg, lat_min, lat_max, lat_dev;
int send_avg, send_min, send_max, send_dev;

float lat_avg2, lat_min2, lat_max2, lat_dev2;
float send_avg2, send_min2, send_max2, send_dev2;

int low_rcv_ovf, low_mid_ovf, low_snd_ovf;
int low_rcv_us, low_mid_us, low_snd_us;
float low_rcv_avg, low_rcv_max;
float low_mid_avg, low_mid_max;
float low_snd_avg, low_snd_max;

unsigned long long g_cur_dc32 = 0;
unsigned long long g_pre_dc32 = 0;
long long g_diff_dc32 = 0;
long long g_cur_DCtime = 0, g_max_DCtime = 0;
int g_PRNS = period_ns;
struct timespec g_ts;
int64 g_toff; //, gl_delta;

TocabiInitArgs g_init_args;

void ec_sync(int64 reftime, int64 cycletime, int64 &offsettime)
{
    static int64 integral = 0;
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    offsettime = -(delta / 100) - (integral / 20);
}

void getErrorName(int err_register, char *err)
{
    // 3 characters

    const char *codes[8] = {"ERR",
                            "CRT",
                            "VTG",
                            "TPR",
                            "COM",
                            "DEV",
                            "RES",
                            "MAN"};

    memset(err, ' ', 4 * 8);
    err[32] = 0; // ending word
    for (int i = 0; i < 8; ++i)
    {
        if (err_register & 1 << i)
        {
            memcpy(err + i * 4, codes[i], 3);
        }
    }
}
void printStatusword(const uint16_t statusWord)
{
    char STAT[10] = {'R', 'S', 'E', 'F'};

    for (int i = 0; i < 4; i++)
    {
        if (statusWord & (1 << i))
        {
            printf("%c", STAT[i]);
        }
        else
        {
            printf(" ");
        }
    }
    printf("    ");
}

void checkFault(const uint16_t statusWord, int slave)
{
    char err_text[100] = {0};
    const bool read_sdo = false;

    if (statusWord & (1 << FAULT_BIT))
    {
        if (read_sdo)
        {
            char data1[128] = {0};
            char data2[128] = {0};
            int data_length = sizeof(data1) - 1;
            printf("ECAT %d : [Fault at slave %d] reading SDO...\n", g_init_args.ecat_device, slave);
            ec_SDOread(slave, 0x1001, 0, false, &data_length, &data1, EC_TIMEOUTRXM);
            ec_SDOread(slave, 0x603f, 0, false, &data_length, &data2, EC_TIMEOUTRXM);
            // ec_SDOread(slave, 0x306a, 0, false, %data_length, &data3, EC_TIMEOUTRXM);
            int reg = *(uint8_t *)data1;
            int errcode = *(uint16_t *)data2;
            printf("ECAT %d : [Err slave %d] Err code: %d Err register: %d", g_init_args.ecat_device, slave, reg, errcode);
            getErrorName(reg, err_text);
            printf("#%s#\n", err_text);
        }
        else
        {
            printf("ECAT %d : [Fault at slave %d] set safety lock but not reading SDO...\n", g_init_args.ecat_device, slave);
            // ElmoSafteyMode[slave] = 1;
        }
    }
}
void cnt_print(int cnt)
{
    if (cnt == 0)
    {
        printf("%d\t", cnt);
    }
    else
    {
        printf("%s%d\t%s", cred, cnt, creset);
    }
}

void ecatDiagnoseOnChange()
{
    uint8 link_lost1[ELMO_DOF]; //link lost counter
    uint8 link_lost2[ELMO_DOF];

    uint8 fe_lost1[ELMO_DOF]; //frame error counter
    uint8 fe_lost2[ELMO_DOF];

    uint8 ple_lost1[ELMO_DOF]; //physical error counter
    uint8 ple_lost2[ELMO_DOF];

    uint8 fre_lost1[ELMO_DOF]; //frame error counter
    uint8 fre_lost2[ELMO_DOF];

    uint8 process_unit_error[ELMO_DOF];

    static uint8 link_lost1_p[ELMO_DOF]; //link lost counter
    static uint8 link_lost2_p[ELMO_DOF];

    static uint8 fe_lost1_p[ELMO_DOF]; //frame error counter
    static uint8 fe_lost2_p[ELMO_DOF];

    static uint8 ple_lost1_p[ELMO_DOF]; //physical error counter
    static uint8 ple_lost2_p[ELMO_DOF];

    static uint8 fre_lost1_p[ELMO_DOF]; //frame error counter
    static uint8 fre_lost2_p[ELMO_DOF];

    static uint8 process_unit_error_p[ELMO_DOF];

    // uint8 link_lost3[ELMO_DOF]; //link lost counter
    // uint8 link_lost4[ELMO_DOF];

    // uint8 ple_lost3[ELMO_DOF]; //physical error counter
    // uint8 ple_lost4[ELMO_DOF];

    int wc = 0;

    for (int slave = 1; slave <= ec_slavecount; slave++)
    {
        wc = ec_FPRD(ec_slave[slave].configadr, 0x310, sizeof(link_lost1[slave - 1]), &link_lost1[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x311, sizeof(link_lost2[slave - 1]), &link_lost2[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x300, sizeof(fe_lost1[slave - 1]), &fe_lost1[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x302, sizeof(fe_lost2[slave - 1]), &fe_lost2[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x301, sizeof(ple_lost1[slave - 1]), &ple_lost1[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x303, sizeof(ple_lost2[slave - 1]), &ple_lost2[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x308, sizeof(fre_lost1[slave - 1]), &fre_lost1[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x309, sizeof(fre_lost2[slave - 1]), &fre_lost2[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x30C, sizeof(process_unit_error[slave - 1]), &process_unit_error[slave - 1], EC_TIMEOUTRET);

        // wc = ec_FPRD(ec_slave[slave].configadr, 0x312, sizeof(link_lost3[slave - 1]), &link_lost3[slave - 1], EC_TIMEOUTRET);

        // wc = ec_FPRD(ec_slave[slave].configadr, 0x313, sizeof(link_lost4[slave - 1]), &link_lost4[slave - 1], EC_TIMEOUTRET);

        // wc = ec_FPRD(ec_slave[slave].configadr, 0x305, sizeof(ple_lost3[slave - 1]), &ple_lost3[slave - 1], EC_TIMEOUTRET);

        // wc = ec_FPRD(ec_slave[slave].configadr, 0x307, sizeof(ple_lost4[slave - 1]), &ple_lost4[slave - 1], EC_TIMEOUTRET);
    }

    bool link_lost_b = false;
    bool crc_error_b = false;
    bool rx_error_b = false;
    bool frd_error_b = false;
    bool process_unit_error_b = false;

    for (int i = 0; i < ec_slavecount; i++)
    {
        if (link_lost1_p[i] != link_lost1[i])
            link_lost_b = true;
        if (link_lost2_p[i] != link_lost2[i])
            link_lost_b = true;

        if (fe_lost1_p[i] != fe_lost1[i])
            crc_error_b = true;
        if (fe_lost2_p[i] != fe_lost2[i])
            crc_error_b = true;

        if (ple_lost1_p[i] != ple_lost1[i])
            rx_error_b = true;
        if (ple_lost2_p[i] != ple_lost2[i])
            rx_error_b = true;

        if (fre_lost1_p[i] != fre_lost1[i])
            frd_error_b = true;
        if (fre_lost2_p[i] != fre_lost2[i])
            frd_error_b = true;

        if (process_unit_error_p[i] != process_unit_error[i])
            process_unit_error_b = true;
    }

    if (link_lost_b)
    {
        printf("ECAT %d : Link Lost Event! \n", g_init_args.ecat_device);
        printf("Link Lost Cnt 0 : \t");
        for (int i = 0; i < ec_slavecount; i++)
        {
            cnt_print(link_lost1[i]);
        }
        printf("\nLink Lost Cnt 1 : \t");
        for (int i = 0; i < ec_slavecount; i++)
        {
            cnt_print(link_lost2[i]);
        }
        printf("\n\n");
    }

    if (crc_error_b)
    {
        printf("ECAT %d : CRC ERROR Event! \n", g_init_args.ecat_device);
        printf("  CRC Err Cnt 0 : \t");
        for (int i = 0; i < ec_slavecount; i++)
        {
            cnt_print(fe_lost1[i]);
        }
        printf("\n  CRC Err Cnt 1 : \t");
        for (int i = 0; i < ec_slavecount; i++)
        {
            cnt_print(fe_lost2[i]);
        }
        printf("\n\n");
    }

    if (frd_error_b)
    {

        printf("ECAT %d : Forwarded Error Event! \n", g_init_args.ecat_device);
        printf(" Forw Err Cnt 0 : \t");
        for (int i = 0; i < ec_slavecount; i++)
        {
            cnt_print(fre_lost1[i]);
        }

        printf("\n Forw Err Cnt 1 : \t");
        for (int i = 0; i < ec_slavecount; i++)
        {
            cnt_print(fre_lost2[i]);
        }
        printf("\n\n");
    }

    if (rx_error_b)
    {
        printf("ECAT %d : RX Error Event! \n", g_init_args.ecat_device);
        printf("   RX Err Cnt 0 : \t");
        for (int i = 0; i < ec_slavecount; i++)
        {
            cnt_print(ple_lost1[i]);
        }
        printf("\n   RX Err Cnt 1 : \t");
        for (int i = 0; i < ec_slavecount; i++)
        {
            cnt_print(ple_lost2[i]);
        }
        printf("\n\n");
    }

    if (process_unit_error_b)
    {
        printf("ECAT %d : EPU Error Event! \n", g_init_args.ecat_device);
        printf("    EPU Err Cnt : \t");
        for (int i = 0; i < ec_slavecount; i++)
        {
            cnt_print(process_unit_error[i]);
        }
        printf("\n\n");
    }

    for (int i = 0; i < ec_slavecount; i++)
    {
        link_lost1_p[i] = link_lost1[i]; //link lost counter
        link_lost2_p[i] = link_lost2[i];

        fe_lost1_p[i] = fe_lost1[i]; //frame error counter
        fe_lost2_p[i] = fe_lost2[i];

        ple_lost1_p[i] = ple_lost1[i]; //physical error counter
        ple_lost2_p[i] = ple_lost2[i];

        fre_lost1_p[i] = fre_lost1[i]; //frame error counter
        fre_lost2_p[i] = fre_lost2[i];

        process_unit_error_p[i] = process_unit_error[i];
    }
}

void ecatDiagnose()
{
    uint8 link_lost1[ELMO_DOF]; //link lost counter
    uint8 link_lost2[ELMO_DOF];

    uint8 fe_lost1[ELMO_DOF]; //frame error counter
    uint8 fe_lost2[ELMO_DOF];

    uint8 ple_lost1[ELMO_DOF]; //physical error counter
    uint8 ple_lost2[ELMO_DOF];

    uint8 fre_lost1[ELMO_DOF]; //frame error counter
    uint8 fre_lost2[ELMO_DOF];

    uint8 process_unit_error[ELMO_DOF];

    // uint8 link_lost3[ELMO_DOF]; //link lost counter
    // uint8 link_lost4[ELMO_DOF];

    // uint8 ple_lost3[ELMO_DOF]; //physical error counter
    // uint8 ple_lost4[ELMO_DOF];

    int wc = 0;

    for (int slave = 1; slave <= ec_slavecount; slave++)
    {
        wc = ec_FPRD(ec_slave[slave].configadr, 0x310, sizeof(link_lost1[slave - 1]), &link_lost1[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x311, sizeof(link_lost2[slave - 1]), &link_lost2[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x300, sizeof(fe_lost1[slave - 1]), &fe_lost1[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x302, sizeof(fe_lost2[slave - 1]), &fe_lost2[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x301, sizeof(ple_lost1[slave - 1]), &ple_lost1[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x303, sizeof(ple_lost2[slave - 1]), &ple_lost2[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x308, sizeof(fre_lost1[slave - 1]), &fre_lost1[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x309, sizeof(fre_lost2[slave - 1]), &fre_lost2[slave - 1], EC_TIMEOUTRET);

        wc = ec_FPRD(ec_slave[slave].configadr, 0x30C, sizeof(process_unit_error[slave - 1]), &process_unit_error[slave - 1], EC_TIMEOUTRET);

        // wc = ec_FPRD(ec_slave[slave].configadr, 0x312, sizeof(link_lost3[slave - 1]), &link_lost3[slave - 1], EC_TIMEOUTRET);

        // wc = ec_FPRD(ec_slave[slave].configadr, 0x313, sizeof(link_lost4[slave - 1]), &link_lost4[slave - 1], EC_TIMEOUTRET);

        // wc = ec_FPRD(ec_slave[slave].configadr, 0x305, sizeof(ple_lost3[slave - 1]), &ple_lost3[slave - 1], EC_TIMEOUTRET);

        // wc = ec_FPRD(ec_slave[slave].configadr, 0x307, sizeof(ple_lost4[slave - 1]), &ple_lost4[slave - 1], EC_TIMEOUTRET);
    }

    // printf("  Cnt : %d ");

    for (int i = 0; i < (cycle_count / 2000) % ec_slavecount; i++)
    {
        printf("OO");
    }
    printf("\n");

    printf("%d : %ld L avg : %5.2f max : %5.2f amax : %5.2f C avg : %5.2f max : %5.2f amax : %5.2f ovf : %d\n ", g_init_args.ecat_device, cycle_count / 2000, shm_msgs_->lat_avg / 1000.0, shm_msgs_->lat_max / 1000.0, shm_msgs_->lat_max2 / 1000.0, shm_msgs_->send_avg / 1000.0, shm_msgs_->send_max / 1000.0, shm_msgs_->send_max2 / 1000.0, shm_msgs_->send_ovf);

    // shm_msgs_->lat_avg = lat_avg;
    // shm_msgs_->lat_max = lmax;
    // shm_msgs_->lat_max2 = lamax;
    // shm_msgs_->lat_ovf = l_ovf;

    // shm_msgs_->send_avg = send_avg;
    // shm_msgs_->send_max = smax;
    // shm_msgs_->send_max2 = samax;
    // shm_msgs_->send_ovf = s_ovf;

    // c_count = 0;
    // total1 = 0;
    // total2 = 0;
    // lmax = 0;
    // smax = 0;

    printf("   Error Cnt Info : \t");
    for (int i = 0; i < ec_slavecount; i++)
    {
        printf("%d\t", i + 1);
    }

    printf("\n  CRC Err Cnt 0 : \t");
    for (int i = 0; i < ec_slavecount; i++)
    {
        cnt_print(fe_lost1[i]);
    }

    printf("\n  CRC Err Cnt 1 : \t");
    for (int i = 0; i < ec_slavecount; i++)
    {
        cnt_print(fe_lost2[i]);
    }

    printf("\n   RX Err Cnt 0 : \t");
    for (int i = 0; i < ec_slavecount; i++)
    {
        cnt_print(ple_lost1[i]);
    }
    printf("\n   RX Err Cnt 1 : \t");
    for (int i = 0; i < ec_slavecount; i++)
    {
        cnt_print(ple_lost2[i]);
    }
    printf("\n Forw Err Cnt 0 : \t");
    for (int i = 0; i < ec_slavecount; i++)
    {
        cnt_print(fre_lost1[i]);
    }

    printf("\n Forw Err Cnt 1 : \t");
    for (int i = 0; i < ec_slavecount; i++)
    {
        cnt_print(fre_lost2[i]);
    }
    printf("\n    EPU Err Cnt : \t");
    for (int i = 0; i < ec_slavecount; i++)
    {
        cnt_print(process_unit_error[i]);
    }
    printf("\nLink Lost Cnt 0 : \t");
    for (int i = 0; i < ec_slavecount; i++)
    {
        cnt_print(link_lost1[i]);
    }
    printf("\nLink Lost Cnt 1 : \t");
    for (int i = 0; i < ec_slavecount; i++)
    {
        cnt_print(link_lost2[i]);
    }

    // printf("\nLink Lost Counter 4 : \t");
    // for (int i = 0; i < ec_slavecount; i++)
    // {
    //     cnt_print(link_lost4[i]);
    // }
    // printf("\nPhysi Err Counter 3 : \t");
    // for (int i = 0; i < ec_slavecount; i++)
    // {
    //     cnt_print(ple_lost3[i]);
    // }
    // printf("\nPhysi Err Counter 4 : \t");
    // for (int i = 0; i < ec_slavecount; i++)
    // {
    //     cnt_print(ple_lost4[i]);
    // }
    printf("\n\n");
}

void ethercatCheck(TocabiInitArgs *targs)
{
    if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
    {
        if (needlf)
        {
            printf("S\n");
            needlf = FALSE;
            printf("\n");
        }
        // one ore more slaves are not responding
        ec_group[currentgroup].docheckstate = FALSE;
        ec_readstate();
        for (int slave = 1; slave <= ec_slavecount; slave++)
        {
            if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
            {
                ec_group[currentgroup].docheckstate = TRUE;
                if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                {
                    printf("%sERROR : slave %d is in SAFE_OP + ERROR, attempting ack.%s\n", cred, slave - 1, creset);
                    ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                    ec_writestate(slave);
                }
                else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                {
                    printf("%sWARNING : slave %d is in SAFE_OP, change to OPERATIONAL.%s\n", cred, slave - 1, creset);
                    ec_slave[slave].state = EC_STATE_OPERATIONAL;
                    ec_writestate(slave);
                }
                else if (ec_slave[slave].state > 0)
                {
                    if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("%sMESSAGE : slave %d reconfigured%s\n", cgreen, slave - 1, creset);
                    }
                }
                else if (!ec_slave[slave].islost)
                {
                    // re-check state
                    ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                    if (!ec_slave[slave].state)
                    {
                        ec_slave[slave].islost = TRUE;
                        printf("%sERROR : slave %d lost %s\n", cred, slave - 1, creset);
                    }
                }
            }
            if (ec_slave[slave].islost)
            {
                if (!ec_slave[slave].state)
                {
                    if (ec_recover_slave(slave, EC_TIMEOUTMON))
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("%sMESSAGE : slave %d recovered%s\n", cgreen, slave - 1, creset);
                    }
                }
                else
                {
                    ec_slave[slave].islost = FALSE;
                    printf("%sMESSAGE : slave %d found%s\n", cgreen, slave - 1, creset);
                }
            }
        }
    }
}

long getTimeDiff(struct timespec &from, struct timespec &to)
{
    long sec_diff = (to.tv_sec - from.tv_sec);
    long total_diff = sec_diff * SEC_IN_NSEC + (to.tv_nsec - from.tv_nsec);
    return total_diff;
}

/**
 @return: diff nanosec 
*/
long getTimeDiff(struct timespec &a)
{
    struct timespec cur_time;
    clock_gettime(CLOCK_MONOTONIC, &cur_time);

    long sec_diff = (cur_time.tv_sec - a.tv_sec);
    long total_diff = sec_diff * SEC_IN_NSEC + (cur_time.tv_nsec - a.tv_nsec);
    return total_diff;
}

void elmoInit()
{
    elmofz[R_Armlink_Joint].init_direction = -1.0;
    elmofz[L_Armlink_Joint].init_direction = -1.0;
    elmofz[R_Elbow_Joint].init_direction = -1.0;
    elmofz[Upperbody_Joint].init_direction = -1.0;

    elmofz[Waist2_Joint].init_direction = -1.0;

    elmofz[Neck_Joint].req_length = 0.11;

    elmofz[R_Elbow_Joint].req_length = 0.05;
    elmofz[L_Elbow_Joint].req_length = 0.05;
    elmofz[L_Forearm_Joint].req_length = 0.08;
    elmofz[R_Forearm_Joint].req_length = 0.14;

    elmofz[L_Shoulder1_Joint].req_length = 0.18;
    elmofz[L_Shoulder2_Joint].req_length = 0.15;
    elmofz[R_Shoulder2_Joint].req_length = 0.07;

    elmofz[R_Shoulder3_Joint].req_length = 0.03;
    elmofz[L_Shoulder3_Joint].req_length = 0.03;

    elmofz[L_Armlink_Joint].req_length = 0.15;

    elmofz[R_Wrist2_Joint].req_length = 0.05;
    elmofz[L_Wrist2_Joint].req_length = 0.05;

    elmofz[Waist2_Joint].req_length = 0.07;
    elmofz[Waist2_Joint].init_direction = -1.0;
    elmofz[Waist1_Joint].req_length = 0.07;

    q_zero_mod_elmo_[8] = 15.46875 * DEG2RAD;
    q_zero_mod_elmo_[7] = 16.875 * DEG2RAD;
    q_zero_mod_elmo_[Waist1_Joint] = -15.0 * DEG2RAD;
    q_zero_mod_elmo_[Upperbody_Joint] = 0.0541;

    memset(ElmoSafteyMode, 0, sizeof(int) * ELMO_DOF);
}
bool initTocabiArgs(const TocabiInitArgs &args)
{
    pthread_mutex_init(&rcv_mtx_, NULL);
    pthread_cond_init(&rcv_cond_, NULL);

    Q_START = args.q_start_;
    PART_ELMO_DOF = args.ecat_slave_num;
    START_N = args.ecat_slave_start_num;

    init_shm(shm_msg_key, shm_id_, &shm_msgs_);

    // shm_msgs_->shutdown = false;
    if (shm_msgs_->shutdown == true)
    {

        printf("ELMO %d : shm reset\n", args.ecat_device);
    }

    g_init_args = args;

    return true;
}

bool initTocabiSystem(const TocabiInitArgs &args)
{

    // const char *ifname1 = args.port1.c_str();
    char ifname2[100];
    strcpy(ifname2, args.port2);
    // char *ifname2 = args.port2;

    if (!ec_init_redundant(args.port1, ifname2))
    // if (!ec_init(args.port1))
    {
        printf("ELMO %d : No socket connection on %s / %s \nExcecute as root\n", args.ecat_device, args.port1, args.port2);
        return false;
    }
    printf("ELMO %d : ec_init on %s %s succeeded.\n", args.ecat_device, args.port1, args.port2);

    if (ec_config_init(FALSE) <= 0) // TRUE when using configtable to init slavtes, FALSE oherwise
    {
        printf("%sELMO : No slaves found!%s\n", cred, creset);
        return false;
    }

    elmoInit();

    printf("ELMO %d : %d / %d slaves found and configured.\n", args.ecat_device, ec_slavecount, args.ecat_slave_num); // ec_slavecount -> slave num

    if (ec_slavecount == args.ecat_slave_num)
    {
        ecat_number_ok = true;
    }
    else
    {
        // std::cout << cred << "WARNING : SLAVE NUMBER INSUFFICIENT" << creset << '\n';
        shm_msgs_->shutdown = true;
        return false;
    }
    /** CompleteAccess disabled for Elmo driver */
    for (int slave = 1; slave <= ec_slavecount; slave++)
    {
        //printf("ELMO : Has Slave[%d] CA? %s\n", slave, ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
        if (!(ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA))
        {
            printf("ELMO %d : slave[%d] CA? : false , shutdown request \n ", args.ecat_device, slave);
        }
        ec_slave[slave].CoEdetails ^= ECT_COEDET_SDOCA;
    }

    for (int slave = 1; slave <= ec_slavecount; slave++)
    {
        //0x1605 :  Target Position             32bit
        //          Target Velocity             32bit
        //          Max Torque                  16bit
        //          Control word                16bit
        //          Modes of Operation          16bit
        uint16 map_1c12[2] = {0x0001, 0x1605};

        //0x1a00 :  position actual value       32bit
        //          Digital Inputs              32bit
        //          Status word                 16bit
        //0x1a11 :  velocity actual value       32bit
        //0x1a13 :  Torque actual value         16bit
        //0x1a1e :  Auxiliary position value    32bit
        uint16 map_1c13[5] = {0x0004, 0x1a00, 0x1a11, 0x1a13, 0x1a1e}; //, 0x1a12};
        //uint16 map_1c13[6] = {0x0005, 0x1a04, 0x1a11, 0x1a12, 0x1a1e, 0X1a1c};
        int os;
        os = sizeof(map_1c12);
        ec_SDOwrite(slave, 0x1c12, 0, TRUE, os, map_1c12, EC_TIMEOUTRXM);
        os = sizeof(map_1c13);
        ec_SDOwrite(slave, 0x1c13, 0, TRUE, os, map_1c13, EC_TIMEOUTRXM);
    }
    /** if CA disable => automapping works */
    printf("ELMO %d : EC CONFIG MAP\n", args.ecat_device);
    ec_config_map(&IOmap);
    // ec_config_overlap_map(IOmap);

    //ecdc
#ifdef ECAT_DC
    printf("ELMO %d : EC CONFIG DC\n", args.ecat_device);
    ec_configdc();

    for (int i = 0; i < ec_slavecount; i++)
        ec_dcsync0(i + 1, TRUE, (uint32)args.period_ns, 0);

#endif
    while (EcatError)
        printf("%s\n", ec_elist2string());
    printf("ELMO %d : EC WAITING STATE TO SAFE_OP\n", args.ecat_device);
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    ec_readstate();
    // for (int cnt = 1; cnt <= ec_slavecount; cnt++)
    // {
    //     /* BEGIN USER CODE */

    //     printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
    //            cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
    //            ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);

    //     /* END USER CODE */
    // }
    if (args.ecat_device == 0)
    {
        uint16 wc, slaveh;
        uint32 t;
        uint64 t1;
        //read the copied system time of local clock
        t1 = 0;

        for (int i = 1; i <= ec_slavecount; i++)
        {
            printf("slave %d delay : %d ns \n", i, ec_slave[i].pdelay);
        }

        // for (int i = 1; i <= ec_slavecount; i++)
        // {
        //     t1 = 0;
        //     slaveh = ec_slave[i].configadr;
        //     wc = ec_FPRD(slaveh, ECT_REG_DCSYSTIME, sizeof(t1), &t1, EC_TIMEOUTRET);
        //     printf("System time of slave %d : %d\n", i, t1);
        //     // cerr << "System time of slave " << i << " : " << dec << t1 << endl;
        // }

        // //read the propagation delay of slaves
        // t = 0;
        // for (int i = 1; i <= ec_slavecount; i++)
        // {
        //     t = 0;
        //     slaveh = ec_slave[i].configadr;
        //     wc = ec_FPRD(slaveh, ECT_REG_DCSYSDELAY, sizeof(t), &t, EC_TIMEOUTRET);
        //     printf("Delay time of slave %d : %d\n", i, t);
        //     // cerr << "Delay time of slave " << i << " : " << dec << t << endl;
        // }

        // //read the offset time from reference time to slaves
        // t1 = 0;
        // for (int i = 1; i <= ec_slavecount; i++)
        // {
        //     t1 = 0;
        //     slaveh = ec_slave[i].configadr;
        //     wc = ec_FPRD(slaveh, ECT_REG_DCSYSOFFSET, sizeof(t1), &t1, EC_TIMEOUTRET);
        //     printf("Offset time of slave %d : %d\n", i, t1);
        //     // cerr << "Offset time of slave " << i << " : " << dec << t1 << endl;
        // }

        // //read the local clock of slaves
        // t1 = 0;
        // for (int i = 1; i <= ec_slavecount; i++)
        // {
        //     t1 = 0;
        //     slaveh = ec_slave[i].configadr;
        //     wc = ec_FPRD(slaveh, ECT_REG_DCSOF, sizeof(t1), &t1, EC_TIMEOUTRET);
        //     printf("Local time of slave %d : %d\n", i, t1);
        //     // cerr << "Local time of slave " << i << " : " << dec << t1 << endl;
        // }

        // //read the system time difference of slaves
        // t = 0;
        // for (int i = 1; i <= ec_slavecount; i++)
        // {
        //     t = 0;
        //     slaveh = ec_slave[i].configadr;
        //     wc = ec_FPRD(slaveh, ECT_REG_DCSYSDIFF, sizeof(t), &t, EC_TIMEOUTRET);
        //     printf("System time difference of slave %d : %d\n", i, t);
        //     // cerr << "System time difference of slave " << i << " : " << dec << t << endl;
        // }

        // //read the sync0 cycle start time
        // t1 = 0;
        // for (int i = 1; i <= ec_slavecount; i++)
        // {
        //     t = 0;
        //     slaveh = ec_slave[i].configadr;
        //     wc = ec_FPRD(slaveh, ECT_REG_DCSTART0, sizeof(t1), &t1, EC_TIMEOUTRET);
        //     printf("next sync0 start time of slave %d : %d\n", i, t1);

        //     // cerr << "next sync0 start time of slave " << i << " : " << dec << t1 << endl;
        // }
    }
    // void *digout;

    // for (int cnt = 1; cnt <= ec_slavecount; cnt++)
    // {
    //     printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
    //            cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
    //            ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
    //     printf("         Out:%8.8x,%4d In:%8.8x,%4d\n",
    //            (int *)ec_slave[cnt].outputs, ec_slave[cnt].Obytes, (int *)ec_slave[cnt].inputs, ec_slave[cnt].Ibytes);
    //     /* check for EL2004 or EL2008 */
    //     if (!digout && ((ec_slave[cnt].eep_id == 0x07d43052) || (ec_slave[cnt].eep_id == 0x07d83052)))
    //     {
    //         digout = ec_slave[cnt].outputs;
    //     }
    // }

    /* wait for all slaves to reach SAFE_OP state */

    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("ELMO %d : Request operational state for all slaves. Calculated workcounter : %d\n", args.ecat_device, expectedWKC);

    if (expectedWKC != 3 * args.ecat_slave_num)
    {
        // std::cout << cred << "WARNING : Calculated Workcounter insufficient!" << creset << '\n';
        ecat_WKC_ok = true;
    }

    /** going operational */
    ec_slave[0].state = EC_STATE_OPERATIONAL;

    /* send one valid process data to make outputs in slaves happy*/
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    /* request OP state for all slaves */
    ec_writestate(0);

    int wait_cnt = 40;

    int64 toff, gl_delta;
    toff = 0;
    unsigned long long cur_dc32 = 0;
    unsigned long long pre_dc32 = 0;
    long long diff_dc32 = 0;
    long long cur_DCtime = 0, max_DCtime = 0;

    /* wait for all slaves to reach OP state */
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 5000);
    } while (wait_cnt-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        printf("%sELMO %d : Not all slaves reached operational state.%s\n", cred, args.ecat_device, creset);
        ec_readstate();
        for (int slave = 1; slave <= ec_slavecount; slave++)
        {
            if (ec_slave[slave - 1].state != EC_STATE_OPERATIONAL)
            {
                printf("%sELMO %d : EtherCAT State Operation Error : Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s%s\n",
                       cred, args.ecat_device, slave - 1, ec_slave[slave - 1].state, ec_slave[slave - 1].ALstatuscode,
                       ec_ALstatuscode2string(ec_slave[slave - 1].ALstatuscode), creset);
            }
        }
        return false;
    }
    for (int slave = 1; slave <= ec_slavecount; slave++)
    {
        txPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *)(ec_slave[slave].outputs);
        rxPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *)(ec_slave[slave].inputs);
    }

    inOP = TRUE;
    const int PRNS = args.period_ns;
    period_ns = args.period_ns;

    // //ecdc
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
#ifdef ECAT_DC
    for (int i = 0; i < ec_slavecount; i++)
        ec_dcsync0(i + 1, TRUE, (uint32)args.period_ns, 2000);
    toff = 0;

    ec_send_processdata();

    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    cur_dc32 = (uint32_t)(ec_DCtime & 0xffffffff);

    long dc_remain_time = cur_dc32 % PRNS;
    ts.tv_nsec = ts.tv_nsec - ts.tv_nsec % PRNS + dc_remain_time;
    while (ts.tv_nsec >= SEC_IN_NSEC)
    {
        ts.tv_sec++;
        ts.tv_nsec -= SEC_IN_NSEC;
    }
#endif

    // std::printf("dc_remain_time : " << dc_remain_time << '\n';

    // clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

    /* cyclic loop */

    //Commutation Checking
    // st_start_time = std::chrono::steady_clock::now(); // TODO:timespec
    // printf("ELMO : Initialization Mode\n");

    query_check_state = true;
    g_toff = toff;
    g_cur_dc32 = cur_dc32;
    g_pre_dc32 = pre_dc32;
    g_diff_dc32 = diff_dc32;
    g_cur_DCtime = cur_DCtime;
    g_PRNS = PRNS;
    g_ts = ts;
    return true;
}

void cleanupTocabiSystem()
{
    deleteSharedMemory(shm_id_, shm_msgs_);
}

void *ethercatThread1(void *data)
{

    TocabiInitArgs *init_args = (TocabiInitArgs *)data;
    int64 toff = 0;
    unsigned long long cur_dc32 = g_cur_dc32;
    unsigned long long pre_dc32 = g_pre_dc32;
    long long diff_dc32 = g_diff_dc32;
    long long cur_DCtime = g_cur_dc32, max_DCtime = g_max_DCtime;
    int PRNS = g_PRNS;
    struct timespec ts;
    struct timespec ts_now;

    char IOmap[4096] = {};
    bool reachedInitial[ELMO_DOF] = {false};
    // force_control_mode = true;
    de_debug_level++;
    status_log = true;

    struct timespec ts_start;
    clock_gettime(CLOCK_MONOTONIC, &ts_start); //ADDED
    clock_gettime(CLOCK_MONOTONIC, &ts);       //ADDED

    struct timespec ts_tt;
    char buff[100];

    timespec_get(&ts_tt, TIME_UTC);
    ts_tt.tv_sec += 60 * 60 * 9;
    strftime(buff, sizeof buff, "%D %T", gmtime(&ts_tt.tv_sec));

    printf("ELMO %d : entering initialize START_N %d jointNUM %d QSTART %d | %s\n", init_args->ecat_device, START_N, init_args->ecat_slave_num, init_args->q_start_, buff);

    if (shm_msgs_->shutdown)
        printf("Shutdown Command Before Start\n");

    while (!shm_msgs_->shutdown)
    {
        if (force_control_mode)
        {
            break;
        }

        ts.tv_nsec += init_args->period_ns + toff;
        if (ts.tv_nsec >= SEC_IN_NSEC)
        {
            ts.tv_sec++;
            ts.tv_nsec -= SEC_IN_NSEC;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

        //std::this_thread::sleep_until(st_start_time + cycle_count * cycletime);
        cycle_count++;

        // printf("ELMO %d : entering initialize START_N %d jointNUM %d\n", init_args->ecat_device, START_N, init_args->ecat_slave_num);

        // if ((cycle_count % 2000) == 0)
        //     printf("ELMO %d running \n", g_init_args.ecat_device);
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_PACKET_TIMEOUT);

        control_time_real_ = cycle_count * init_args->period_ns / 1000000000.0;
        // if (init_args->is_main)
        //     shm_msgs_->control_time_real_ = control_time_real_;
        // control_time_real_ = std::chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - st_start_time).count() / 1000000.0;
        // while (EcatError)
        //     printf("%f %s", control_time_real_, ec_elist2string());

        for (int i = 0; i < ec_slavecount; i++)
        {
            // printf("%d\t",rxPDO[i]->statusWord);

            elmost[i].state = getElmoState(rxPDO[i]->statusWord);

            if (elmost[i].state != elmost[i].state_before)
            {
                state_elmo_[JointMap2[START_N + i]] = elmost[i].state;

                if (elmost[i].first_check)
                {
                    if (elmost[i].state == ELMO_NOTFAULT)
                    {
                        elmost[i].commutation_required = true;
                    }
                    else if (elmost[i].state == ELMO_FAULT)
                    {
                        //printf("slave : " << i << " commutation check complete at first\n");
                        elmost[i].commutation_not_required = true;
                    }
                    else if (elmost[i].state == ELMO_OPERATION_ENABLE)
                    {
                        //printf("slave : " << i << " commutation check complete with operation enable\n");
                        elmost[i].commutation_not_required = true;
                        elmost[i].commutation_ok = true;
                    }
                    else
                    {
                        //printf("first missing : slave : " << i << " state : " << elmost[i].state << '\n';
                    }
                    elmost[i].first_check = false;
                }
                else
                {
                    if (elmost[i].state == ELMO_OPERATION_ENABLE)
                    {
                        // printf("slave : %d commutation check complete with operation enable 2\n",i);
                        elmost[i].commutation_ok = true;
                        elmost[i].commutation_required = false;
                    }
                }
                query_check_state = true;
            }
            elmost[i].state_before = elmost[i].state;
        }
        // printf("\n");

        if (check_commutation)
        {
            if (check_commutation_first)
            {
                if (init_args->ecat_device == 0)
                {
                    printf("Commutation Status : \n");
                    for (int i = 0; i < ec_slavecount; i++)
                        printf("--");
                    printf("\n");
                    for (int i = 0; i < ec_slavecount; i++)
                        printf("%2d", (i - i % 10) / 10);
                    printf("\n");
                    for (int i = 0; i < ec_slavecount; i++)
                        printf("%2d", i % 10);
                    printf("\n");
                    printf("\n");
                    printf("\n");
                }
                check_commutation_first = false;
            }
            if (query_check_state)
            {
                if (init_args->ecat_device == 0)
                {
                    printf("\x1b[A\x1b[A\33[2K\r");
                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        if (elmost[i].state == ELMO_OPERATION_ENABLE)
                        {
                            printf("%s%2d%s", cgreen, elmost[i].state, creset);
                        }
                        else
                        {
                            printf("%2d", elmost[i].state);
                        }
                    }
                    printf("\n");
                    for (int i = 0; i < ec_slavecount; i++)
                        printf("--");
                    printf("\n");
                    fflush(stdout);
                }
                query_check_state = false;
            }
        }

        bool waitop = true;
        for (int i = 0; i < ec_slavecount; i++)
            waitop = waitop && elmost[i].commutation_ok;

        if (waitop)
        {
            static bool pub_once = true;

            if (pub_once)
            {

                // std::chrono::milliseconds commutation_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - st_start_time);
                //printf("ELMO : All slaves Operational in " << commutation_time.count() << "ms\n");
                long commutation_time = getTimeDiff(ts_start);
                if (commutation_time / 1e6 < 500)
                {

                    de_commutation_done = true;
                    check_commutation = false;
                    if (init_args->verbose)
                        printf("ELMO %d : Load ZP ... \n", init_args->ecat_device);
                }
                else
                {
                    if (saveCommutationLog())
                    {
                        if (init_args->verbose)
                            printf("\nELMO %d : Commutation is done, logging success\n", init_args->ecat_device);
                    }
                    else
                    {
                        printf("\nELMO %d : Commutation is done, logging failed\n", init_args->ecat_device);
                    }
                    de_commutation_done = true;
                    check_commutation = false;
                }

                pub_once = false;
            }
        }

        if (de_commutation_done)
        {
            static bool pub_once = true;
            if (pub_once)
            {

                if (loadZeroPoint())
                {
                    // if (init_args->verbose)
                    //     printf("ELMO %d : Initialize Complete \n", init_args->ecat_device);
                    break;
                }
                else
                {
                    printf("ELMO %d : ZeroPoint load failed. Ready to Search Zero Point \n", init_args->ecat_device);
                    de_zp_sequence = true;
                }
                pub_once = false;
            }
        }

        if (shm_msgs_->force_load_saved_signal)
        {
            loadZeroPoint(true);
            printf("ELMO 1 : force load ZP\n");
            break;
        }

        bool waitcm = true;
        for (int i = 0; i < ec_slavecount; i++)
            waitcm = waitcm && elmost[i].commutation_not_required;

        if (waitcm)
        {
            if (wait_kill_switch)
            {
                if (init_args->verbose)
                    printf("ELMO %d : Commutation state OK\n", init_args->ecat_device);
                //loadCommutationLog();
                loadZeroPoint();
                wait_kill_switch = false;
                check_commutation = false;
            }
            if (wait_cnt == 200)
            {
                printf("ELMO %d : slaves status are not OP! maybe kill switch is on?\n", init_args->ecat_device);
            }

            wait_cnt++;
        }
        else
        {
            int total_commutation_cnt = 0;
            for (int i = 0; i < ec_slavecount; i++)
            {
                if (elmost[i].commutation_required)
                {
                    total_commutation_cnt++;
                    if (total_commutation_cnt < 5)
                        controlWordGenerate(rxPDO[i]->statusWord, txPDO[i]->controlWord);
                    txPDO[i]->maxTorque = (uint16)1000; // originaly 1000
                }
            }
        }

        for (int slave = 1; slave <= ec_slavecount; slave++)
        {
            if (!elmost[slave - 1].commutation_required)
            {
                if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
                {
                    reachedInitial[slave - 1] = true;
                }

                if (reachedInitial[slave - 1])
                {
                    q_elmo_[START_N + slave - 1] = rxPDO[slave - 1]->positionActualValue * CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1];
                    hommingElmo[START_N + slave - 1] =
                        (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)1));
                    q_dot_elmo_[START_N + slave - 1] =
                        (((int32_t)ec_slave[slave].inputs[10]) +
                         ((int32_t)ec_slave[slave].inputs[11] << 8) +
                         ((int32_t)ec_slave[slave].inputs[12] << 16) +
                         ((int32_t)ec_slave[slave].inputs[13] << 24)) *
                        CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1];
                    torque_elmo_[START_N + slave - 1] =
                        (int16_t)(((int16_t)ec_slave[slave].inputs[14]) +
                                  ((int16_t)ec_slave[slave].inputs[15] << 8));
                    q_ext_elmo_[START_N + slave - 1] =
                        (((int32_t)ec_slave[slave].inputs[16]) +
                         ((int32_t)ec_slave[slave].inputs[17] << 8) +
                         ((int32_t)ec_slave[slave].inputs[18] << 16) +
                         ((int32_t)ec_slave[slave].inputs[19] << 24) - q_ext_mod_elmo_[START_N + slave - 1]) *
                        EXTCNT2RAD[START_N + slave - 1] * elmo_ext_axis_direction[START_N + slave - 1];
                    if (START_N + slave == 1 || START_N + slave == 2 || START_N + slave == 7 || START_N + slave == 19 || START_N + slave == 20 || START_N + slave == 16)
                    {
                        hommingElmo[START_N + slave - 1] = !hommingElmo[START_N + slave - 1];
                    }
                    txPDO[slave - 1]->maxTorque = (uint16)500; // originaly 1000
                }
            }
        }
        for (int i = 0; i < ec_slavecount; i++)
        {
            q_[JointMap2[START_N + i]] = q_elmo_[START_N + i];
            q_dot_[JointMap2[START_N + i]] = q_dot_elmo_[START_N + i];
            torque_[JointMap2[START_N + i]] = torque_elmo_[START_N + i];
            q_ext_[JointMap2[START_N + i]] = q_ext_elmo_[START_N + i];
            //joint_state_[JointMap2[START_N + i]] = joint_state_elmo_[START_N + i];
        }

        // for(int i=0;i<ec_slavecount;i++)
        // {
        //     shm_msgs_->pos[]
        // }

        //shm_msgs_->pos[JointMap2[START_N + i]] = q_elmo_[START_N + i];
        if (g_init_args.ecat_device == 1)
        {
            if (shm_msgs_->upper_init_signal)
            {
                de_zp_upper_switch = true;
                shm_msgs_->upper_init_signal = false;
            }
        }
        if (g_init_args.ecat_device == 2)
        {

            if (shm_msgs_->low_init_signal)
            {
                de_zp_lower_switch = true;
                shm_msgs_->low_init_signal = false;
            }
            if (shm_msgs_->waist_init_signal)
            {
                de_zp_upper_switch = true;
                shm_msgs_->waist_init_signal = false;
            }
        }

        sendJointStatus();
        if (de_zp_sequence)
        {
            static bool zp_upper = false;
            static bool zp_lower = false;

            if (de_zp_upper_switch)
            {
                printf("ELMO %d starting upper zp\n", init_args->ecat_device);
                // for (int i = 0; i < 8; i++)
                //     printf("L" << i << "\t";
                // for (int i = 0; i < 8; i++)
                //     printf("R" << i << "\t";
                // cout << '\n';
                elmofz[R_Shoulder3_Joint].findZeroSequence = 7;
                elmofz[R_Shoulder3_Joint].initTime = control_time_real_;
                elmofz[L_Shoulder3_Joint].findZeroSequence = 7;
                elmofz[L_Shoulder3_Joint].initTime = control_time_real_;

                for (int i = 0; i < ec_slavecount; i++)
                    hommingElmo_before[START_N + i] = hommingElmo[START_N + i];

                de_zp_upper = true;
                de_zp_upper_switch = false;
            }

            if (de_zp_lower_switch)
            {
                printf("starting lower zp\n");
                de_zp_lower_switch = false;
                zp_lower = true;
            }

            if (de_zp_upper)
            {
                if (init_args->ecat_device == 1) // Upper
                {

                    for (int i = 0; i < 18; i++)
                    {
                        findZeroPoint(fz_group1[i], control_time_real_);
                    }
                }
                else // lower
                {
                    for (int i = 0; i < 3; i++)
                    {
                        findZeroPoint(fz_group2[i], control_time_real_);
                    }
                }
                for (int i = 0; i < ec_slavecount; i++)
                    hommingElmo_before[START_N + i] = hommingElmo[START_N + i];
            }

            if (zp_lower)
            {
                if (zp_lower_calc)
                {
                    findzeroLeg();
                    zp_lower_calc = false;
                }
                else
                {
                    for (int i = 0; i < 6; i++)
                    {
                        findZeroPointlow(i + R_HipYaw_Joint, control_time_real_);
                        findZeroPointlow(i + L_HipYaw_Joint, control_time_real_);
                    }
                }
            }

            if (g_init_args.ecat_device == 1)
            {
                fz_group1_check = true;
                for (int i = 0; i < 18; i++)
                {
                    fz_group1_check = fz_group1_check && (elmofz[fz_group1[i]].result == ElmoHommingStatus::SUCCESS);
                }
            }

            if (g_init_args.ecat_device == 2)
            {
                fz_group2_check = true;
                for (int i = 0; i < 3; i++)
                {
                    fz_group2_check = fz_group2_check && (elmofz[fz_group2[i]].result == ElmoHommingStatus::SUCCESS);
                }
                fz_group3_check = true;
                for (int i = 0; i < 12; i++)
                {
                    fz_group3_check = fz_group3_check && (elmofz[fz_group3[i]].result == ElmoHommingStatus::SUCCESS);
                }
            }

            // if (fz_group1_check && (fz_group == 0))
            // {
            //     // printf("ELMO : arm zp done \n");
            //     // fz_group++;
            // }
            // if (fz_group2_check && (fz_group == 1))
            // {
            //     // fz_group++;
            //     // printf("ELMO : waist zp done\n");
            // }

            static bool low_verbose = true;
            if (low_verbose && fz_group3_check)
            {
                printf("ELMO : lowerbody zp done \n");
                low_verbose = false;
            }
            if (fz_group2_check && g_init_args.ecat_device == 2)
            {
                if (saveZeroPoint())
                {
                    // printf("ELMO : zeropoint searching complete, saved \n");
                    de_zp_sequence = false;
                    break;
                }
                else
                {
                    // printf("ELMO : zeropoint searching complete, save failed\n");
                    de_zp_sequence = false;
                    break;
                }
            }

            if (fz_group1_check && g_init_args.ecat_device == 1)
            {
                if (saveZeroPoint())
                {
                    // printf("ELMO : zeropoint searching complete, saved \n");
                    de_zp_sequence = false;
                    break;
                }
                else
                {
                    // printf("ELMO : zeropoint searching complete, save failed\n");
                    de_zp_sequence = false;
                    break;
                }
            }
        }

        //command
        for (int i = 0; i < ec_slavecount; i++)
        {
            if (ElmoMode[START_N + i] == EM_POSITION)
            {
                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
                txPDO[i]->targetPosition = (int)(elmo_axis_direction[START_N + i] * RAD2CNT[START_N + i] * q_desired_elmo_[START_N + i]);
                txPDO[i]->maxTorque = 500;
            }
            else if (ElmoMode[START_N + i] == EM_TORQUE)
            {
                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;

                txPDO[i]->targetTorque = (int)(torque_desired_elmo_[START_N + i] * NM2CNT[START_N + i] * elmo_axis_direction[START_N + i]);
            }
            else if (ElmoMode[START_N + i] == EM_COMMUTATION)
            {
            }
            else
            {
                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                txPDO[i]->targetTorque = (int)0;
            }
        }
#ifdef ECAT_DC
        cur_dc32 = (uint32_t)(ec_DCtime & 0xffffffff);
        if (cur_dc32 > pre_dc32)
            diff_dc32 = cur_dc32 - pre_dc32;
        else
            diff_dc32 = (0xffffffff - pre_dc32) + cur_dc32;
        pre_dc32 = cur_dc32;

        cur_DCtime += diff_dc32;

        ec_sync(cur_DCtime, period_ns, toff);

        if (cur_DCtime > max_DCtime)
            max_DCtime = cur_DCtime;
#endif
    }

    if (!shm_msgs_->shutdown)
        printf("%sELMO %d: Control Mode Start ... at%ld %s\n", cgreen, g_init_args.ecat_device, cycle_count, creset);

    // memset(joint_state_elmo_, ESTATE::OPERATION_READY, sizeof(int) * ec_slavecount);
    // st_start_time = std::chrono::steady_clock::now();
    //cycle_count = 1;
    ////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////
    //Starting
    ///////////////////////////////////////////////////////////////////////////////////////////

    int64_t total1, total2;
    int64_t total_dev1, total_dev2;
    int lat_ns;
    int sat_ns;
    float lmax, lamax, lmin, ldev, lavg, lat;
    float smax, samax, smin, sdev, savg, sat;

    rcv_cnt = 0;

    int l_ovf = 0;
    int s_ovf = 0;

    total1 = 0;
    total2 = 0;
    total_dev1 = 0;
    total_dev2 = 0;

    ldev = 0.0;
    lavg = 0.0;
    lat = 0;

    lmax = 0.0;
    lmin = 10000.00;
    smax = 0.0;
    smin = 100000.0;

    sdev = 0;
    savg = 0;
    sat = 0;

    lamax = 0.0;
    samax = 0.0;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_nsec += PRNS;
    while (ts.tv_nsec >= SEC_IN_NSEC)
    {
        ts.tv_sec++;
        ts.tv_nsec -= SEC_IN_NSEC;
    }
    struct timespec ts1;
    struct timespec ts2;

    uint16_t statusWord[ELMO_DOF];
    uint16_t statusWord_before[ELMO_DOF];
    bool status_first = true;
    control_mode = true;

    if (init_args->ecat_device == 1)
    {
        shm_msgs_->controlModeUpper = true;
    }
    else if (init_args->ecat_device == 2)
    {
        shm_msgs_->controlModeLower = true;
    }

    while (!shm_msgs_->shutdown)
    {
        bool latency_no_count = false;

        clock_gettime(CLOCK_MONOTONIC, &ts1);

        if (getTimeDiff(ts1, ts) < 0)
        {
            latency_no_count = true;
            int ov_cnt = 0;
            while (getTimeDiff(ts1, ts) < 0)
            {
                ts.tv_nsec += PRNS;
                while (ts.tv_nsec >= SEC_IN_NSEC)
                {
                    ts.tv_sec++;
                    ts.tv_nsec -= SEC_IN_NSEC;
                }
                ov_cnt++;
            }
            printf("ov_cnt event at ELMO %d\n", init_args->ecat_device);
        }

        ec_send_processdata();

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

        control_time_real_ = cycle_count * init_args->period_ns / 1000000000.0;

        clock_gettime(CLOCK_MONOTONIC, &ts1);

        lat_ns = (ts1.tv_sec - ts.tv_sec) * SEC_IN_NSEC + ts1.tv_nsec - ts.tv_nsec;

        if (latency_no_count)
            lat_ns = 0;

        /** PDO I/O refresh */

        clock_gettime(CLOCK_MONOTONIC, &ts1);

        pthread_mutex_lock(&rcv_mtx_);

        pthread_cond_signal(&rcv_cond_);

        pthread_mutex_unlock(&rcv_mtx_);

        wkc = ec_receive_processdata(EC_PACKET_TIMEOUT);
        rcv_cnt++;

        __asm volatile("pause" ::
                           : "memory");

        clock_gettime(CLOCK_MONOTONIC, &ts2);
        sat_ns = (ts2.tv_sec - ts1.tv_sec) * SEC_IN_NSEC + ts2.tv_nsec - ts1.tv_nsec;

        for (int i = 0; i < ec_slavecount; i++)
        {
            statusWord[i] = rxPDO[i]->statusWord;
        }

        if (status_first)
        {
            for (int i = 0; i < ec_slavecount; i++)
                statusWord_before[i] = statusWord[i];
            status_first = false;
        }

        static int status_changed_count = -1;
        if (status_log)
        {
            bool stword_changed = false;
            bool stword_slvs[ELMO_DOF] = {
                0,
            };

            for (int i = 0; i < ec_slavecount; i++)
            {
                elmost[i].state = getElmoState(statusWord[i]);

                if (elmost[i].state_before != elmost[i].state)
                {
                    state_elmo_[JointMap2[START_N + i]] = elmost[i].state;
                }
                elmost[i].state_before = elmost[i].state;

                if (statusWord_before[i] != statusWord[i])
                {
                    stword_changed = true;
                    stword_slvs[i] = true;
                }
            }
            // if (stword_changed)
            // {
            //     for (int i=0; i<ec_slavecount; i++)
            //         printStatusword(statusWord[i]);
            //     printf("\n");
            // }
            // if (stword_changed)
            // {
            //     timespec_get(&ts_tt, TIME_UTC + 9);
            //     strftime(buff, sizeof buff, "%T", gmtime(&ts_tt.tv_sec));
            //     printf("ELMO %d | CNT %ld : PDS EVENT %s.%09ld UTC\n", g_init_args.ecat_device, cycle_count, buff, ts_tt.tv_nsec);
            // }

            // if (status_changed)
            // {
            //     status_changed_count++;
            //     printf("status changed time: %lf\n[bits]\n", control_time_real_);
            //     for (int i = 0; i < ec_slavecount; i++)
            //     {
            //         printf("0x%x\t", rxPDO[i]->statusWord);
            //     }
            //     printf("\n");
            // }
        }

        if (wkc >= expectedWKC)
        {
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                // checkFault(rxPDO[slave - 1]->statusWord, slave);
                if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
                {
                    reachedInitial[slave - 1] = true;
                }
                if (reachedInitial[slave - 1])
                {
                    q_elmo_[START_N + slave - 1] = rxPDO[slave - 1]->positionActualValue * CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1] - q_zero_elmo_[START_N + slave - 1];

                    hommingElmo[START_N + slave - 1] =
                        (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)1));
                    q_dot_elmo_[START_N + slave - 1] =
                        (((int32_t)ec_slave[slave].inputs[10]) +
                         ((int32_t)ec_slave[slave].inputs[11] << 8) +
                         ((int32_t)ec_slave[slave].inputs[12] << 16) +
                         ((int32_t)ec_slave[slave].inputs[13] << 24)) *
                        CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1];
                    torque_elmo_[START_N + slave - 1] =
                        (int16_t)(((int16_t)ec_slave[slave].inputs[14]) +
                                  ((int16_t)ec_slave[slave].inputs[15] << 8));
                    q_ext_elmo_[START_N + slave - 1] =
                        (((int32_t)ec_slave[slave].inputs[16]) +
                         ((int32_t)ec_slave[slave].inputs[17] << 8) +
                         ((int32_t)ec_slave[slave].inputs[18] << 16) +
                         ((int32_t)ec_slave[slave].inputs[19] << 24) - q_ext_mod_elmo_[START_N + slave - 1]) *
                        EXTCNT2RAD[START_N + slave - 1] * elmo_ext_axis_direction[START_N + slave - 1];
                    if (START_N + slave == 1 || START_N + slave == 2 || START_N + slave == 7 || START_N + slave == 19 || START_N + slave == 20 || START_N + slave == 16)
                    {
                        hommingElmo[START_N + slave - 1] = !hommingElmo[START_N + slave - 1];
                    }
                    txPDO[slave - 1]->maxTorque = (uint16)1; // originaly 1000
                }
            }
        }

        for (int i = 0; i < ec_slavecount; i++)
        {
            q_[JointMap2[START_N + i]] = q_elmo_[START_N + i];
            q_dot_[JointMap2[START_N + i]] = q_dot_elmo_[START_N + i];
            torque_[JointMap2[START_N + i]] = torque_elmo_[START_N + i];
            q_ext_[JointMap2[START_N + i]] = q_ext_elmo_[START_N + i];
            //joint_state_[JointMap2[START_N + i]] = joint_state_elmo_[START_N + i];

            ElmoMode[START_N + i] = EM_DEFAULT;
        }

        sendJointStatus();
        if (g_init_args.ecat_device != 0)
            getJointCommand();

        // printf("%f %f %f %f %f %f\n",q_elmo_[START_N],q_elmo_[START_N+1],q_elmo_[START_N+2],q_elmo_[START_N+3],q_elmo_[START_N+4],q_elmo_[START_N+5]);
        //memcpy(q_desired_elmo_, &shm_msgs_->positionCommand, sizeof(float) * MODEL_DOF);

        //ECAT JOINT COMMAND
        if (g_init_args.ecat_device == 1)
        {
            if (shm_msgs_->safety_reset_upper_signal)
            {
                memset(ElmoSafteyMode, 0, sizeof(int) * ELMO_DOF);
                shm_msgs_->safety_reset_upper_signal = false;
            }
        }

        if (g_init_args.ecat_device == 2)
        {
            if (shm_msgs_->safety_reset_lower_signal)
            {
                memset(ElmoSafteyMode, 0, sizeof(int) * ELMO_DOF);
                shm_msgs_->safety_reset_lower_signal = false;
            }
        }

        //Joint safety checking ..
        // static int safe_count = 10;

        // if (safe_count-- < 0)
        // {
        if (!shm_msgs_->safety_disable)
            checkJointSafety();
        // }

        //ECAT JOINT COMMAND
        for (int i = 0; i < ec_slavecount; i++)
        {

            if (g_init_args.ecat_device == 0)
            {
                if (pos_hold_switch)
                {
                    torque_desired_elmo_[START_N + i] = (q_desired_elmo_[START_N + i] - q_elmo_[START_N + i]) * pos_p_gain[JointMap2[START_N + i]] - q_dot_elmo_[START_N + i] * pos_d_gain[JointMap2[START_N + i]];
                    maxTorque = 1000;
                    ElmoMode[START_N + i] = EM_TORQUE;
                }
            }
            // txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
            // txPDO[i]->targetTorque = (int)(torque_desired_elmo_[START_N + i] * NM2CNT[START_N + i] * elmo_axis_direction[START_N + i]);
            // txPDO[i]->maxTorque = (uint16)maxTorque;

            if (ElmoMode[START_N + i] == EM_POSITION)
            {
                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
                txPDO[i]->targetPosition = (int)(elmo_axis_direction[START_N + i] * RAD2CNT[START_N + i] * (q_desired_elmo_[START_N + i] + q_zero_elmo_[START_N + i]));
                txPDO[i]->maxTorque = (uint16)maxTorque; // originaly 1000
            }
            else if (ElmoMode[START_N + i] == EM_TORQUE)
            {

                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                txPDO[i]->targetTorque = (int)(torque_desired_elmo_[START_N + i] * NM2CNT[START_N + i] * elmo_axis_direction[START_N + i]);
                txPDO[i]->maxTorque = (uint16)maxTorque; // originaly 1000
            }
            else
            {
                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                txPDO[i]->targetTorque = (int)0;
            }
        }

        if (de_emergency_off)
            emergencyOff();

#ifdef ECAT_DC
        cur_dc32 = (uint32_t)(ec_DCtime & 0xffffffff);
        if (cur_dc32 > pre_dc32)
            diff_dc32 = cur_dc32 - pre_dc32;
        else
            diff_dc32 = (0xffffffff - pre_dc32) + cur_dc32;
        pre_dc32 = cur_dc32;

        cur_DCtime += diff_dc32;

        ec_sync(cur_DCtime, period_ns, toff);

        if (cur_DCtime > max_DCtime)
            max_DCtime = cur_DCtime;

        shm_msgs_->low_toff = toff;
#endif
        for (int i = 0; i < ec_slavecount; i++)
        {
            shm_msgs_->elmo_torque[JointMap2[START_N + i]] = txPDO[i]->targetTorque;
        }
        ts.tv_nsec += PRNS + toff;
        if (ts.tv_nsec >= SEC_IN_NSEC)
        {
            ts.tv_sec++;
            ts.tv_nsec -= SEC_IN_NSEC;
        }

        total1 += lat_ns;

        total2 += sat_ns;

        if (lat_ns > PERIOD_OVF * 1000)
        {
            l_ovf++;
        }
        if (sat_ns > 350 * 1000)
        {
            s_ovf++;
        }

        static int c_count = 0;
        c_count++;

        lat_avg = total1 / c_count;
        send_avg = total2 / c_count;

        if (lmax < lat_ns)
        {
            lmax = lat_ns;
        }
        if (smax < sat_ns)
        {
            smax = sat_ns;
        }
        if (lamax < lat_ns)
        {
            lamax = lat_ns;
        }
        if (samax < sat_ns)
        {
            samax = sat_ns;
        }
        total_dev2 += sqrt(((sat - savg) * (sat - savg)));
        sdev = total_dev2 / cycle_count;

        // shm_msgs_->send_avg2 = savg;
        // shm_msgs_->send_max2 = smax;
        // shm_msgs_->send_min2 = smin;
        // shm_msgs_->send_dev2 = sdev;

        if (g_init_args.ecat_device == 0)
        {
            if (cycle_count % 2000 == 0)
            {
                //printf("%d : %ld L avg : %5.2f max : %5.2f amax : %5.2f C avg : %5.2f max : %5.2f amax : %5.2f ovf : %d\n ", g_init_args.ecat_device, cycle_count / 2000, lat_avg / 1000.0, lmax / 1000.0, lamax / 1000.0, send_avg / 1000.0, smax / 1000.0, samax / 1000.0, s_ovf);
                //printf("  rcv max : %7.3f ovf : %d mid max : %7.3f ovf : %d snd max : %7.3f ovf : %d statwrd chg cnt : %d\n", low_rcv_max / 1000.0, low_rcv_ovf, low_mid_max / 1000.0, low_mid_ovf, low_snd_max / 1000.0, low_snd_ovf, status_changed_count);

                shm_msgs_->lat_avg = lat_avg;
                shm_msgs_->lat_max = lmax;
                shm_msgs_->lat_max2 = lamax;
                shm_msgs_->lat_ovf = l_ovf;

                shm_msgs_->send_avg = send_avg;
                shm_msgs_->send_max = smax;
                shm_msgs_->send_max2 = samax;
                shm_msgs_->send_ovf = s_ovf;

                c_count = 0;
                total1 = 0;
                total2 = 0;
                lmax = 0;
                smax = 0;
            }
        }
        else if (g_init_args.ecat_device == 1)
        {
            shm_msgs_->lat_avg = lat_avg;
            shm_msgs_->lat_max = lamax;
            shm_msgs_->lat_ovf = l_ovf;
            shm_msgs_->send_avg = send_avg;
            shm_msgs_->send_max = samax;
            shm_msgs_->send_ovf = s_ovf;
        }
        else if (g_init_args.ecat_device == 2)
        {
            shm_msgs_->lat_avg2 = lat_avg;
            shm_msgs_->lat_max2 = lamax;
            shm_msgs_->lat_ovf2 = l_ovf;
            shm_msgs_->send_avg2 = send_avg;
            shm_msgs_->send_max2 = samax;
            shm_msgs_->send_ovf2 = s_ovf;
        }

        cycle_count++;
    }

    inOP = FALSE;

    printf("\nELMO : Request init state for all slaves\n");
    /** request INIT state for all slaves
            *  slave number = 0 -> write to all slaves
            */
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);

    printf("ELMO : Checking EC STATE ... \n");
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    printf("ELMO : Checking EC STATE Complete \n");

    return (void *)NULL;
}

void *ethercatThread2(void *data)
{
    TocabiInitArgs *init_args = (TocabiInitArgs *)data;

    struct timespec ts_thread2;

    clock_gettime(CLOCK_MONOTONIC, &ts_thread2);

    int thread2_cnt = 0;

    int rcv_cnt_t2 = 0;

    while (!shm_msgs_->shutdown)
    {
        ts_thread2.tv_nsec += 1000000;

        if (ts_thread2.tv_nsec > SEC_IN_NSEC)
        {
            ts_thread2.tv_nsec -= SEC_IN_NSEC;
            ts_thread2.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts_thread2, NULL);

        if (control_mode)
        {
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                checkFault(rxPDO[slave - 1]->statusWord, slave);
            }
        }

        thread2_cnt++;

        // if (control_mode)
        // {
        //     if (rcv_cnt == rcv_cnt_t2)
        //     {

        //         // printf("ELMO %d : rcv not increased \n", g_init_args.ecat_device);

        //         ec_send_processdata();
        //     }

        //     rcv_cnt_t2 = rcv_cnt;
        // }

        if (thread2_cnt % 4 == 0)
        {
            ethercatCheck(init_args);
        }

        if (thread2_cnt % 1000 == 0)
        {
            if (diag_switch)
                ecatDiagnose();
            else
            {
                ecatDiagnoseOnChange();
            }
        }
    }

    // std::printf("ELMO : EthercatThread2 Shutdown\n");
    return (void *)NULL;
}

void *ethercatThread3(void *data)
{
    TocabiInitArgs *init_args = (TocabiInitArgs *)data;

    struct timespec ts_thread3, ts_timeout, ts_timeout2, ts_timeout3;

    ts_timeout.tv_nsec = 350000;
    ts_timeout.tv_sec = 0;

    ts_timeout2.tv_nsec = 500000;
    ts_timeout2.tv_sec = 0;

    ts_timeout3.tv_nsec = 150000;
    ts_timeout3.tv_sec = 0;

    clock_gettime(CLOCK_MONOTONIC, &ts_thread3);

    int thread3 = 0;

    int rcv_cnt_thread3 = 0;

    while (!shm_msgs_->shutdown)
    {
        pthread_mutex_lock(&rcv_mtx_);

        pthread_cond_timedwait(&rcv_cond_, &rcv_mtx_, &ts_timeout3);

        pthread_mutex_unlock(&rcv_mtx_);

        clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_timeout2, NULL);

        // if (thread3 % 4000 == 0)
        // {
        //     printf("ECAT %d : Hello from Thread3 \n", init_args->ecat_device);
        // }

        __asm volatile("pause" ::
                           : "memory");

        if (rcv_cnt > 10)
        {
            if (control_mode && (rcv_cnt_thread3 == rcv_cnt))
            {
                printf("ELMO %d : SAME RCV CNT DETECTED AT %d\n", init_args->ecat_device, rcv_cnt);
                ec_send_processdata();
                clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_timeout2, NULL);

                while (rcv_cnt_thread3 == rcv_cnt)
                {
                    printf("ELMO %d : SENDING FOR RECOVERY ...  %d\n", init_args->ecat_device, rcv_cnt);

                    ec_send_processdata();
                    clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_timeout2, NULL);
                }
                printf("ELMO %d : RCV CNT UPDATE CHECK %d\n", init_args->ecat_device, rcv_cnt);
            }
        }

        // if (control_mode) //Watching Other ECAT systems..
        // {
        //     if (g_init_args.ecat_device == 1)
        //     {
        //         if (shm_msgs_->controlModeLower)
        //         {
        //             static int monitor_other_devie = shm_msgs_->statusCount2 - shm_msgs_->statusCount;

        //             printf("ECAT %d : different status Count on Other ECAT  \n", g_init_args.ecat_device);
        //         }
        //     }
        //     else if (g_init_args.ecat_device == 2)
        //     {
        //         if (shm_msgs_->controlModeUpper)
        //         {
        //             static int monitor_other_devie = shm_msgs_->statusCount - shm_msgs_->statusCount2;

        //             if (monitor_other_devie != (shm_msgs_->statusCount - shm_msgs_->statusCount2))
        //             {
        //                 monitor_other_devie = shm_msgs_->statusCount - shm_msgs_->statusCount2;
        //             }

        //             printf("ECAT %d : different status Count on Other ECAT \n", g_init_args.ecat_device);
        //         }
        //     }
        // }

        rcv_cnt_thread3 = rcv_cnt;

        // if(init_args->ecat_device == )

        if (init_args->ecat_device == 0)
        {
            int ch = kbhit();
            if (ch != -1)
            {

                // printf("input : %d ", ch);
                //std::cout << "key input : " << (char)(ch % 256) << std::endl;
                if ((ch % 256 == 'q'))
                {
                    printf("ELMO : shutdown request\n");
                    shm_msgs_->shutdown = true;
                }
                else if ((ch % 256 == 'l'))
                {
                    printf("ELMO : start searching zero point lower\n");
                    de_zp_lower_switch = true;
                }
                else if ((ch % 256 == 'u'))
                {
                    printf("ELMO : start searching zero point upper\n");
                    de_zp_upper_switch = true;
                }
                else if ((ch % 256 == 's'))
                {
                    // printf("------------------------------------------------------\n");
                    // printf("%f\n", control_time_real_);
                    // for (int i = 0; i < ec_slavecount; i++)
                    // {
                    //     printf("%4d   %20s  %16d\n", i, ELMO_NAME[START_N + i], std::bitset<16>(rxPDO[i]->statusWord));
                    // }diag_switc
                    diag_switch = !diag_switch;
                }
                else if ((ch % 256 == 'd'))
                {
                    de_debug_level++;
                    if (de_debug_level > 2)
                        de_debug_level = 0;
                    printf("ELMO : debug mode, level %d\n", de_debug_level);
                }
                else if ((ch % 256 == 'p'))
                {
                    printf("------------------------------------------------------\n");
                    printf("%f\n", control_time_real_);
                    for (int i = 0; i < ec_slavecount; i++)
                    { //std::cout << i << ELMO_NAME[i] <<
                        printf("%4d   %20s  %12f  ext : %12f\n", i, ELMO_NAME[START_N + i], q_elmo_[START_N + i], q_ext_elmo_[START_N + i]);
                    }
                }
                else if ((ch % 256 == 't'))
                {
                    printf("torque actual\n");
                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        printf("%4d   %20s  %12f \n", i, ELMO_NAME[START_N + i], (double)torque_elmo_[i]);
                    }
                }
                else if ((ch % 256 == 'h'))
                {
                    printf("------------------------------------------------------\n");
                    printf("%f\n", control_time_real_);
                    for (int i = 0; i < ec_slavecount; i++)
                        printf("%4d | %20s \t %d %d %d %d \n", i, ELMO_NAME[START_N + i], (uint32_t)ec_slave[i + 1].inputs[4], (uint32_t)ec_slave[i + 1].inputs[5], (uint32_t)ec_slave[i + 1].inputs[6], (uint32_t)ec_slave[i + 1].inputs[7]);
                }
                else if ((ch % 256 == 'c'))
                {
                    printf("ELMO : Force control mode\n");
                    force_control_mode = true;
                }
                else if ((ch % 256 == 'o'))
                {
                    pos_hold_switch = !pos_hold_switch;

                    if (pos_hold_switch)
                    {
                        printf("ELMO : Lock current position %d \n", START_N);

                        for (int i = 0; i < ec_slavecount; i++)
                        {
                            q_desired_elmo_[START_N + i] = q_elmo_[START_N + i];
                        }
                    }
                    else
                    {

                        printf("ELMO : Disable Hold mode\n");
                    }
                }
                else if ((ch % 256 == 'f'))
                {
                    printf("torque off\n");
                    pos_hold_switch = false;
                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        //q_desired_elmo_[i] = q_elmo_[i];
                        ElmoMode[START_N + i] = EM_DEFAULT;
                    }
                }
                else if ((ch % 256 == 'w'))
                {

                    status_log = !status_log;

                    if (status_log)
                    {
                        printf("log status start\n");
                    }
                    else
                    {
                        printf("log status end\n");
                    }
                }
                else if ((ch % 256 == 'k'))
                {
                    ec_send_processdata();
                }
            }
        }

        thread3++;
    }

    return (void *)NULL;
}

double elmoJointMove(double current_time, double init, double angle, double start_time, double traj_time)
{
    double des_pos;

    if (current_time < start_time)
    {
        des_pos = init;
    }
    else if ((current_time >= start_time) && (current_time < (start_time + traj_time)))
    {
        des_pos = init + angle * (current_time - start_time) / traj_time;
    }
    else if (current_time >= (start_time + traj_time))
    {
        des_pos = init + angle;
    }

    return des_pos;
}

bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord)
{
    if (!(statusWord & (1 << OPERATION_ENABLE_BIT)))
    {
        if (!(statusWord & (1 << SWITCHED_ON_BIT)))
        {
            if (!(statusWord & (1 << READY_TO_SWITCH_ON_BIT)))
            {
                if (statusWord & (1 << FAULT_BIT))
                {
                    controlWord = 0x80;
                    return false;
                }
                else
                {
                    controlWord = CW_SHUTDOWN;
                    return false;
                }
            }
            else
            {
                controlWord = CW_SWITCHON;
                return true;
            }
        }
        else
        {
            controlWord = CW_ENABLEOP;
            return true;
        }
    }
    else
    {
        controlWord = CW_ENABLEOP;
        return true;
    }
    controlWord = 0;
    return false;
}
void checkJointSafety()
{
    for (int i = 0; i < g_init_args.ecat_slave_num; i++)
    {

        if (ElmoSafteyMode[START_N + i] == 0)
        {
            state_safety_[JointMap2[START_N + i]] = SSTATE::SAFETY_OK;

            if ((joint_lower_limit[START_N + i] > q_elmo_[START_N + i]))
            {
                printf("%sELMO %d %s : SAFETY LOCK - JOINT LIMIT : %f LIMIT : %f %s\n", cred, g_init_args.ecat_device, ELMO_NAME[START_N + i], q_elmo_[START_N + i], joint_lower_limit[START_N + i], creset);
                state_safety_[JointMap2[START_N + i]] = SSTATE::SAFETY_JOINT_LIMIT;
                ElmoSafteyMode[START_N + i] = 1;
            }
            else if ((joint_upper_limit[START_N + i] < q_elmo_[START_N + i]))
            {
                printf("%sELMO %d %s : SAFETY LOCK - JOINT LIMIT : %f LIMIT : %f %s\n", cred, g_init_args.ecat_device, ELMO_NAME[START_N + i], q_elmo_[START_N + i], joint_upper_limit[START_N + i], creset);

                state_safety_[JointMap2[START_N + i]] = SSTATE::SAFETY_JOINT_LIMIT;
                ElmoSafteyMode[START_N + i] = 1;
            }

            if (joint_velocity_limit[START_N + i] < abs(q_dot_elmo_[START_N + i]))
            {
                printf("%sELMO %d %s : SAFETY LOCK - VELOCITY LIMIT %s\n", cred, g_init_args.ecat_device, ELMO_NAME[START_N + i], creset);
                state_safety_[JointMap2[START_N + i]] = SSTATE::SAFETY_VELOCITY_LIMIT;
                ElmoSafteyMode[START_N + i] = 1;
            }
        }

        if (ElmoSafteyMode[START_N + i] == 1)
        {
            q_desired_elmo_[START_N + i] = q_elmo_[START_N + i];
            ElmoMode[START_N + i] = EM_POSITION;
            ElmoSafteyMode[START_N + i] = 2;
        }

        if (ElmoSafteyMode[START_N + i] == 2)
        {
            ElmoMode[START_N + i] = EM_POSITION;
        }
    }
}

void checkJointStatus()
{
}

// void initSharedMemory()
// {

//     if ((shm_id_ = shmget(shm_msg_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
//     {
//         std::printf("shm mtx failed \n");
//         exit(0);
//     }

//     if ((shm_msgs_ = (SHMmsgs *)shmat(shm_id_, NULL, 0)) == (SHMmsgs *)-1)
//     {
//         std::printf("shmat failed \n");
//         exit(0);
//     }
//     /*
//     if (pthread_mutexattr_init(&shm_msgs_->mutexAttr) == 0)
//     {
//         std::printf("shared mutex attr init\n");
//     }

//     if (pthread_mutexattr_setpshared(&shm_msgs_->mutexAttr, PTHREAD_PROCESS_SHARED) == 0)
//     {
//         std::printf("shared mutexattr set\n");
//     }

//     if (pthread_mutex_init(&shm_msgs_->mutex, &shm_msgs_->mutexAttr) == 0)
//     {
//         std::printf("shared mutex init\n");
//     }*/

//     if (shmctl(shm_id_, SHM_LOCK, NULL) == 0)
//     {
//         //std::printf("SHM_LOCK enabled\n");
//     }
//     else
//     {
//         std::printf("SHM lock failed\n");
//     }

//     shm_msgs_->t_cnt = 0;
//     shm_msgs_->controllerReady = false;
//     shm_msgs_->statusWriting = 0;
//     shm_msgs_->commanding = false;
//     shm_msgs_->reading = false;
//     shm_msgs_->shutdown = false;

//     //
//     //float lat_avg, lat_min, lat_max, lat_dev;
//     //float send_avg, send_min, send_max, send_dev;

//     shm_msgs_->lat_avg = 0;
//     shm_msgs_->lat_min = 0;
//     shm_msgs_->lat_max = 100000;
//     shm_msgs_->lat_dev = 0;

//     shm_msgs_->send_avg = 0;
//     shm_msgs_->send_min = 0;
//     shm_msgs_->send_max = 100000;
//     shm_msgs_->send_dev = 0;
// }
void sendJointStatus()
{

    shm_msgs_->statusWriting++;

    memcpy(&shm_msgs_->pos[Q_START], &q_[Q_START], sizeof(float) * PART_ELMO_DOF);
    memcpy(&shm_msgs_->posExt[Q_START], &q_ext_[Q_START], sizeof(float) * PART_ELMO_DOF);
    memcpy(&shm_msgs_->vel[Q_START], &q_dot_[Q_START], sizeof(float) * PART_ELMO_DOF);
    memcpy(&shm_msgs_->torqueActual[Q_START], &torque_[Q_START], sizeof(float) * PART_ELMO_DOF);

    memcpy(&shm_msgs_->safety_status[Q_START], &state_safety_[Q_START], sizeof(int8_t) * PART_ELMO_DOF);
    memcpy(&shm_msgs_->zp_status[Q_START], &state_zp_[Q_START], sizeof(int8_t) * PART_ELMO_DOF);
    memcpy(&shm_msgs_->ecat_status[Q_START], &state_elmo_[Q_START], sizeof(int8_t) * PART_ELMO_DOF);

    shm_msgs_->statusWriting--;

    if (g_init_args.ecat_device == 1)
    {
        shm_msgs_->statusCount = cycle_count; //,memory_order_release);
    }
    // // TODO: 2
    if (g_init_args.ecat_device == 2)
        shm_msgs_->statusCount2 = cycle_count; //,memory_order_release);

    //shm_msgs_->statusCount = cycle_count;

    if (g_init_args.is_main)
    {
        shm_msgs_->triggerS1 = true;

        cpu_relax();
        // atomic_store(&shm_->triggerS1,1,memory_order_release);
    }
}

void getJointCommand()
{
    // while (shm_msgs_->commanding.load(std::memory_order_acquire))
    // {
    //     clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_us1, NULL);
    // }

    cpu_relax();

    static int stloop;
    static bool stloop_check;
    stloop_check = false;
    if (stloop == shm_msgs_->stloopCount)
    {
        stloop_check = true;
    }
    stloop = shm_msgs_->stloopCount;
    static int commandCount;
    int wait_tick;

    // if (!stloop_check)
    // {
    //     while (shm_msgs_->commandCount == commandCount)
    //     {
    //         clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_us1, NULL);
    //         if (++wait_tick > 3)
    //         {
    //             break;
    //         }
    //     }
    // }

    // memcpy(&command_mode_[Q_START], &shm_msgs_->commandMode[Q_START], sizeof(int) * PART_ELMO_DOF);
    // memcpy(&q_desired_[Q_START], &shm_msgs_->positionCommand[Q_START], sizeof(float) * PART_ELMO_DOF);

    if (g_init_args.ecat_device == 1)
    {
        while (shm_msgs_->cmd_upper)
        {
        };
        shm_msgs_->cmd_upper = true;
        memcpy(&torque_desired_[Q_START], &shm_msgs_->torqueCommand[Q_START], sizeof(float) * PART_ELMO_DOF);
        shm_msgs_->cmd_upper = false;
    }

    if (g_init_args.ecat_device == 2)
    {
        while (shm_msgs_->cmd_lower)
        {
        };
        shm_msgs_->cmd_lower = true;
        memcpy(&torque_desired_[Q_START], &shm_msgs_->torqueCommand[Q_START], sizeof(float) * PART_ELMO_DOF);
        shm_msgs_->cmd_lower = false;
    }

    commandCount = shm_msgs_->commandCount;

    for (int i = 0; i < ec_slavecount; i++)
    {
        // command_mode_elmo_[JointMap[Q_START + i]] = command_mode_[Q_START + i];

        // if (command_mode_[Q_START + i] == 1)
        // {
        torque_desired_elmo_[JointMap[Q_START + i]] = torque_desired_[Q_START + i];

        ElmoMode[START_N + i] = EM_TORQUE;
        // }
        // else if (command_mode_[Q_START + i] == 2)
        // {
        //     q_desired_elmo_[JointMap[Q_START + i]] = q_desired_[Q_START + i];
        // }
    }

    static int commandCount_before = -1;
    static int commandCount_before2 = -1;
    static int errorCount = -2;
    static int errorTimes = 0;

    static bool start_observe = false;

    if (!start_observe)
    {
        if (shm_msgs_->controlModeUpper && shm_msgs_->controlModeUpper)
        {
            start_observe = true;
        }
    }

    if (start_observe)
    {
        if ((shm_msgs_->controlModeUpper && g_init_args.ecat_device == 1) ||
            (shm_msgs_->controlModeLower && g_init_args.ecat_device == 2))
        {

            if (errorTimes == 0)
            {
                if (commandCount <= commandCount_before) //shit
                {
                    if (stloop_check)
                    {
                        errorTimes++;
                        if (errorTimes > 2)
                            printf("%ld ELMO %d : commandCount Error wit sterror current : %d before : %d \n", (long)control_time_real_ / 1000, g_init_args.ecat_device, commandCount, commandCount_before);
                        // std::cout << control_time_us_ << "ELMO_LOW : commandCount Error current : " << commandCount << " before : " << commandCount_before << std::'\n';
                        // std::cout << "stloop same cnt" << std::'\n';
                    }
                    else
                    {
                        // printf("%ld ELMO %d : commandCount Error without st current : %d before : %d \n", (long)control_time_real_ / 1000, g_init_args.ecat_device, commandCount, commandCount_before);
                    }
                }
            }
            else if (errorTimes > 0)
            {
                if (commandCount > commandCount_before) // no problem
                {
                    errorTimes = 0;
                    errorCount = 0;
                }
                else //shit
                {
                    errorTimes++;

                    if (errorTimes > CL_LOCK)
                    {
                        if (errorCount != commandCount)
                        {
                            printf("%s %f ELMO %d : commandCount Warn! SAFETY LOCK%s\n", cred, control_time_real_, g_init_args.ecat_device, creset);

                            // std::cout << cred << control_time_us_ << "ELMO_LOW : commandCount Warn! SAFETY LOCK" << creset << std::'\n';

                            // std::fill(ElmoSafteyMode, ElmoSafteyMode + MODEL_DOF, 1);

                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                ElmoSafteyMode[START_N + i] = 1;
                                state_safety_[JointMap2[START_N + i]] = SSTATE::SAFETY_COMMAND_LOCK;
                            }
                            errorCount = commandCount;
                        }
                        else
                        {
                            //std::cout << errorTimes << "ELMO_LOW : commandCount error duplicated" << std::'\n';
                        }
                    }
                }
            }
        }
    }
    else
    {
        shm_msgs_->maxTorque = 0;
    }

    commandCount_before2 = commandCount_before;
    commandCount_before = commandCount;

    maxTorque = shm_msgs_->maxTorque;
}

bool saveCommutationLog()
{
    FILE *fp;
    fp = fopen(g_init_args.commutation_cache_file, "wb");
    if (fp == NULL)
    {
        printf("ELMO %d : Failed to Open Commutation Log File\n", g_init_args.ecat_device);
        return false;
    }
    else
    {
        printf("ELMO %d : COMMUTATION SAVED!\n", g_init_args.ecat_device);
    }

    struct timespec cm_time;
    clock_gettime(CLOCK_REALTIME, &cm_time);

    fwrite(&cm_time, sizeof(timespec), 1, fp);

    fclose(fp);

    return true;
}

bool loadCommutationLog(struct timespec &commutation_time)
{
    FILE *fp;
    fp = fopen(g_init_args.commutation_cache_file, "rb");
    if (fp == NULL)
    {
        printf("ELMO %d : Failed to Open Commutation Log file\n", g_init_args.ecat_device);
        return false;
    }

    struct timespec ts_now;
    clock_gettime(CLOCK_REALTIME, &ts_now);

    int fr = fread(&commutation_time, sizeof(timespec), 1, fp);

    fclose(fp);

    float t_before = (float)(getTimeDiff(commutation_time, ts_now) / SEC_IN_NSEC);

    printf("ELMO %d : Commutation done %d seconds before \n", g_init_args.ecat_device, (int)t_before);

    return true;
}

bool saveZeroPoint()
{
    FILE *fp;
    fp = fopen(g_init_args.zeropoint_cache_file, "wb");
    if (fp == NULL)
    {
        printf("ELMO %d : Failed to Open zeropoint Log file\n", g_init_args.ecat_device);
        return false;
    }
    struct timespec t_now;
    clock_gettime(CLOCK_REALTIME, &t_now);

    fwrite(&t_now, sizeof(timespec), 1, fp);
    fwrite(q_zero_elmo_, sizeof(double), ELMO_DOF, fp);

    fclose(fp);
    return true;
}

bool loadZeroPoint(bool force)
{
    FILE *fp;
    fp = fopen(g_init_args.zeropoint_cache_file, "rb");
    if (fp == NULL)
    {
        printf("ELMO %d : Failed to Open zeropoint Log file\n", g_init_args.ecat_device);
        return false;
    }

    struct timespec ts_now_;
    clock_gettime(CLOCK_REALTIME, &ts_now_);

    struct timespec ts_zp_saved_;
    int fr = fread(&ts_zp_saved_, sizeof(timespec), 1, fp);

    double getzp[ELMO_DOF];
    fr = fread(getzp, sizeof(double), ELMO_DOF, fp);

    fclose(fp);

    if (!force)
    {

        struct timespec commutation_save_time_;

        loadCommutationLog(commutation_save_time_);

        if (getTimeDiff(ts_zp_saved_, commutation_save_time_) > 0)
        {
            printf("ELMO %d : commutation saved time is before than zp saved time\n", g_init_args.ecat_device);
            return false;
        }
    }

    for (int i = 0; i < g_init_args.ecat_slave_num; i++)
    {
        state_zp_[JointMap2[START_N + i]] = ZSTATE::ZP_SUCCESS;
        q_zero_elmo_[START_N + i] = getzp[START_N + i];
    }

    return true;
}

void emergencyOff() //TorqueZero
{
    for (int i = 0; i < ec_slavecount; i++)
    {
        txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
        txPDO[i]->targetTorque = (int)0;
    }
}

int kbhit(void)
{
    struct termios oldt, newt;
    int ch = 0;
    int oldf = 0;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO | ISIG);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    int nread = read(0, &ch, 1);
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (nread >= 1)
    {
        return ch;
    }
    else
    {
        return -1;
    }
}

int getElmoState(uint16_t state_bit)
{
    if (!(state_bit & (1 << OPERATION_ENABLE_BIT)))
    {
        if (!(state_bit & (1 << SWITCHED_ON_BIT)))
        {
            if (!(state_bit & (1 << READY_TO_SWITCH_ON_BIT)))
            {
                if (state_bit & (1 << FAULT_BIT))
                {
                    return ELMO_FAULT;
                }
                else
                {
                    return ELMO_NOTFAULT;
                }
            }
            else
            {
                return ELMO_READY_TO_SWITCH_ON;
            }
        }
        else
        {
            return ELMO_SWITCHED_ON;
        }
    }
    else
    {
        return ELMO_OPERATION_ENABLE;
    }
}

void findzeroLeg()
{
    for (int i = 0; i < 6; i++)
    {
        q_zero_elmo_[i + R_HipYaw_Joint] = q_elmo_[i + R_HipYaw_Joint] - q_ext_elmo_[i + R_HipYaw_Joint];
        //pub_to_gui(dc, "jointzp %d %d", i + R_HipYaw_Joint, 1);
        q_zero_elmo_[i + L_HipYaw_Joint] = q_elmo_[i + L_HipYaw_Joint] - q_ext_elmo_[i + L_HipYaw_Joint];
        //pub_to_gui(dc, "jointzp %d %d", i + TOCABI::L_HipYaw_Joint, 1);
        //std::cout << ELMO_NAME[i + R_HipRoll_Joint] << " pz IE P : " << q_elmo_[i + R_HipRoll_Joint] << " pz EE P : " << q_ext_elmo_[i + R_HipRoll_Joint] << '\n';
        //std::cout << ELMO_NAME[i + L_HipRoll_Joint] << " pz ELMO : " << q_elmo_[i + L_HipRoll_Joint] << " pz ELMO : " << q_ext_elmo_[i + L_HipRoll_Joint] << '\n';
    }
}
void findZeroPointlow(int slv_number, double time_real_)
{
    double velocity = 0.05;
    double fztime = 1.0;
    if (elmofz[slv_number].findZeroSequence == FZ_CHECKHOMMINGSTATUS)
    {

        state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_SEARCHING_ZP;
        elmofz[slv_number].initTime = time_real_;
        elmofz[slv_number].initPos = q_elmo_[slv_number];
        elmofz[slv_number].desPos = q_ext_elmo_[slv_number];
        elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGSTART;

        if (q_ext_elmo_[slv_number] > 0)
        {
            elmofz[slv_number].init_direction = -1;
        }
        else
        {
            elmofz[slv_number].init_direction = 1;
        }

        if ((q_ext_elmo_[slv_number] > 3.14) || (q_ext_elmo_[slv_number < -3.14]))
        {
            printf("%sELMO %d : REBOOTING REQUIRED! joint %d : ext encoder error%s\n", cred, g_init_args.ecat_device, slv_number, creset);
            // std::cout << cred << "elmo reboot required. joint " << slv_number << "external encoder error" << q_ext_elmo_[slv_number] << '\n';
        }
    }

    if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGSTART)
    {

        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(time_real_, elmofz[slv_number].initPos, -elmofz[slv_number].desPos, elmofz[slv_number].initTime, fztime);

        if (time_real_ == elmofz[slv_number].initTime)
        {
            //std::printf("joint " << slv_number << "  init pos : " << elmofz[slv_number].initPos << "   goto " << elmofz[slv_number].initPos + elmofz[slv_number].init_direction * 0.6 << '\n';
        }
        static int sucnum = 0;

        if (time_real_ > (elmofz[slv_number].initTime + fztime + 2.0))
        {
            if (q_ext_elmo_[slv_number] == 0.0)
            {
                elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGEND;
                elmofz[slv_number].result = ElmoHommingStatus::SUCCESS;
                sucnum++;
                printf("ELMO %d : %d success \n", g_init_args.ecat_device, slv_number);
                // std::cout << slv_number << "success : " << sucnum << '\n';
                state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_SUCCESS;
            }
            else
            {
                elmofz[slv_number].initTime = time_real_;
                elmofz[slv_number].initPos = q_elmo_[slv_number];
                elmofz[slv_number].desPos = q_ext_elmo_[slv_number];
                elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGSTART;
            }
        }
        // if (control_time_real_ > elmofz[slv_number].initTime + fztime * 4.0)
        // {
        //     elmofz[slv_number].result == ElmoHommingStatus::FAILURE;
        // }
    }
}
void findZeroPoint(int slv_number, double time_now_)
{
    double fztime = 3.0;
    double fztime_manual = 300.0;
    if (elmofz[slv_number].findZeroSequence == FZ_CHECKHOMMINGSTATUS)
    {
        state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_SEARCHING_ZP;
        //pub_to_gui(dc, "jointzp %d %d", slv_number, 0);
        if (hommingElmo[slv_number])
        {
            // printf("init homming on : %d %7.3f\n", slv_number, time_now_);
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGSTART;
            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
            elmofz[slv_number].firstPos = q_elmo_[slv_number];
        }
        else
        {

            // printf("init homming off : %d %7.3f\n", slv_number, time_now_);
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMING;
            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
            elmofz[slv_number].firstPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGSTART)
    {
        //go to + 0.3rad until homming sensor turn off
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(time_now_, elmofz[slv_number].initPos, 0.3, elmofz[slv_number].initTime, fztime);

        if ((hommingElmo[slv_number] == 0) && (hommingElmo_before[slv_number] == 0))
        {
            // printf("goto homming off : %d %7.3f\n", slv_number, time_now_);
            //std::printf("motor " << slv_number << " seq 1 complete, wait 1 sec\n");
            hommingElmo_before[slv_number] = hommingElmo[slv_number];
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGEND;
            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].posStart = q_elmo_[slv_number];
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }

        if (time_now_ > elmofz[slv_number].initTime + fztime)
        {
            printf("%s ELMO %d : WARNING! : %s homming not turning off! %s\n", cred, g_init_args.ecat_device, ELMO_NAME[slv_number], creset);
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGEND)
    {
        // printf("%f goto homming on : %d\n", time_now_, slv_number);
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(time_now_, elmofz[slv_number].posStart, -0.3, elmofz[slv_number].initTime, fztime);

        //go to -20deg until homming turn on, and turn off
        if ((hommingElmo_before[slv_number] == 1) && (hommingElmo[slv_number] == 0))
        {
            // printf("goto homming on : %d\n", slv_number);
            if (abs(elmofz[slv_number].posStart - q_elmo_[slv_number]) > elmofz[slv_number].req_length)
            {
                elmofz[slv_number].posEnd = q_elmo_[slv_number];
                elmofz[slv_number].endFound = 1;
            }
            else
            {
                printf("ELMO %d : Joint %d %s : Not enough distance, required : %f current : %f \n", g_init_args.ecat_device, slv_number, ELMO_NAME[slv_number], elmofz[slv_number].req_length, abs(elmofz[slv_number].posStart - q_elmo_[slv_number]));
                // std::printf("Joint " << slv_number << " " << ELMO_NAME[slv_number] << " : Not enough distance, required : " << elmofz[slv_number].req_length << ", detected : " << abs(elmofz[slv_number].posStart - q_elmo_[slv_number]) << '\n';

                // std::printf("Joint " << slv_number << " " << ELMO_NAME[slv_number] << "if you want to proceed with detected length, proceed with manual mode \n");

                state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_NOT_ENOUGH_HOMMING;
                elmofz[slv_number].findZeroSequence = 7;
                elmofz[slv_number].result = ElmoHommingStatus::FAILURE;
                elmofz[slv_number].initTime = time_now_;
            }
        }
        else if ((hommingElmo_before[slv_number] == 0) && (hommingElmo[slv_number] == 0))
        {
            if (elmofz[slv_number].endFound == 1)
            {
                // printf("goto zero : %d\n", slv_number);
                elmofz[slv_number].findZeroSequence = FZ_GOTOZEROPOINT;
                state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_GOTO_ZERO;

                elmofz[slv_number].initPos = q_elmo_[slv_number];
                q_zero_elmo_[slv_number] = (elmofz[slv_number].posEnd + elmofz[slv_number].posStart) * 0.5 + q_zero_mod_elmo_[slv_number];
                elmofz[slv_number].initTime = time_now_;
            }
        }

        if (time_now_ > elmofz[slv_number].initTime + fztime)
        {
            // printf("goto seg 6 : %d\n", slv_number);
            //If dection timeout, go to failure sequence
            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].findZeroSequence = 6;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMING)
    { //start from unknown

        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(time_now_, elmofz[slv_number].initPos, elmofz[slv_number].init_direction * 0.3, elmofz[slv_number].initTime, fztime);
        if (time_now_ > (elmofz[slv_number].initTime + fztime))
        {
            // printf("fzhm: %d\n", slv_number);
            q_desired_elmo_[slv_number] = elmoJointMove(time_now_, elmofz[slv_number].initPos + 0.3 * elmofz[slv_number].init_direction, -0.6 * elmofz[slv_number].init_direction, elmofz[slv_number].initTime + fztime, fztime * 2.0);
        }

        if (hommingElmo[slv_number] && hommingElmo_before[slv_number])
        {
            // printf("set to seq 1 : %d\n", slv_number);
            elmofz[slv_number].findZeroSequence = 1;
            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }

        if (time_now_ > (elmofz[slv_number].initTime + fztime * 3.0))
        {
            // printf("set to seq 6 : %d\n", slv_number);
            //If dection timeout, go to failure sequence
            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].findZeroSequence = 6;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_GOTOZEROPOINT)
    {
        ElmoMode[slv_number] = EM_POSITION;

        double go_to_zero_dur = fztime * (abs(q_zero_elmo_[slv_number] - elmofz[slv_number].initPos) / 0.3);
        q_desired_elmo_[slv_number] = elmoJointMove(time_now_, elmofz[slv_number].initPos, q_zero_elmo_[slv_number] - elmofz[slv_number].initPos, elmofz[slv_number].initTime, go_to_zero_dur);

        //go to zero position
        if (time_now_ > (elmofz[slv_number].initTime + go_to_zero_dur))
        {
            //std::printf("go to zero complete !\n");
            //printf("Motor %d %s : Zero Point Found : %8.6f, homming length : %8.6f ! ", slv_number, ELMO_NAME[slv_number].c_str(), q_zero_elmo_[slv_number], abs(elmofz[slv_number].posStart - elmofz[slv_number].posEnd));
            //fflush(stdout);

            // printf("\33[2K\r");
            // for(int i=0;i<16;i++)
            // {

            // }

            //pub_to_gui(dc, "jointzp %d %d", slv_number, 1);
            elmofz[slv_number].result = ElmoHommingStatus::SUCCESS;
            state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_SUCCESS;
            //                                        //std::cout << slv_number << "Start : " << elmofz[slv_number].posStart << "End:" << elmofz[slv_number].posEnd << '\n';
            //                                        //q_desired_elmo_[slv_number] = positionZeroElmo(slv_number);
            elmofz[slv_number].findZeroSequence = 8; // torque to zero -> 8 position hold -> 5
            ElmoMode[slv_number] = EM_TORQUE;
            torque_desired_elmo_[slv_number] = 0.0;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 5)
    {
        //find zero complete, hold zero position.
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = q_zero_elmo_[slv_number];
    }
    else if (elmofz[slv_number].findZeroSequence == 6)
    {
        //find zero point failed
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(time_now_, elmofz[slv_number].initPos, elmofz[slv_number].firstPos - elmofz[slv_number].initPos, elmofz[slv_number].initTime, fztime);
        if (time_now_ > (elmofz[slv_number].initTime + fztime))
        {
            elmofz[slv_number].findZeroSequence = 7;
            printf("Motor %d %s : Zero point detection Failed. Manual Detection Required. \n", slv_number, ELMO_NAME[slv_number]);
            //pub_to_gui(dc, "jointzp %d %d", slv_number, 2);
            elmofz[slv_number].result = ElmoHommingStatus::FAILURE;
            state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_MANUAL_REQUIRED;
            elmofz[slv_number].initTime = time_now_;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 7)
    {
        ElmoMode[slv_number] = EM_TORQUE;
        torque_desired_elmo_[slv_number] = 0.0;
        if (hommingElmo[slv_number] && hommingElmo_before[slv_number])
        {
            elmofz[slv_number].findZeroSequence = 1;
            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
        if (time_now_ > (elmofz[slv_number].initTime + fztime_manual))
        {
            printf("Motor %d %s :  Manual Detection Failed. \n", slv_number, ELMO_NAME[slv_number]);
            //pub_to_gui(dc, "jointzp %d %d", slv_number, 3);
            elmofz[slv_number].findZeroSequence = 8;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 8)
    {
        ElmoMode[slv_number] = EM_TORQUE;
        torque_desired_elmo_[slv_number] = 0.0;
    }
}
