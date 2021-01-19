#include "tocabi_ecat.h"

void ethercatCheck()
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
                    printf("%sERROR : slave %d is in SAFE_OP + ERROR, attempting ack.%s\n", cred.c_str(), slave - 1, creset.c_str());
                    ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                    ec_writestate(slave);
                }
                else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                {
                    printf("%sWARNING : slave %d is in SAFE_OP, change to OPERATIONAL.%s\n", cred.c_str(), slave - 1, creset.c_str());
                    ec_slave[slave].state = EC_STATE_OPERATIONAL;
                    ec_writestate(slave);
                }
                else if (ec_slave[slave].state > 0)
                {
                    if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("%sMESSAGE : slave %d reconfigured%s\n", cgreen.c_str(), slave - 1, creset.c_str());
                    }
                }
                else if (!ec_slave[slave].islost)
                {
                    // re-check state
                    ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                    if (!ec_slave[slave].state)
                    {
                        ec_slave[slave].islost = TRUE;
                        printf("%sERROR : slave %d lost %s\n", cred.c_str(), slave - 1, creset.c_str());
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
                        printf("%sMESSAGE : slave %d recovered%s\n", cgreen.c_str(), slave - 1, creset.c_str());
                    }
                }
                else
                {
                    ec_slave[slave].islost = FALSE;
                    printf("%sMESSAGE : slave %d found%s\n", cgreen.c_str(), slave - 1, creset.c_str());
                }
            }
        }
    }
}

void ethercatThread1()
{
    char IOmap[4096] = {};
    bool reachedInitial[ELMO_DOF] = {false};

    string ifname_str = "enp4s0";

    const char *ifname = ifname_str.c_str();

    if (ec_init(ifname))
    {
        printf("ELMO : ec_init on %s succeeded.\n", ifname);

        /* find and auto-config slaves */
        /* network discovery */
        //ec_config_init()
        if (ec_config_init(FALSE) > 0) // TRUE when using configtable to init slaves, FALSE otherwise
        {
            printf("ELMO : %d slaves found and configured.\n", ec_slavecount); // ec_slavecount -> slave num
            if (ec_slavecount == ELMO_DOF)
            {
                ecat_number_ok = true;
            }
            else
            {
                std::cout << cred << "WARNING : SLAVE NUMBER INSUFFICIENT" << creset << std::endl;
                de_shutdown = true;
            }
            /** CompleteAccess disabled for Elmo driver */
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                //printf("ELMO : Has Slave[%d] CA? %s\n", slave, ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");
                if (!(ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA))
                {
                    printf("ELMO : slave[%d] CA? : false , shutdown request \n ", slave);
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
            ec_config_map(&IOmap);

            /* wait for all slaves to reach SAFE_OP state */
            printf("ELMO : EC WAITING STATE TO SAFE_OP\n");
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("ELMO : Request operational state for all slaves. Calculated workcounter : %d\n", expectedWKC);

            if (expectedWKC != 3 * ELMO_DOF)
            {
                std::cout << cred << "WARNING : Calculated Workcounter insufficient!" << creset << std::endl;
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

            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 5000);
            } while (wait_cnt-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                inOP = TRUE;

                /* cyclic loop */
                for (int slave = 1; slave <= ec_slavecount; slave++)
                {
                    txPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *)(ec_slave[slave].outputs);
                    rxPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *)(ec_slave[slave].inputs);
                }

                //Commutation Checking
                st_start_time = std::chrono::steady_clock::now();
                cout << "ELMO : Initialization Mode" << endl;
                while (!de_shutdown)
                {
                    std::this_thread::sleep_until(st_start_time + cycle_count * cycletime);
                    cycle_count++;
                    wkc = ec_receive_processdata(0);

                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        elmost[i].state = getElmoState(rxPDO[i]->statusWord);

                        if (elmost[i].state != elmost[i].state_before)
                        {
                            if (elmost[i].first_check)
                            {
                                if (elmost[i].state == ELMO_NOTFAULT)
                                {
                                    elmost[i].commutation_required = true;
                                }
                                else if (elmost[i].state == ELMO_FAULT)
                                {
                                    //cout << "slave : " << i << " commutation check complete at first" << endl;
                                    elmost[i].commutation_not_required = true;
                                }
                                else if (elmost[i].state == ELMO_OPERATION_ENABLE)
                                {
                                    //cout << "slave : " << i << " commutation check complete with operation enable" << endl;
                                    elmost[i].commutation_not_required = true;
                                    elmost[i].commutation_ok = true;
                                }
                                else
                                {
                                    //cout << "first missing : slave : " << i << " state : " << elmost[i].state << endl;
                                }
                                elmost[i].first_check = false;
                            }
                            else
                            {
                                if (elmost[i].state == ELMO_OPERATION_ENABLE)
                                {
                                    //cout << "slave : " << i << " commutation check complete with operation enable 2" << endl;
                                    elmost[i].commutation_ok = true;
                                    elmost[i].commutation_required = false;
                                }
                            }
                        }
                        elmost[i].state_before = elmost[i].state;
                    }

                    bool waitop = true;
                    for (int i = 0; i < ec_slavecount; i++)
                        waitop = waitop && elmost[i].commutation_ok;

                    //bool commutation_required = false;
                    //for (int i = 0; i < ec_slavecount; i++)
                    //    commutation_required = commutation_required || elmost[i].commutation_required;

                    if (waitop)
                    {
                        if (de_commutation_done)
                        {
                            if (saveCommutationLog())
                            {
                                cout << "ELMO : Commutation is done, logging success" << endl;
                                for (int i = 0; i < ELMO_DOF; i++)
                                {
                                    q_zero_elmo_[i] = i;
                                }
                            }
                            else
                            {
                                cout << "ELMO : Commutaion is done, logging failed" << endl;
                            }
                        }
                        cout << "ELMO : All slaves Operational" << endl;

                        break;
                    }

                    bool waitcm = true;
                    for (int i = 0; i < ec_slavecount; i++)
                        waitcm = waitcm && elmost[i].commutation_not_required;

                    if (waitcm)
                    {
                        if (wait_kill_switch)
                        {
                            cout << "ELMO : Commutation state OK" << endl;
                            loadCommutationLog();
                            loadZeroPoint();
                            wait_kill_switch = false;
                        }
                        if (wait_cnt == 200)
                        {
                            cout << "ELMO : slaves status are not OP! maybe kill switch is on?" << endl;
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
                                de_commutation_done = true;
                                total_commutation_cnt++;
                                if (total_commutation_cnt < 10)
                                    controlWordGenerate(rxPDO[i]->statusWord, txPDO[i]->controlWord);
                                txPDO[i]->maxTorque = (uint16)1000; // originaly 1000
                            }
                        }

                        if (elmost[commutation_joint].commutation_ok)
                        {
                            commutation_joint++;
                            if (commutation_joint > ec_slavecount)
                                commutation_joint = 0;
                        }
                        else
                        {
                            controlWordGenerate(rxPDO[commutation_joint]->statusWord, txPDO[commutation_joint]->controlWord);
                            txPDO[commutation_joint]->maxTorque = (uint16)1000; // originaly 1000
                        }
                    }

                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        if (!elmost[i].commutation_required)
                        {
                            controlWordGenerate(rxPDO[i]->statusWord, txPDO[i]->controlWord);
                            txPDO[i]->maxTorque = (uint16)1000; // originaly 1000
                        }
                    }

                    //zeropoint search

                    ec_send_processdata();
                }

                cout << "ELMO : Control Mode Start ... " << endl;
                st_start_time = std::chrono::steady_clock::now();
                cycle_count = 0;
                while (!de_shutdown)
                {
                    std::this_thread::sleep_until(st_start_time + cycle_count * cycletime);

                    /** PDO I/O refresh */
                    //ec_send_processdata();
                    wkc = ec_receive_processdata(0);
                    /*
                    elmost[20].check_value = rxPDO[20]->statusWord;
                    if (elmost[20].check_value != elmost[20].check_value_before)
                    {
                        elmost[20].state = getElmoState(elmost[20].check_value);
                        cout << "20 : " << elmost[20].state << endl;
                    }
                    elmost[20].check_value_before = rxPDO[20]->statusWord;*/

                    if (wkc >= expectedWKC)
                    {
                        for (int slave = 1; slave <= ec_slavecount; slave++)
                        {
                            if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
                            {

                                reachedInitial[slave - 1] = true;
                            }
                        }
                        for (int slave = 1; slave <= ec_slavecount; slave++)
                        {
                            if (reachedInitial[slave - 1])
                            {
                                //Get status
                                q_elmo_[slave - 1] = rxPDO[slave - 1]->positionActualValue * CNT2RAD[slave - 1] * elmo_axis_direction[slave - 1];

                                hommingElmo[slave - 1] =
                                    (((uint32_t)ec_slave[slave].inputs[4]) +
                                     ((uint32_t)ec_slave[slave].inputs[5] << 8) +
                                     ((uint32_t)ec_slave[slave].inputs[6] << 16) +
                                     ((uint32_t)ec_slave[slave].inputs[7] << 24));

                                q_dot_elmo_[slave - 1] =
                                    (((int32_t)ec_slave[slave].inputs[10]) +
                                     ((int32_t)ec_slave[slave].inputs[11] << 8) +
                                     ((int32_t)ec_slave[slave].inputs[12] << 16) +
                                     ((int32_t)ec_slave[slave].inputs[13] << 24)) *
                                    CNT2RAD[slave - 1] * elmo_axis_direction[slave - 1];

                                torque_elmo_[slave - 1] =
                                    (((int16_t)ec_slave[slave].inputs[14]) +
                                     ((int16_t)ec_slave[slave].inputs[15] << 8));

                                q_ext_elmo_[slave - 1] =
                                    (((int32_t)ec_slave[slave].inputs[16]) +
                                     ((int32_t)ec_slave[slave].inputs[17] << 8) +
                                     ((int32_t)ec_slave[slave].inputs[18] << 16) +
                                     ((int32_t)ec_slave[slave].inputs[19] << 24) - q_ext_mod_elmo_[slave - 1]) *
                                    EXTCNT2RAD[slave - 1] * elmo_ext_axis_direction[slave - 1];

                                if (slave == 1 || slave == 2 || slave == 19 || slave == 20)
                                {
                                    hommingElmo[slave - 1] = !hommingElmo[slave - 1];
                                }

                                txPDO[slave - 1]->maxTorque = (uint16)1500; // originaly 1000
                                //ElmoMode[slave - 1] = EM_TORQUE;
                                // torqueDemandElmo[slave - 1] = 0.0;
                            }
                        }
                    }

                    //Homming, state, positionExternal -> internal
                    //position, velocity, torque -> external

                    //Get state Here !!!!!!

                    //Get State Seqence End, user controller start

                    //Check Commutation
                    //

                    // if joint is warmstart
                    //   loadcommutation
                    //   loadzeropoint

                    // if joint is commutation
                    //   if commutation complete
                    //      save commutation time
                    //

                    // if commutation ok && zp not loaded
                    //    wait for fz command
                    //    zp_low_ready  =true
                    //    zp_upper_ready = true

                    //torqueDesiredController = getCommand();

                    //receive command

                    //convert received command to elmo command

                    /*
                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                if (operation_ready)
                                {
                                    if (dc.torqueOn)
                                    {
                                        //If torqueOn command received, torque will increases slowly, for rising_time, which is currently 3 seconds.
                                        to_ratio = DyrosMath::minmax_cut((control_time_real_ - dc.torqueOnTime) / rising_time, 0.0, 1.0);
                                        ElmoMode[i] = EM_TORQUE;
                                        dc.t_gain = to_ratio;

                                        ELMO_torque[i] = to_ratio * ELMO_torque[i];
                                    }
                                    else if (dc.torqueOff)
                                    {
                                        //If torqueOff command received, torque will decreases slowly, for rising_time(3 seconds. )

                                        if (dc.torqueOnTime + rising_time > dc.torqueOffTime)
                                        {
                                            to_calib = (dc.torqueOffTime - dc.torqueOnTime) / rising_time;
                                        }
                                        else
                                        {
                                            to_calib = 0.0;
                                        }
                                        to_ratio = DyrosMath::minmax_cut(1.0 - to_calib - (control_time_real_ - dc.torqueOffTime) / rising_time, 0.0, 1.0);

                                        dc.t_gain = to_ratio;

                                        ELMO_torque[i] = to_ratio * ELMO_torque[i];
                                    }
                                    else
                                    {
                                        ElmoMode[i] = EM_TORQUE;
                                        ELMO_torque[i] = 0.0;
                                    }
                                }
                                else
                                {
                                    if ((!zp_upper_check) && (!zp_low_check))
                                    {
                                        ElmoMode[i] = EM_TORQUE;
                                        ELMO_torque[i] = 0.0;
                                    }
                                }
                            }

                            //ECAT JOINT COMMAND
                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                if (ElmoMode[i] == EM_POSITION)
                                {
                                    txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
                                    txPDO[i]->targetPosition = (int)(Dr[i] * RAD2CNT[i] * q_desired_elmo_[i]);
                                }
                                else if (ElmoMode[i] == EM_TORQUE)
                                {
                                    txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;

                                    if (dc.customGain)
                                    {
                                        txPDO[i]->targetTorque = (int)(ELMO_torque[i] * CustomGain[i] * Dr[i]);
                                    }
                                    else
                                    {
                                        txPDO[i]->targetTorque = (roundtoint)(ELMO_torque[i] * ELMO_NM2CNT[i] * Dr[i]);
                                    }
                                }
                                else if (ElmoMode[i] == EM_COMMUTATION)
                                {
                                }
                                else
                                {
                                    txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                                    txPDO[i]->targetTorque = (int)0;
                                }
                            }*/
                    /*
                            bool ecat_lost_before = de_ecat_lost;
                            de_ecat_lost = false;
                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                if (ec_slave[i].islost)
                                {
                                    de_ecat_lost = de_ecat_lost || true;
                                }
                            }

                            if ((ecat_lost_before) && (!de_ecat_lost))
                            {
                                de_ecat_recovered = true;
                            }

                            if (de_ecat_lost)
                            {
                                for (int i = 0; i < ec_slavecount; i++)
                                {
                                    txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                                    txPDO[i]->targetTorque = (int)0;
                                }
                            }

                            //Hold position if safety limit breached
                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                if (ElmoMode[i] != EM_POSITION)
                                {
                                    checkPosSafety[i] = false;
                                }

                                checkSafety(i, joint_velocity_limit[i], 10.0 * CYCLETIME / 1E+6); //if angular velocity exceeds 0.5rad/s, Hold to current Position ///
                            }
                            */

                    //Torque off if emergency off received
                    if (de_emergency_off)
                        emergencyOff();

                    //std::this_thread::sleep_until(st_start_time + cycle_count * cycletime+ std::chrono::microseconds(250));

                    ec_send_processdata();

                    /*
                            if (dc.disableSafetyLock)
                            {
                                for (int i = 0; i < ec_slavecount; i++)
                                {
                                    ElmoSafteyMode[i] = 0;
                                }
                                dc.disableSafetyLock = false;
                            }
                            

                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        if (ElmoMode[i] == EM_POSITION)
                        {
                            checkPosSafety[i] = true;
                        }
                    }

                     */
                    cycle_count++;
                }

                inOP = FALSE;
            }
            else
            {
                printf("%sELMO : Not all slaves reached operational state.%s\n", cred.c_str(), creset.c_str());
                ec_readstate();
                for (int slave = 1; slave <= ec_slavecount; slave++)
                {
                    if (ec_slave[slave - 1].state != EC_STATE_OPERATIONAL)
                    {
                        printf("%sELMO : EtherCAT State Operation Error : Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s%s\n", cred.c_str(), slave - 1, ec_slave[slave - 1].state, ec_slave[slave - 1].ALstatuscode, ec_ALstatuscode2string(ec_slave[slave - 1].ALstatuscode), creset.c_str());
                    }
                }
            }
            printf("\nELMO : Request init state for all slaves\n");
            /** request INIT state for all slaves
                    *  slave number = 0 -> write to all slaves
                    */
            ec_slave[0].state = EC_STATE_INIT;
            ec_writestate(0);

            printf("ELMO : Checking EC STATE ... \n");
            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
            printf("ELMO : Checking EC STATE Complete \n");
        }
        else
        {
            printf("%sELMO : No slaves found!%s\n", cred.c_str(), creset.c_str());
        }
    }
    else
    {
        printf("ELMO : No socket connection on %s\nExcecute as root\n", ifname);
    }

    std::cout << "ELMO : EthercatThread1 Shutdown" << std::endl;
}

void ethercatThread2()
{
    while (!de_shutdown)
    {
        ethercatCheck();

        int ch = kbhit();
        if (ch != -1)
        {
            //std::cout << "key input : " << (char)(ch % 256) << std::endl;
            if ((ch % 256 == 'q'))
            {
                std::cout << "ELMO : shutdown request" << std::endl;
                de_shutdown = true;
            }
            else if ((ch % 256 == 'i'))
            {
                std::cout << "ELMO : start searching zero point" << std::endl;
                de_initialize = true;
            }
            else if ((ch % 256 == 'd'))
            {
                std::cout << "ELMO : start debug mode" << std::endl;
                de_debug_level++;
                if (de_debug_level > 2)
                    de_debug_level = 0;
            }

            this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::cout << "ELMO : EthercatThread2 Shutdown" << std::endl;
}

double elmoJointMove(double init, double angle, double start_time, double traj_time)
{
    double des_pos;

    if (control_time_real_ < start_time)
    {
        des_pos = init;
    }
    else if ((control_time_real_ >= start_time) && (control_time_real_ < (start_time + traj_time)))
    {
        des_pos = init + angle * (control_time_real_ - start_time) / traj_time;
    }
    // else if ((control_time_real_ >= (start_time + traj_time)) && (control_time_real_ < (start_time + 3 * traj_time)))
    //{
    //    des_pos = init + angle - 2 * angle * (control_time_real_ - (start_time + traj_time)) / traj_time;
    //}
    else if (control_time_real_ > (start_time + traj_time))
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
}

void checkJointStatus()
{
}

void sendJointStatus(array<atomic<double>, ELMO_DOF> q_elmo, array<atomic<double>, ELMO_DOF> q_dot_elmo, array<atomic<int>, ELMO_DOF> joint_state_elmo, array<atomic<double>, ELMO_DOF> torque_elmo)
{
    int sendJointPos, sendJointDot, sendJointCurrent, sendJointStatus;
    double *jointPos, *jointDot, *jointCurrent;
    int *jointStatus;

    sendJointPos = shmget(sendJointKey, sizeof(q_elmo_), 0666|IPC_CREAT);
    sendJointDot = shmget(sendJointDotKey, sizeof(q_dot_elmo_), 0666|IPC_CREAT);
    sendJointCurrent = shmget(sendJointCurrentKey, sizeof(torque_elmo_), 0666|IPC_CREAT);
    sendJointStatus = shmget(sendJointStatusKey, sizeof(joint_state_elmo), 0666|IPC_CREAT);

    if(sendJointPos == -1)
    {
        printf("SendJointPos shmget send failed");
    }

    if(sendJointDot == -1)
    {
        printf("SendJointDot shmget send failed");
    }

    if(sendJointCurrent == -1)
    {
        printf("SendJointCurrent shmget send failed");
    }

    if(sendJointStatus == -1)
    {
        printf("sendJointStatus shmget send failed");
    }
    
    jointPos = (double*)shmat(sendJointPos, NULL, 0);
    jointDot = (double*)shmat(sendJointDot, NULL, 0);
    jointCurrent = (double*)shmat(sendJointCurrent, NULL, 0);
    jointStatus = (int*)shmat(sendJointStatus, NULL, 0);

    for(int i = 0; i < ELMO_DOF; i++)
    {
        jointPos[i] = q_elmo[i];
        jointDot[i] = q_dot_elmo[i];
        jointCurrent[i] = torque_elmo[i];
        jointStatus[i] = joint_state_elmo[i];
    }

    shmdt(jointPos);
    shmdt(jointDot);
    shmdt(jointCurrent);
    shmdt(jointStatus);
}

void getJointCommand(array<atomic<double>, ELMO_DOF> &torque_desired)
{
    int getJointTorque;
    double *jointTorque;

    getJointTorque = shmget(getJointDesiredTorqueKey, sizeof(torque_desired), 0666|IPC_CREAT);

    if(getJointTorque == -1)
    {
        printf("GetJointTorque shmget send failed");
    }

    jointTorque = (double*)shmat(getJointTorque, NULL, 0);

    for(int i = 0; i < ELMO_DOF; i++)
    {
        torque_desired[i] = jointTorque[i];
    }

    shmdt(jointTorque);
}

bool saveCommutationLog()
{
    std::ofstream comfs(commutation_cache_file, std::ios::binary);

    if (!comfs.is_open())
    {
        return false;
    }

    auto const cache_time = (chrono::system_clock::now()).time_since_epoch().count();
    comfs.write(reinterpret_cast<char const *>(&cache_time), sizeof cache_time);
    comfs.close();
    return true;
}

bool loadCommutationLog()
{
    std::ifstream ifs(commutation_cache_file, std::ios::binary);

    if (!ifs.is_open())
    {
        return false;
    }

    std::chrono::system_clock::rep file_time_rep;
    if (!ifs.read(reinterpret_cast<char *>(&file_time_rep), sizeof file_time_rep))
    {
        return false;
    }
    ifs.close();

    std::chrono::system_clock::time_point const cache_valid_time{std::chrono::system_clock::duration{file_time_rep}};
    std::time_t const file_time = std::chrono::system_clock::to_time_t(cache_valid_time);

    std::chrono::duration<double> commutation_before = std::chrono::system_clock::now() - cache_valid_time;
    std::cout << std::ctime(&file_time);
    std::cout << "done " << commutation_before.count() << "seconds before .... " << std::endl;
}

bool saveZeroPoint()
{
    std::ofstream comfs(zeropoint_cache_file, std::ios::binary);

    if (!comfs.is_open())
    {
        return false;
    }

    auto const cache_time = (chrono::system_clock::now()).time_since_epoch().count();
    comfs.write(reinterpret_cast<char const *>(&cache_time), sizeof cache_time);

    for (int i = 0; i < ELMO_DOF; i++)
        comfs.write(reinterpret_cast<char const *>(&q_zero_elmo_[i]), sizeof(double));

    comfs.close();
    return true;
}

bool loadZeroPoint()
{
    std::ifstream ifs(zeropoint_cache_file, std::ios::binary);

    if (!ifs.is_open())
    {
        return false;
    }

    std::chrono::system_clock::rep file_time_rep;

    ifs.read(reinterpret_cast<char *>(&file_time_rep), sizeof file_time_rep);
    double getzp[ELMO_DOF];
    for (int i = 0; i < ELMO_DOF; i++)
        ifs.read(reinterpret_cast<char *>(&getzp[i]), sizeof(double));

    ifs.close();

    std::chrono::system_clock::time_point const cache_valid_time{std::chrono::system_clock::duration{file_time_rep}};
    std::time_t const file_time = std::chrono::system_clock::to_time_t(cache_valid_time);

    std::chrono::duration<double> commutation_before = std::chrono::system_clock::now() - cache_valid_time;
    std::cout << "ZP saved at " << std::ctime(&file_time);
    std::cout << "ZP saved at " << commutation_before.count() << "seconds before .... " << std::endl;
    for (int i = 0; i < ELMO_DOF; i++)
        cout << getzp[i] << endl;
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
        std::cout << ELMO_NAME[i + R_HipRoll_Joint] << " pz IE P : " << q_elmo_[i + R_HipRoll_Joint] << " pz EE P : " << q_ext_elmo_[i + R_HipRoll_Joint] << std::endl;
        std::cout << ELMO_NAME[i + L_HipRoll_Joint] << " pz ELMO : " << q_elmo_[i + L_HipRoll_Joint] << " pz ELMO : " << q_ext_elmo_[i + L_HipRoll_Joint] << std::endl;
    }
}
void findZeroPointlow(int slv_number)
{
    double velocity = 0.1;
    double fztime = 3.0;
    if (elmofz[slv_number].findZeroSequence == FZ_CHECKHOMMINGSTATUS)
    {
        elmofz[slv_number].initTime = control_time_real_;
        elmofz[slv_number].initPos = q_elmo_[slv_number];
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

            std::cout << cred << "elmo reboot required. joint " << slv_number << "external encoder error" << q_ext_elmo_[slv_number] << std::endl;
        }
        else if (slv_number == 24)
        {
            std::cout << "positionExternal OK " << q_ext_elmo_[slv_number] << std::endl;
        }
    }

    if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGSTART)
    {

        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].initPos, elmofz[slv_number].init_direction * 0.6, elmofz[slv_number].initTime, fztime * 4.0);

        if (control_time_real_ == elmofz[slv_number].initTime)
        {
            //std::cout << "joint " << slv_number << "  init pos : " << elmofz[slv_number].initPos << "   goto " << elmofz[slv_number].initPos + elmofz[slv_number].init_direction * 0.6 << std::endl;
        }

        if (q_ext_elmo_[slv_number] * elmofz[slv_number].init_direction > 0)
        {
            q_zero_elmo_[slv_number] = q_elmo_[slv_number];
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGEND;
            //pub_to_gui(dc, "jointzp %d %d", slv_number, 1);
            elmofz[slv_number].result = ElmoHommingStatus::SUCCESS;
        }
        if (control_time_real_ > elmofz[slv_number].initTime + fztime * 4.0)
        {
            elmofz[slv_number].result == ElmoHommingStatus::FAILURE;
        }
    }
}
void findZeroPoint(int slv_number)
{
    double fztime = 3.0;
    double fztime_manual = 300.0;
    if (elmofz[slv_number].findZeroSequence == FZ_CHECKHOMMINGSTATUS)
    {
        //pub_to_gui(dc, "jointzp %d %d", slv_number, 0);
        if (hommingElmo[slv_number])
        {
            //std::cout << "motor " << slv_number << " init state : homming on" << std::endl;
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGSTART;
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
            elmofz[slv_number].firstPos = q_elmo_[slv_number];
        }
        else
        {
            //std::cout << "motor " << slv_number << " init state : homming off" << std::endl;
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMING;
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
            elmofz[slv_number].firstPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGSTART)
    {
        //go to + 0.3rad until homming sensor turn off
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].initPos, 0.3, elmofz[slv_number].initTime, fztime);

        if ((hommingElmo[slv_number] == 0) && (hommingElmo_before[slv_number] == 0))
        {
            //std::cout << "motor " << slv_number << " seq 1 complete, wait 1 sec" << std::endl;
            hommingElmo_before[slv_number] = hommingElmo[slv_number];
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGEND;
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].posStart = q_elmo_[slv_number];
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGEND)
    {
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].posStart, -0.3, elmofz[slv_number].initTime, fztime);

        //go to -20deg until homming turn on, and turn off
        if ((hommingElmo_before[slv_number] == 1) && (hommingElmo[slv_number] == 0))
        {
            if (abs(elmofz[slv_number].posStart - q_elmo_[slv_number]) > elmofz[slv_number].req_length)
            {
                elmofz[slv_number].posEnd = q_elmo_[slv_number];
                elmofz[slv_number].endFound = 1;
            }
            else
            {
                std::cout << "Motor " << slv_number << " : Not enough length start point : " << elmofz[slv_number].posStart << ", Current Point " << q_elmo_[slv_number] << endl;

                //std::cout << "off : homming turned off, but not enough length start point : " << elmofz[slv_number].posStart << " Current off point : " << q_elmo_[slv_number] << std::endl;
            }
        }
        else if ((hommingElmo_before[slv_number] == 0) && (hommingElmo[slv_number] == 0))
        {
            if (elmofz[slv_number].endFound == 1)
            {
                //std::cout << "motor " << slv_number << " seq 2 complete" << std::endl;
                elmofz[slv_number].findZeroSequence = FZ_GOTOZEROPOINT;
                elmofz[slv_number].initPos = q_elmo_[slv_number];
                q_zero_elmo_[slv_number] = (elmofz[slv_number].posEnd + elmofz[slv_number].posStart) * 0.5 + q_zero_mod_elmo_[slv_number];
                elmofz[slv_number].initTime = control_time_real_;
                //std::cout << "on : Motor " << slv_number << " zero point found : " << positionZeroElmo[slv_number] << std::endl;
            }
        }

        if (control_time_real_ > elmofz[slv_number].initTime + fztime)
        {
            //If dection timeout, go to failure sequence
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].findZeroSequence = 6;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMING)
    { //start from unknown

        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].initPos, elmofz[slv_number].init_direction * 0.3, elmofz[slv_number].initTime, fztime);
        if (control_time_real_ > (elmofz[slv_number].initTime + fztime))
        {
            q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].initPos + 0.3 * elmofz[slv_number].init_direction, -0.6 * elmofz[slv_number].init_direction, elmofz[slv_number].initTime + fztime, fztime * 2.0);
        }

        if (hommingElmo[slv_number] && hommingElmo_before[slv_number])
        {
            //std::cout << "homming found ! to sequence 1 ! " << std::endl;
            elmofz[slv_number].findZeroSequence = 1;
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }

        if (control_time_real_ > (elmofz[slv_number].initTime + fztime * 3.0))
        {
            //If dection timeout, go to failure sequence
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].findZeroSequence = 6;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_GOTOZEROPOINT)
    {
        ElmoMode[slv_number] = EM_POSITION;

        double go_to_zero_dur = fztime * (abs(q_zero_elmo_[slv_number] - elmofz[slv_number].initPos) / 0.3);
        q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].initPos, q_zero_elmo_[slv_number] - elmofz[slv_number].initPos, elmofz[slv_number].initTime, go_to_zero_dur);

        //go to zero position
        if (control_time_real_ > (elmofz[slv_number].initTime + go_to_zero_dur))
        {
            //std::cout << "go to zero complete !" << std::endl;
            printf("Motor %d %s : Zero Point Found : %8.6f, homming length : %8.6f ! \n", slv_number, ELMO_NAME[slv_number].c_str(), q_zero_elmo_[slv_number], abs(elmofz[slv_number].posStart - elmofz[slv_number].posEnd));
            //pub_to_gui(dc, "jointzp %d %d", slv_number, 1);
            elmofz[slv_number].result = ElmoHommingStatus::SUCCESS;
            //std::cout << slv_number << "Start : " << elmofz[slv_number].posStart << "End:" << elmofz[slv_number].posEnd << std::endl;
            //q_desired_elmo_[slv_number] = positionZeroElmo(slv_number);
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
        q_desired_elmo_[slv_number] = elmoJointMove(elmofz[slv_number].initPos, elmofz[slv_number].firstPos - elmofz[slv_number].initPos, elmofz[slv_number].initTime, fztime);
        if (control_time_real_ > (elmofz[slv_number].initTime + fztime))
        {
            elmofz[slv_number].findZeroSequence = 7;
            printf("Motor %d %s : Zero point detection Failed. Manual Detection Required. \n", slv_number, ELMO_NAME[slv_number].c_str());
            //pub_to_gui(dc, "jointzp %d %d", slv_number, 2);
            elmofz[slv_number].result = ElmoHommingStatus::FAILURE;
            elmofz[slv_number].initTime = control_time_real_;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 7)
    {
        ElmoMode[slv_number] = EM_TORQUE;
        torque_desired_elmo_[slv_number] = 0.0;
        if (hommingElmo[slv_number] && hommingElmo_before[slv_number])
        {
            elmofz[slv_number].findZeroSequence = 1;
            elmofz[slv_number].initTime = control_time_real_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
        if (control_time_real_ > (elmofz[slv_number].initTime + fztime_manual))
        {
            printf("Motor %d %s :  Manual Detection Failed. \n", slv_number, ELMO_NAME[slv_number].c_str());
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