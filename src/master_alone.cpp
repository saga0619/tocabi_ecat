#include "tocabi_ecat/ecat_master.h"
#include <iostream>

static int latency_target_fd = -1;
static int32_t latency_target_value = 0;

#define stack64k (64 * 1024)
int main(int argc, char *argv[])
{
    // ios::sync_with_stdio(false);
    bool val_received = false;

    int lock_core_ = 0;

    if (argc != 7)
    {
        printf("usage : tocabi_ecat {port1} {port2} {period_us} {ecat num} {starting num} {Core}\n");
        return 0;
    }
    
    TocabiInitArgs init_args;
    strcpy(init_args.port1,argv[1]);
    strcpy(init_args.port2,argv[2]);
    // soem_port = argv[1];
    init_args.ecat_device = 0;
    int period_us = atoi(argv[3]);
    init_args.period_ns = period_us * 1000;
    int expected_counter = atoi(argv[4]);
    init_args.ecat_slave_num = expected_counter;
    int start_joint_ = atoi(argv[5]);
    init_args.ecat_slave_start_num = start_joint_;
    lock_core_ = atoi(argv[6]);
    init_args.lock_core = lock_core_;
    
    int max_jnum = 33;
    for (int i = 0; i < init_args.ecat_slave_num; i++)
    {
        if (max_jnum > JointMap2[init_args.ecat_slave_start_num + i])
        {
            max_jnum = JointMap2[init_args.ecat_slave_start_num + i];
        }
    }
    init_args.q_start_ = max_jnum;

    int period_ns = period_us * 1000;

    std::cout << " ecat port : " << init_args.port1 << std::endl;
    std::cout << " ecat port : " << init_args.port2 << std::endl;
    std::cout << " period_ns  : " << period_ns << std::endl;
    std::cout << " elmo num  : " << expected_counter << std::endl;
    std::cout << " start from : " << start_joint_ << ", : " << ELMO_NAME[start_joint_] << std::endl;
    std::cout << " locked at core " << lock_core_ << std::endl;
    std::cout << " ----------------------------- " << std::endl;
    std::cout << " command :  q(quit), l(lower init), u(upper init), d(debug), p(position), h(homming), c(force control), o(lock), f(torque off), w(status log)" << std::endl;

    val_received = true;

    struct sched_param param;
    pthread_attr_t attr, attr2;
    pthread_t thread1, thread2;
    int ret = 0;


    printf("[ECAT - INFO] start init process...\n");
    initTocabiArgs(init_args);
    bool init_result = initTocabiSystem(init_args);
    if (!init_result)
    {
        printf("[ECAT - ERRO] init failed\n");
        return -1;
    }

    printf("[ECAT - INFO] init process has been done\n");
    printf("[ECAT - INFO] start main threads\n");
    /* Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr);
    if (ret)
    {
        printf("init pthread attributes failed\n");
        return ret;
    }

    ret = pthread_attr_init(&attr2);
    if (ret)
    {
        printf("init pthread attributes failed\n");
        return ret;
    }

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret)
    {
        printf("pthread setschedpolicy failed\n");
        return ret;
    }
    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret)
    {
        printf("pthread setschedparam failed\n");
        return ret;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(lock_core_, &cpuset);

    ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
    if (ret)
    {
        printf("pthread setaffinity failed\n");
        return ret;
    }

    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret)
    {
        printf("pthread setinheritsched failed\n");
        return ret;
    }
    // /* Create a pthread with specified attributes */
    ret = pthread_create(&thread1, &attr, ethercatThread1, &init_args);
    if (ret)
    {
        printf("create pthread 1 failed\n");
        return ret;
    }
    ret = pthread_create(&thread2, &attr2, ethercatThread2, &init_args);
    if (ret)
    {
        printf("create pthread 2 failed\n");
        return ret;
    }

    pthread_attr_destroy(&attr);
    pthread_attr_destroy(&attr2);

    /* Join the thread and wait until it is done */
    ret = pthread_join(thread1, NULL);
    if (ret)
        printf("join pthread failed: %m\n");

    ret = pthread_join(thread2, NULL);
    if (ret)
        printf("join pthread failed: %m\n");

    printf("[ECAT - INFO] cleaning up\n");
    cleanupTocabiSystem();
    return 0;
}