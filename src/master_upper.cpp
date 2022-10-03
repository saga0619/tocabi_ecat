#include "tocabi_ecat/ecat_master.h"
#include "sys/mman.h"
// #include <iostream>
#include <signal.h>


void SIGINT_handler(int sig)
{
    shutdownSystem();
}

int main(int argc, char **argv)
{
    mlockall(MCL_CURRENT | MCL_FUTURE);
    signal(SIGINT, SIGINT_handler);

    TocabiInitArgs init_args;
    strcpy(init_args.port1, "rteth1");
    strcpy(init_args.port2, "rteth0");
    // strcpy(init_args.port1, "enp7s0f1");
    // strcpy(init_args.port2, "enp7s0f0");

    init_args.period_ns = 500 * 1000;
    init_args.ecat_slave_num = 18;
    init_args.ecat_slave_start_num = 0;
    init_args.lock_core = 7;
    init_args.ecat_device = 1;
    init_args.is_main = true;
    init_args.verbose = false;

    strcpy(init_args.commutation_cache_file, "/home/dyros/.tocabi_bootlog/commutationlog_upper");
    strcpy(init_args.zeropoint_cache_file, "/home/dyros/.tocabi_bootlog/zeropointlog_upper");

    int max_jnum = 33;
    for (int i = 0; i < init_args.ecat_slave_num; i++)
    {
        if (max_jnum > JointMap2[init_args.ecat_slave_start_num + i])
        {
            max_jnum = JointMap2[init_args.ecat_slave_start_num + i];
        }
    }
    init_args.q_start_ = max_jnum;

    struct sched_param param, param2;
    pthread_attr_t attr, attr2;
    pthread_t thread1, thread2, thread3;
    int ret;

    initTocabiArgs(init_args);

    
    bool init_result = initTocabiSystem(init_args);
    if (!init_result)
    {
        printf("[ECAT - ERRO] init failed\n");
        return -1;
    }

    // printf("[ECAT - INFO] start main threads\n");
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
    param.sched_priority = 47+50;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret)
    {
        printf("pthread setschedparam failed\n");
        return ret;
    }

    // ret = pthread_attr_setschedpolicy(&attr2, SCHED_FIFO);
    // if (ret)
    // {
    //     printf("pthread setschedpolicy failed\n");
    //     return ret;
    // }
    // param2.sched_priority = 40;
    // ret = pthread_attr_setschedparam(&attr2, &param2);
    // if (ret)
    // {
    //     printf("pthread setschedparam failed\n");
    //     return ret;
    // }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(7, &cpuset);

    ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
    if (ret)
    {
        printf("pthread setaffinity failed\n");
        return ret;
    }
    cpu_set_t cpuset2;
    CPU_ZERO(&cpuset2);
    CPU_SET(6, &cpuset2);

    ret = pthread_attr_setaffinity_np(&attr2, sizeof(cpuset2), &cpuset2);
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

    // ret = pthread_attr_setinheritsched(&attr2, PTHREAD_EXPLICIT_SCHED);
    // if (ret)
    // {
    //     printf("pthread setinheritsched failed\n");
    //     return ret;
    // }
    // std::cout<<"ECATUP"<<std::endl;
    // printf("[ECAT - UP] start init process\n");

    // printf("[ECAT - INFO] init process has been done\n");
    /* Create a pthread with specified attributes */

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
    // ret = pthread_create(&thread3, &attr2, ethercatThread3, &init_args);
    // if (ret)
    // {
    //     printf("create pthread 3 failed\n");
    //     return ret;
    // }

    pthread_attr_destroy(&attr);
    pthread_attr_destroy(&attr2);

    /* Join the thread and wait until it is done */
    ret = pthread_join(thread1, NULL);
    if (ret)
        printf("join pthread failed: %m\n");

    ret = pthread_join(thread2, NULL);
    if (ret)
        printf("join pthread failed: %m\n");

    // ret = pthread_join(thread3, NULL);
    // if (ret)
    //     printf("join pthread failed: %m\n");

    printf("[ECAT - INFO] cleaning up\n");
    cleanupTocabiSystem();
    return 0;
}
