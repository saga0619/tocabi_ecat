#include "tocabi_ecat.h"
#include <cstring>

int main(void)
{
    std::cout << "hello world" << std::endl;

    std::thread thread1, thread2;

    //RT THREAD FIRST!
    thread1 = std::thread(ethercatThread1);

    //EthercatElmo Management Thread
    thread2 = std::thread(ethercatThread2);

    sched_param sch;
    int policy;
    int priority = 39;
    pthread_getschedparam(thread1.native_handle(), &policy, &sch);

    sch.sched_priority = priority;
    if (pthread_setschedparam(thread1.native_handle(), SCHED_FIFO, &sch))
    {
        std::cout << "Failed to setschedparam: " << std::strerror(errno) << std::endl;
    }

    thread1.join();
    thread2.join();


    std::cout << "tocabiEcat Shutdown" << std::endl;
    return 0;
}