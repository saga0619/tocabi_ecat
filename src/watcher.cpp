#include <limits.h>
#include <pthread.h>
#include <thread>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/ipc.h>

#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

#include <cstring>
#include <atomic>
#include <chrono>

#include "shm_msgs.h"

void init_shm()
{
  int data_number = 40 * 5;

  if ((shm_msg_id = shmget(shm_msg_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
  {
    std::cout << "shm mtx failed " << std::endl;
    exit(0);
  }

  if ((shm_msgs_ = (SHMmsgs *)shmat(shm_msg_id, NULL, 0)) == (SHMmsgs *)-1)
  {
    std::cout << "shmat failed " << std::endl;
    exit(0);
  }
}

int main()
{
  init_shm();

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    printf("\33[2K\r  cnt %10d lat avg %6.3f max %6.3f min %6.3f dev %6.4f, send avg %6.3f max %6.3f min %6.3f dev %6.4f head pos %6.4f", (int)shm_msgs_->t_cnt, 
      shm_msgs_->lat_avg/1000.0, shm_msgs_->lat_max/1000.0, shm_msgs_->lat_min/1000.0, shm_msgs_->lat_dev/1000.0,
      shm_msgs_->send_avg/1000.0, shm_msgs_->send_max/1000.0, shm_msgs_->send_min/1000.0,  shm_msgs_->send_dev/1000.0,
      shm_msgs_->pos[0]
      );
    std::fflush(stdout);

    if (shm_msgs_->t_cnt > 100000000)
    {
      break;
    }
  }

  return 0;
}
