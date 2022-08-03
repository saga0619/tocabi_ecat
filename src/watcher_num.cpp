#include <limits.h>
#include <pthread.h>
#include <thread>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <signal.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

#include <cstring>
#include <atomic>
#include <chrono>

#include "shm_msgs.h"
#include "tocabi_ecat/ecat_settings.h"

int main()
{
  int shm_id_;
  SHMmsgs *shm_msgs_;

  init_shm(shm_msg_key, shm_id_, &shm_msgs_);

  // shm_msgs_->

  return 0;
}
