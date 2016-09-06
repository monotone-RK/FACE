/******************************************************************************/
/* Measurement of Sort Process Time                          Ryohei Kobayashi */
/*                                                                 2016-08-01 */
/******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include "timer.h"
#include "riffa.h"
#include "xorshift.h"

// #define RANGE  100000000
#define RANGE  (1024*1024*1024)
#define MIN    0
#define MAX    (RANGE - 1)

// for verification
/**********************************************************************/
void QuickSort(int sortdata[], int left, int right) {
  int temp = 0;
  int i = left + 1;
  int k = right;
  
  while (i < k) {
    while (sortdata[i] < sortdata[left] && i < right) i++;
    while (sortdata[k] >= sortdata[left] && k > left) k--;
    if (i < k) {
      temp = sortdata[i];
      sortdata[i] = sortdata[k];
      sortdata[k] = temp;
    }
  }
  if (right-left != 1 || sortdata[left] > sortdata[k]) {
    temp = sortdata[left];
    sortdata[left] = sortdata[k];
    sortdata[k] = temp;
  }
  if (left < k-1)  QuickSort(sortdata, left, k-1);
  if (k+1 < right) QuickSort(sortdata, k+1, right);
  
}

/******************************************************************************/
int GetRandom(int min, int max) {
  return min + (int)(rand()*(max-min+1.0)/(1.0+RAND_MAX));
}

/******************************************************************************/
int data_init(char type[], unsigned int data[], size_t datanum) {
  int i;
  srand(time(NULL));
  // printf("time(NULL): %d\n", (unsigned)time(NULL));

  if      (!strcmp(type, "random"))   { for (i = 0; i < datanum; i++) data[i] = GetRandom(MIN, MAX); return 0; }
  else if (!strcmp(type, "sorted"))   { for (i = 0; i < datanum; i++) data[i] = i + 1;               return 0; }
  else if (!strcmp(type, "reverse"))  { for (i = 0; i < datanum; i++) data[i] = datanum - i;         return 0; }
  else if (!strcmp(type, "xorshift")) {
    for (i = 0; i < datanum; i++) {
      switch (i%16) {
        case 0 : data[i] = xorshift00()&0x7fffffff; break;
        case 1 : data[i] = xorshift01()&0x7fffffff; break;
        case 2 : data[i] = xorshift02()&0x7fffffff; break;
        case 3 : data[i] = xorshift03()&0x7fffffff; break;
        case 4 : data[i] = xorshift04()&0x7fffffff; break;
        case 5 : data[i] = xorshift05()&0x7fffffff; break;
        case 6 : data[i] = xorshift06()&0x7fffffff; break;
        case 7 : data[i] = xorshift07()&0x7fffffff; break;
        case 8 : data[i] = xorshift08()&0x7fffffff; break;
        case 9 : data[i] = xorshift09()&0x7fffffff; break;
        case 10: data[i] = xorshift10()&0x7fffffff; break;
        case 11: data[i] = xorshift11()&0x7fffffff; break;
        case 12: data[i] = xorshift12()&0x7fffffff; break;
        case 13: data[i] = xorshift13()&0x7fffffff; break;
        case 14: data[i] = xorshift14()&0x7fffffff; break;
        case 15: data[i] = xorshift15()&0x7fffffff; break;
      }
    }
    return 0;
  } else {
    return -1;
  }
}

/******************************************************************************/
int main(int argc, char *argv[]) {
  fpga_t *fpga;
  fpga_info_list info;
  int i;
  int id;
  int chnl;
  size_t numWords;
  int sent;
  int recvd;
  unsigned int *sendBuffer;
  unsigned int *recvBuffer;
  int init_rslt;
  GET_TIME_INIT(3);

  if (argc == 1) {
    printf("Usage: %s <data_size> <data_init type>\n", argv[0]);
    printf(" data_init type : random, sorted, reverse, xorshift\n");
    printf("\n");
    // Populate the fpga_info_list struct
    if (fpga_list(&info) != 0) {
      printf("Error populating fpga_info_list\n");
      exit(1);
    }
    printf("Number of devices: %d\n", info.num_fpgas);
    for (i = 0; i < info.num_fpgas; i++) {
      printf("%d: id:%d\n", i, info.id[i]);
      printf("%d: num_chnls:%d\n", i, info.num_chnls[i]);
      printf("%d: name:%s\n", i, info.name[i]);
      printf("%d: vendor id:%04X\n", i, info.vendor_id[i]);
      printf("%d: device id:%04X\n", i, info.device_id[i]);
    }
    exit(0);
  }
  
  if (argc != 3) {
    printf("Error! The number of argument is wrong.\n");
    exit(1);
  }

  // Set id, chnl, and data size
  id       = 0;
  chnl     = 0;
  if (atoi(argv[1]) == 0) {
    printf("Invalid data size: %s\n", argv[1]);
    printf("Please check the number of values to be sorted\n");
    exit(1);
  }
  numWords = atoi(argv[1]);

  // Get the device with id
  fpga = fpga_open(id);
  if (fpga == NULL) { printf("Could not get FPGA %d\n", id); exit(1); }
  
  // Reset
  fpga_reset(fpga); fprintf(stderr, "FPGA is reset");
  sleep(1);
  fprintf(stderr, ".");
  sleep(1);
  fprintf(stderr, ".");
  sleep(1);
  fprintf(stderr, ".");
  sleep(1);
  fprintf(stderr, "done!!\n");

  // Malloc the arrays
  sendBuffer = (unsigned int*)malloc(numWords<<2);
  if (sendBuffer == NULL) { printf("Could not malloc memory for sendBuffer\n"); fpga_close(fpga); exit(1); }
  
  recvBuffer = (unsigned int*)malloc(numWords<<2);
  if (recvBuffer == NULL) { printf("Could not malloc memory for recvBuffer\n"); free(sendBuffer); fpga_close(fpga); exit(1); }
  
  // Initialize the data
  for (i = 0; i < numWords; i++) recvBuffer[i] = 0;
  init_rslt = data_init(argv[2], sendBuffer, numWords);
  if (init_rslt == -1) {
    printf("data_init type is wrong\n");
    free(sendBuffer);
    free(recvBuffer);
    exit(1);
  }
  
  GET_TIME_VAL(0);

  // Send the data
  sent = fpga_send(fpga, chnl, sendBuffer, numWords, 0, 1, 25000);
  printf("words sent: %d\n", sent);
  
  GET_TIME_VAL(1);
  
  if (sent != 0) {
    // Recv the data
    recvd = fpga_recv(fpga, chnl, recvBuffer, numWords, 25000);
    printf("words recv: %d\n", recvd);
  }

  GET_TIME_VAL(2);

  // Done with device
  fpga_close(fpga);

  // Display some data
  for (i = 0; i < 20; i++) {
    printf("recvBuffer[%d]: %d\n", i, recvBuffer[i]);
  }
    
  // Check the data
  if (recvd != 0) {

    if (!strcmp(argv[2], "sorted")) {
      for (i = 0; i < recvd; i++) {
        if (recvBuffer[i] != sendBuffer[i]) {
          printf("recvBuffer[%d]: %d, expected %d\n", i, recvBuffer[i], sendBuffer[i]); break;
        }
      }
    }
    else if (!strcmp(argv[2], "reverse")) {
      for (i = 0; i < recvd; i++) {
        if (recvBuffer[i] != sendBuffer[recvd-(i+1)]) {
          printf("recvBuffer[%d]: %d, expected %d\n", i, recvBuffer[i], sendBuffer[recvd-(i+1)]); break;
        }
      }
    }
    else {
      QuickSort(sendBuffer, 0, numWords-1);
      for (i = 0; i < recvd; i++) {
        if (recvBuffer[i] != sendBuffer[i]) {
          printf("\nrecvBuffer[%d]: %08x, expected %08x\n", i, recvBuffer[i], sendBuffer[i]); break;
        }
      }
      // FILE *fp;
      // fp = fopen("result.txt", "w");
      // if (fp == NULL) { printf("fail to open\n"); free(sendBuffer); free(recvBuffer); exit(1); }
      // for (i = 0; i < recvd; i++) fprintf(fp, "%08x\n", recvBuffer[i]);
      // fclose(fp);
    }
    
    printf("send bw: %f GB/s %fms\n",
           sent*4.0/1000/1000/1000/((TIME_VAL_TO_MS(1) - TIME_VAL_TO_MS(0))/1000.0),
           (TIME_VAL_TO_MS(1) - TIME_VAL_TO_MS(0)) );
    
    printf("recv bw: %f GB/s %fms\n",
           recvd*4.0/1000/1000/1000/((TIME_VAL_TO_MS(2) - TIME_VAL_TO_MS(1))/1000.0),
           (TIME_VAL_TO_MS(2) - TIME_VAL_TO_MS(1)) );

    printf("Elapsed time: %9.3f sec\n", ((TIME_VAL_TO_MS(2) - TIME_VAL_TO_MS(0))/1000.0));
  }

  free(sendBuffer);
  free(recvBuffer);
  
  return 0;
}
