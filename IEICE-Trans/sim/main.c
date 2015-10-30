/************************************************************************/
/* Function-level Simulator of Sorting Accelerator  ArchLab. TOKYO TECH */
/*                                                           2015-04-07 */
/************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "xorshift.h"

#define L_NAME "Function-level Simulator of Sorting Accelerator Ver 1.0"
#define RANGE   100000000
#define MIN     0
#define MAX     (RANGE - 1)
#define SHOWNUM 100

#define ELEMS_PER_UNIT 16

int *data;
int DATANUM;

/** function to return the current time                              **/
/**********************************************************************/
long long get_time() {
  struct timeval  tp;
  struct timezone tz;
  gettimeofday(&tp, &tz);
  return tp.tv_sec * 1000000ull + tp.tv_usec;
}

/**********************************************************************/
int *temp;
void MergeSort(int sortdata[], int left, int right) {

  if (left >= right) return;

  int mid, i, j, k;
  
  mid = (left + right) / 2;
  MergeSort(sortdata, left, mid);
  MergeSort(sortdata, mid + 1, right);
  
  for (i = left; i <= mid; i++)                temp[i] = sortdata[i];
  for (i=mid+1, j=right; i <= right; i++, j--) temp[i] = sortdata[j];
  
  i = left;  
  j = right; 
  for (k = left; k <= right; k++) {
    if (temp[i] <= temp[j]) sortdata[k] = temp[i++];
    else                    sortdata[k] = temp[j--];
  }
}

/**********************************************************************/
int GetRandom(int min, int max) {
  return min + (int)(rand()*(max-min+1.0)/(1.0+RAND_MAX));
}

/**********************************************************************/
void data_init(char type[]) {
  int i;
  srand(1);

  if      (!strcmp(type, "random"))   for (i = 0; i < DATANUM; i++) data[i] = GetRandom(MIN, MAX);
  else if (!strcmp(type, "sorted"))   for (i = 0; i < DATANUM; i++) data[i] = i;
  else if (!strcmp(type, "reverse"))  for (i = 0; i < DATANUM; i++) data[i] = DATANUM-i;
  else if (!strcmp(type, "xorshift")) {
    for (i = 0; i < DATANUM; i++) {
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
        // case 0 : data[i] = (xorshift00()&0x7fffffff)%65536; break;
        // case 1 : data[i] = (xorshift01()&0x7fffffff)%65536; break;
        // case 2 : data[i] = (xorshift02()&0x7fffffff); break;
        // case 3 : data[i] = (xorshift03()&0x7fffffff); break;
        // case 4 : data[i] = (xorshift04()&0x7fffffff)%65536; break;
        // case 5 : data[i] = (xorshift05()&0x7fffffff); break;
        // case 6 : data[i] = (xorshift06()&0x7fffffff)%65536; break;
        // case 7 : data[i] = (xorshift07()&0x7fffffff); break;
        // case 8 : data[i] = (xorshift08()&0x7fffffff)%65536; break;
        // case 9 : data[i] = (xorshift09()&0x7fffffff); break;
        // case 10: data[i] = (xorshift10()&0x7fffffff)%65536; break;
        // case 11: data[i] = (xorshift11()&0x7fffffff); break;
        // case 12: data[i] = (xorshift12()&0x7fffffff)%65536; break;
        // case 13: data[i] = (xorshift13()&0x7fffffff); break;
        // case 14: data[i] = (xorshift14()&0x7fffffff)%65536; break;
        // case 15: data[i] = (xorshift15()&0x7fffffff)%65536; break;
      }
    }
  }
  else                               { printf("data_init type is wrong.\n"); exit(1); }
}

/**********************************************************************/
int *buf;
int main(int argc, char *argv[]) {

  if (argc == 1) {
    printf("%s\n", L_NAME);
    printf("usage: ./sim [last_phase] [way_log] [p_log] [sim_type] [data_init type]\n");
    printf(" data_init type : random, sorted, reverse, xorshift\n");
    exit(1);
  }

  if (argc != 6) {
    printf("Error! The number of argument is wrong.\n");
    exit(1);
  }

  int LAST_PHASE = atoi(argv[1]);
  int WAY_LOG    = atoi(argv[2]);
  int SORT_WAY   = (1 << atoi(argv[2]));
  int P_NUM      = (1 << atoi(argv[3]));
  int sim_type   = atoi(argv[4]);
  DATANUM        = ((1<<((LAST_PHASE+1)*WAY_LOG))*ELEMS_PER_UNIT);
  int memblocks  = (DATANUM >> 4);
  
  if (sim_type >= 3) {
    printf("Error! sim_type is wrong.\n");
    exit(1);
  }
  
  data    = (int*)malloc(sizeof(int)*DATANUM);
  temp    = (int*)malloc(sizeof(int)*DATANUM);
  buf     = (int*)malloc(sizeof(int)*(DATANUM/P_NUM)); // buffer for a single tree
  
  // long long start;
  // long long end;

  /* ----- Data Initialization ----- */
  printf("# sort elements n = %d\n", DATANUM);
  printf("# memory blocks   = %d\n", memblocks);
  data_init(argv[5]);
  // for (i = 0; i < SHOWNUM; i++) printf("%d ", data[i]);
  // printf("\n");

  /* ----- Simulate Sort Process ----- */

  int i, j, k;
  int phase;    
  int buf_idx;  // the index of buf
  int ecnt_mb;  // # of elements before merged in a phase (begin)
  int ecnt_me;  // # of elements after merged in a phase (end)
  int p_units;  // # of units per a way of a tree
  int offset;
  int p_i;
  int p_raddr;  // the initial address of a tree
  int datanum_p_way = (DATANUM/P_NUM/SORT_WAY);  // # of elements per a way of a tree
  int cbcnt;    // counter of compressible blocks
  int cbflag0, cbflag1;   // first data is compressible
  float cb_aprnc_ratio;  // appearance ratio of compressible blocks

  for (phase = 0; phase < LAST_PHASE+1; phase++) {
    printf("Phase %d ==============================================================\n", phase);
    cbcnt  = 0;
    
    if (phase == LAST_PHASE) {
      MergeSort(data, 0, DATANUM-1);
      
      switch (sim_type) {
        case 0: 
          cbflag0 = 0;
          for (i = 0; i < DATANUM; i++) {
            if (i%16 == 15) {
              if ((data[i]-data[i-1]<=0x1fff) && (data[i-1]-data[i-2]<=0x1fff) && (data[i-2]-data[i-3]<=0x1fff) && (data[i-3]-data[i-4]<=0x1fff) &&
                  (data[i-4]-data[i-5]<=0x1fff) && (data[i-5]-data[i-6]<=0x1fff) && (data[i-6]-data[i-7]<=0x1fff) && (data[i-7]-data[i-8]<=0x1fff) &&
                  (data[i-8]-data[i-9]<=0x1fff) && (data[i-9]-data[i-10]<=0x1fff) && (data[i-10]-data[i-11]<=0x1fff) && (data[i-11]-data[i-12]<=0x1fff) &&
                  (data[i-12]-data[i-13]<=0x1fff) && (data[i-13]-data[i-14]<=0x1fff) && (data[i-14]-data[i-15]<=0x1fff)) {
                if (cbflag0) {cbcnt++; cbflag0 = 0;}
                else {                 cbflag0 = 1;}
              } else {
                cbflag0 = 0;
              }
            }
          }
          cb_aprnc_ratio = ((float)cbcnt/(float)(memblocks>>1));
          printf("(compressible blocks, all blocks): (%d, %d)\n", cbcnt, memblocks>>1);
          printf("compression ratio: %1.3f (%3.2f%%)\n", 1.0+(2.0-1.0)*cb_aprnc_ratio, cb_aprnc_ratio*100.0);
          break;
        case 1:
          cbflag0 = 0;
          cbflag1 = 0;
          for (i = 0; i < DATANUM; i++) {
            if (i%16 == 15) {
              if (data[i]-data[i-15] <= 0x1ff) {
                if      (cbflag1)                    {cbcnt++; cbflag0 = 0; cbflag1 = 0;}
                else if (cbflag0)                    {                      cbflag1 = 1;}
                else if (data[i]-data[i-15] <= 0xff) {                      cbflag0 = 1;}
              } else {
                cbflag0 = 0;
                cbflag1 = 0;
              }
            }
          }
          cb_aprnc_ratio = ((float)cbcnt/(float)(memblocks/3));
          printf("(compressible blocks, all blocks): (%d, %d)\n", cbcnt, memblocks/3);
          printf("compression ratio: %1.3f (%3.2f%%)\n", 1.0+(3.0-1.0)*cb_aprnc_ratio, cb_aprnc_ratio*100.0);
          break;
        case 2:
          for (i = 0; i < DATANUM; i++) printf("%d ", data[i]);
          break;
      }
      
    } else {
      ecnt_mb = (ELEMS_PER_UNIT << (phase * WAY_LOG));
      ecnt_me = ((ELEMS_PER_UNIT << WAY_LOG) << (phase * WAY_LOG));
      p_units = datanum_p_way / ecnt_mb;
      
      // operate the following process as required (equal to # of trees)
      for (p_i = 0; p_i < P_NUM; p_i++) {
        p_raddr = (DATANUM/P_NUM)*p_i;
        buf_idx = 0;
        
        // copy data required to be merged to buf
        for (k = 0, offset = 0; k < p_units; k++, offset+=ecnt_mb) {
          for (j = 0; j < SORT_WAY; j++) {
            for (i = datanum_p_way*j+offset+p_raddr;
                 i < ((datanum_p_way*j+offset+p_raddr)+ecnt_mb);
                 i++, buf_idx++) {
              buf[buf_idx] = data[i];
            }
          }
        }
        
        // data is merged(sorted)
        for (i = 0; i < (DATANUM/P_NUM)/ecnt_me; i++) {
          MergeSort(buf, i*ecnt_me, (i+1)*ecnt_me-1);
        }
        
        // overwrite data[i] and count # of compressible blocks
        switch (sim_type) {
          case 0:
            cbflag0 = 0;
            for (i = 0; i < (DATANUM/P_NUM); i++) {
              data[i+p_raddr] = buf[i];
              if (i%16 == 15) {
                if ((data[i]-data[i-1]<=0x1fff) && (data[i-1]-data[i-2]<=0x1fff) && (data[i-2]-data[i-3]<=0x1fff) && (data[i-3]-data[i-4]<=0x1fff) &&
                    (data[i-4]-data[i-5]<=0x1fff) && (data[i-5]-data[i-6]<=0x1fff) && (data[i-6]-data[i-7]<=0x1fff) && (data[i-7]-data[i-8]<=0x1fff) &&
                    (data[i-8]-data[i-9]<=0x1fff) && (data[i-9]-data[i-10]<=0x1fff) && (data[i-10]-data[i-11]<=0x1fff) && (data[i-11]-data[i-12]<=0x1fff) &&
                    (data[i-12]-data[i-13]<=0x1fff) && (data[i-13]-data[i-14]<=0x1fff) && (data[i-14]-data[i-15]<=0x1fff)) {
                  if (cbflag0) {cbcnt++; cbflag0 = 0;}
                  else {                 cbflag0 = 1;}
                } else {
                  cbflag0 = 0;
                }
              }
            }
            break;
          case 1:
            cbflag0 = 0;
            cbflag1 = 0;
            for (i = 0; i < (DATANUM/P_NUM); i++) {
              data[i+p_raddr] = buf[i];
              if (i%16 == 15) {
                if (buf[i]-buf[i-15] <= 0x1ff) {
                  if      (cbflag1)                  {cbcnt++; cbflag0 = 0; cbflag1 = 0;}
                  else if (cbflag0)                  {                      cbflag1 = 1;}
                  else if (buf[i]-buf[i-15] <= 0xff) {                      cbflag0 = 1;}
                } else {
                  cbflag0 = 0;
                  cbflag1 = 0;
                }
              }
            }
            break;
          case 2:
            for (i = 0; i < (DATANUM/P_NUM); i++) {
              data[i+p_raddr] = buf[i];
            }
            break;
        }
      }

      switch (sim_type) {
        case 0:
          cb_aprnc_ratio = ((float)cbcnt/(float)(memblocks>>1));
          printf("(compressible blocks, all blocks): (%d, %d)\n", cbcnt, memblocks>>1);
          printf("compression ratio: %1.3f (%3.2f%%)\n", 1.0+(2.0-1.0)*cb_aprnc_ratio, cb_aprnc_ratio*100.0);
          break;
        case 1:
          cb_aprnc_ratio = ((float)cbcnt/(float)(memblocks/3));
          printf("(compressible blocks, all blocks): (%d, %d)\n", cbcnt, memblocks/3);
          printf("compression ratio: %1.3f (%3.2f%%)\n", 1.0+(3.0-1.0)*cb_aprnc_ratio, cb_aprnc_ratio*100.0);
          break;
        case 2:
          // display merged data in a phase
          for (i = 0; i < DATANUM; i++) {
            printf("%d ", data[i]);
            if (i % ecnt_me == ecnt_me-1) printf("\n");
            if (i % (DATANUM/P_NUM) == (DATANUM/P_NUM)-1) printf("\n");
          }
          break;
      }
    }
    printf("\n");    
  }

  free(data);
  free(temp);
  free(buf);
  
  return 0;
}
/**********************************************************************/
