
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>

#include <stdio.h>
#include <errno.h>
#include <limits.h>

char read_buffer[4096];

int main(void)
{
      int count = 0;
      int retry = 0; 
      
      int opfd = open("/dev/pcie560",O_RDWR);
      if(opfd < 0){
            printf("open fd error!\n");
            return -1;
      }

      while(1){
            int ret = read(opfd, read_buffer, 4096);
            if(ret < 0){
                  if(errno == EAGAIN || errno == EWOULDBLOCK){
                        retry++;
                        if(retry  == INT_MAX>>8){
                              printf("return EAGAIN %d times\n", retry);
                              retry = 0;
                        }
                        continue;
                  }
                  perror("read error");
                  break;
            }
            count ++;
          //  printf("\rread %d packets", count);
         	 printf("\rread %d packets  num is %d", count,*(unsigned int *)(&read_buffer[4]));
            fflush(stdout);
      }
      return 0;
}



