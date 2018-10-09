
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/epoll.h>
#include <sys/time.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>


struct bufctl{
	int16_t  pkt_ID;	
	int16_t  pkt_length;/*Not  Single  MultiBegin  MultiCtn  MultiEnd */
	int pkt_fragcnt;/*多包验签时的包总数*/
	int pkt_fragidx;	/*多包验签时的包ID*/
	short pkt_keyno;		/*密钥号*/
	short pkt_oper;		/*8548实施的加工操作，第一bit为0表示内网请求，为1表示外网请求*/
	short pkt_dtefd;		/*dte socket*/
	short pkt_dcefd;		/*dce socket*/
	union{
		char  	data[8000];
		int		value;
	}ctlu;			/*数据内容*/
	//签名值赋予每包的最后4byte（无需验签时此位仍需附加）
};

static char buffer[8192];
static char rebuffer[8192];

static  struct timeval start;
static  struct timeval end;

#define ST_WAIT	0
#define ST_ESTAB 	1
#define ST_ERR	2

static const unsigned int crc_32_tab_PL[] = { /* CRC polynomial */
0x00000000, 0x1621ca79, 0x2c4394f2, 0x3a625e8b, 0x588729e4, 0x4ea6e39d, 0x74c4bd16, 0x62e5776f,
0xb10e53c8, 0xa72f99b1, 0x9d4dc73a, 0x8b6c0d43, 0xe9897a2c, 0xffa8b055, 0xc5caeede, 0xd3eb24a7,
0x743d6de9, 0x621ca790, 0x587ef91b, 0x4e5f3362, 0x2cba440d, 0x3a9b8e74, 0x00f9d0ff, 0x16d81a86,
0xc5333e21, 0xd312f458, 0xe970aad3, 0xff5160aa, 0x9db417c5, 0x8b95ddbc, 0xb1f78337, 0xa7d6494e,
0xe87adbd2, 0xfe5b11ab, 0xc4394f20, 0xd2188559, 0xb0fdf236, 0xa6dc384f, 0x9cbe66c4, 0x8a9facbd,
0x5974881a, 0x4f554263, 0x75371ce8, 0x6316d691, 0x01f3a1fe, 0x17d26b87, 0x2db0350c, 0x3b91ff75,
0x9c47b63b, 0x8a667c42, 0xb00422c9, 0xa625e8b0, 0xc4c09fdf, 0xd2e155a6, 0xe8830b2d, 0xfea2c154,
0x2d49e5f3, 0x3b682f8a, 0x010a7101, 0x172bbb78, 0x75cecc17, 0x63ef066e, 0x598d58e5, 0x4fac929c,
0xc6d47ddd, 0xd0f5b7a4, 0xea97e92f, 0xfcb62356, 0x9e535439, 0x88729e40, 0xb210c0cb, 0xa4310ab2,
0x77da2e15, 0x61fbe46c, 0x5b99bae7, 0x4db8709e, 0x2f5d07f1, 0x397ccd88, 0x031e9303, 0x153f597a,
0xb2e91034, 0xa4c8da4d, 0x9eaa84c6, 0x888b4ebf, 0xea6e39d0, 0xfc4ff3a9, 0xc62dad22, 0xd00c675b,
0x03e743fc, 0x15c68985, 0x2fa4d70e, 0x39851d77, 0x5b606a18, 0x4d41a061, 0x7723feea, 0x61023493,
0x2eaea60f, 0x388f6c76, 0x02ed32fd, 0x14ccf884, 0x76298feb, 0x60084592, 0x5a6a1b19, 0x4c4bd160,
0x9fa0f5c7, 0x89813fbe, 0xb3e36135, 0xa5c2ab4c, 0xc727dc23, 0xd106165a, 0xeb6448d1, 0xfd4582a8,
0x5a93cbe6, 0x4cb2019f, 0x76d05f14, 0x60f1956d, 0x0214e202, 0x1435287b, 0x2e5776f0, 0x3876bc89,
0xeb9d982e, 0xfdbc5257, 0xc7de0cdc, 0xd1ffc6a5, 0xb31ab1ca, 0xa53b7bb3, 0x9f592538, 0x8978ef41,
0x9b8931c3, 0x8da8fbba, 0xb7caa531, 0xa1eb6f48, 0xc30e1827, 0xd52fd25e, 0xef4d8cd5, 0xf96c46ac,
0x2a87620b, 0x3ca6a872, 0x06c4f6f9, 0x10e53c80, 0x72004bef, 0x64218196, 0x5e43df1d, 0x48621564,
0xefb45c2a, 0xf9959653, 0xc3f7c8d8, 0xd5d602a1, 0xb73375ce, 0xa112bfb7, 0x9b70e13c, 0x8d512b45,
0x5eba0fe2, 0x489bc59b, 0x72f99b10, 0x64d85169, 0x063d2606, 0x101cec7f, 0x2a7eb2f4, 0x3c5f788d,
0x73f3ea11, 0x65d22068, 0x5fb07ee3, 0x4991b49a, 0x2b74c3f5, 0x3d55098c, 0x07375707, 0x11169d7e,
0xc2fdb9d9, 0xd4dc73a0, 0xeebe2d2b, 0xf89fe752, 0x9a7a903d, 0x8c5b5a44, 0xb63904cf, 0xa018ceb6,
0x07ce87f8, 0x11ef4d81, 0x2b8d130a, 0x3dacd973, 0x5f49ae1c, 0x49686465, 0x730a3aee, 0x652bf097,
0xb6c0d430, 0xa0e11e49, 0x9a8340c2, 0x8ca28abb, 0xee47fdd4, 0xf86637ad, 0xc2046926, 0xd425a35f,
0x5d5d4c1e, 0x4b7c8667, 0x711ed8ec, 0x673f1295, 0x05da65fa, 0x13fbaf83, 0x2999f108, 0x3fb83b71,
0xec531fd6, 0xfa72d5af, 0xc0108b24, 0xd631415d, 0xb4d43632, 0xa2f5fc4b, 0x9897a2c0, 0x8eb668b9,
0x296021f7, 0x3f41eb8e, 0x0523b505, 0x13027f7c, 0x71e70813, 0x67c6c26a, 0x5da49ce1, 0x4b855698,
0x986e723f, 0x8e4fb846, 0xb42de6cd, 0xa20c2cb4, 0xc0e95bdb, 0xd6c891a2, 0xecaacf29, 0xfa8b0550,
0xb52797cc, 0xa3065db5, 0x9964033e, 0x8f45c947, 0xeda0be28, 0xfb817451, 0xc1e32ada, 0xd7c2e0a3,
0x0429c404, 0x12080e7d, 0x286a50f6, 0x3e4b9a8f, 0x5caeede0, 0x4a8f2799, 0x70ed7912, 0x66ccb36b,
0xc11afa25, 0xd73b305c, 0xed596ed7, 0xfb78a4ae, 0x999dd3c1, 0x8fbc19b8, 0xb5de4733, 0xa3ff8d4a,
0x7014a9ed, 0x66356394, 0x5c573d1f, 0x4a76f766, 0x28938009, 0x3eb24a70, 0x04d014fb, 0x12f1de82 
};

static void printuchar(const void *ptr, int len, const char *tick)
{
      const unsigned char *p = (const unsigned char *)ptr;

      printf("%s", tick);
      for(int i=0; i<len; i++){
            printf("%0#2x ", p[i]);
      }
      printf("\n");
           
      return;
}

static uint32_t crc32(uint32_t init_crc, const void *in, int inlen)
{
          const unsigned char *data = in;
	uint32_t crc = init_crc;
	
	for(int i = 0; i < inlen; i++)	{
		crc = (crc<<8) ^ crc_32_tab_PL[(crc>>24)^data[i]];
	}
	return crc;
}

static uint32_t convert_ptr_u32_le(const void *ptr)
{
      uint32_t ret = 0;
      const unsigned char *in_ptr = ptr;
      ret |= in_ptr[0];
      ret |= in_ptr[1]<<8;
      ret |= in_ptr[2]<<16;
      ret |= in_ptr[3]<<24;
      return ret;  
}

int main(int argc,char* argv[])
{
      struct epoll_event ev,events[4];
	int i;
      int epfd = epoll_create(256);
      printf("epfd = %d\n",epfd);

      int opfd = open("/dev/pcie56",O_RDWR);
	unsigned long  speed;
      if(opfd < 0){
            perror("open pipe error");
            return -1;
      }

      ev.data.fd = opfd;
      //ev.events = EPOLLOUT;
      ev.events = EPOLLIN;
      epoll_ctl(epfd, EPOLL_CTL_ADD, opfd, &ev);

      int state = ST_WAIT;
      int writeTimes = 0;
      int readTimes = 0;
      
      memset(rebuffer, 0, 8192);
      srand(time(NULL));
      
      gettimeofday(&start, NULL);
	for(i=0;i<4096;i++)
		buffer[i] = i&0xff;

      while(1){
            int nfds = epoll_wait(epfd, events, 4, 10000);

            //printf("testrev nfds = %d\n",nfds);
	
            for(int  i = 0 ; i < nfds ; i++ ){
                  int fd = events[i].data.fd;
#if 0	

                  if(events[i].events & EPOLLOUT){
                        int count = rand() %12;
                        //printf("count = %d, events %0#x\n", events[i].events );
                  
                        if( state == ST_WAIT){
                              state = ST_ESTAB;
                              ev.data.fd = opfd;
                              ev.events = EPOLLIN|EPOLLOUT;
                              epoll_ctl(epfd, EPOLL_CTL_MOD, fd, &ev);
                              continue;
                        }
			
                        for(int i = 0; i < count ; i++){
                              struct bufctl *cb = (struct bufctl*)buffer;
                             
                              #define MAX_PACKETLEN  (4088-32)
                              #define MIN_PACKETLEN  32
                              int packet_size = (int)((double)rand()/RAND_MAX*(MAX_PACKETLEN - MIN_PACKETLEN) + MIN_PACKETLEN);
                              //int packet_size = 3840;

                              cb->pkt_ID = 0;
                              cb->pkt_length= 128;
                              cb->pkt_fragidx = writeTimes;
                              cb->pkt_keyno = 0;
                              cb->pkt_fragcnt = rand();
                              
                             // uint32_t crc_calc = crc32(0, buffer, packet_size -4);
                            //  for(int i = 0; i < 4; i++){
                             //       buffer[packet_size-4+i] = (crc_calc >>(i*8)) & 0x0ff;
                            //  }
                              
                              int ret = write(fd, cb, cb->pkt_length);                                                          
                              //printf("write() return %d, writeTimes = %d\n", ret, writeTimes);
                              if( ret < 0 ){
                                    if( errno == EAGAIN || errno == ENOMEM ){
                                          continue;
                                    }
                                    perror("write failed");
                                    exit(1);
                              }
                              
                              //char tick[18] = "";
                              //snprintf(tick, 18, "%d: ", writeTimes);
                              //printuchar(buffer, 32, tick);
                              writeTimes++;

                              if( writeTimes >= 5000000){
                                    printf("write finish..........\n");
                                    ev.data.fd = opfd;
                                    ev.events = EPOLLOUT;
                                    epoll_ctl(epfd, EPOLL_CTL_MOD, fd, &ev);
                                    break;
                              }
                        }
                  }
             
#endif 
       	if(events[i].events & EPOLLIN){
                        //printf("begin to EPOLLIN\n");
                        int count = rand()%12;
				
                        for(int j = 0; j < count; j++){

                              int ret = read(fd, rebuffer, 4096);                              
                              //printf("read() return %d\n", ret);

                              if( ret < 0){
                                    if(errno == EAGAIN || errno == EWOULDBLOCK){
                                          continue;
                                    }
                                    printf("errno = %d, readError !\n", errno);
                                    exit(1);
                              }

                              struct bufctl  *rcb = (struct bufctl*)rebuffer;
                              //printuchar(rebuffer, 32);
                                                                  
                              if((rcb->pkt_fragidx != readTimes) || (ret < rcb->pkt_length)){
                                    printf("read() return %d, rcb->pkt_length = %d, rcb->pkt_fragidx = %d, readTimes = %d\n", 
                                          ret, rcb->pkt_length, rcb->pkt_fragidx, readTimes);
                                    printuchar(rebuffer, 32, "err_packet: ");
                                    exit(1);
                              }
					if(readTimes==0)
						gettimeofday(&start, NULL);
                              readTimes ++;

                              uint32_t crc_calc = crc32(0, rebuffer, rcb->pkt_length -4);
                              uint32_t crc_val = convert_ptr_u32_le(rebuffer + rcb->pkt_length -4);
                              if(crc_calc != crc_val){
                                    printf("idx %d: crc wrong, crc_calc = %0#x, crc_val = %0#x\n", rcb->pkt_fragidx, crc_calc, crc_val);
                                    exit(1);
                              }
					//printf("readTimes %d\n", readTimes);			  
                              if(readTimes % 10000 == 0){
                                    printf("readTimes %d, crc_calc = %0#x, crc_val = %0#x\n", readTimes, crc_calc, crc_val);
                              }
                              if((readTimes % 10000 == 0) || (readTimes > 5000000)){
                                    gettimeofday(&end, NULL);
                                    printf("recv %d packets, last 10000 packets use %ld sec,%ld usec\n", readTimes,
                                          end.tv_sec - start.tv_sec, end.tv_usec - start.tv_usec);
						speed = ( (rcb->pkt_length*8*10000)/((end.tv_sec-start.tv_sec)*1000000+end.tv_usec - start.tv_usec))*1000000/1024/1024;
					  	 printf("speed is %ld Mbps\n",speed);
                                    gettimeofday(&start, NULL);
                              }
                        }
                  }
			 

            }
            //printf("end loop!\n");
      }
      return 0;
}

