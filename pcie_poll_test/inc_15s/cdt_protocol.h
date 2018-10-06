#ifndef CDT_PROTOCOL_H
#define CDT_PROTOCOL_H

#include "IProtclCommon.h"

/*
  *版本定义
  */
#define CDTP_VERSION_MAIN   1
#define CDTP_VERSION_SUB   0
#define GETMVERSION(x) (((x)&0xFF00)>>8)
#define MAKEVERSION(x,y) (((x)<<8) | (y))

/*
  *长度常量定义
  */
#define CDTP_HEADER_LEN 32      //跨域传输头长度
#define CDTP_AUTHDATA_LEN 16     //认证数据长度
#define CDTP_GRPHEADER_LEN  8       //分组报文头长度
#define CDTP_LINKFAULT_LEN  8       //链路故障数据长度
#define CDTP_SIMREC_LEN 5       //简单报文回执长度
#define CDTP_GRPREC_MIN_LEN     10      //分组报文回执最小长度
#define CDTP_SIMRES_MIN_LEN     4       //简单响应报文最小长度
#define CDTP_GRPRESHEADER_LEN     12       //分组响应报文头长度

#define CDTP_INNER_HEADER_LEN   1   //内部协议头长度

#define CDTP_DATABUF_LEN 4048    //数据区缓存长度
#define CDTP_FAILPACKBUF_LEN   255     //需要重发数据包号保存缓存长度

/*
  *报文类型常量定义
  */
#define CDTP_MSGTYPE_SIMPLE 0x00    //简单报文
#define CDTP_MSGTYPE_GROUP 0x01    //分组报文
#define CDTP_MSGTYPE_LINKFAULT 0x02    //链路故障检测报文
#define CDTP_MSGTYPE_SIMACK 0x10    //简单报文回执
#define CDTP_MSGTYPE_GRPACK 0x11    //分组报文回执
#define CDTP_MSGTYPE_INNER 0xF1    //跨域传输内部协议

/*
  *业务类型常量定义
  */
#define CDTP_BSTYPE_BSMSG   0x00    //业务报文
#define CDTP_BSTYPE_HTTP    0x01    //HTTP报文


/**
 * 分组报文回执类型
 */
#define CDTP_GRPACK_ALL			0x0	// 确认数据包号前的报文均成功接收
#define CDTP_GRPACK_RESEND			0x1	//  确认数据包号前的有一批数据包需要重发

/*
  *内部报文类型
  */
#define CDTP_INNER_TYPE_LOGIN   0x01    //代理登录报文
#define CDTP_INNER_TYPE_SLINKIN   0x02    //服务连接初始报文


/*
  *error code
  */
#define CDTP_OK 0       //成功
#define CDTP_FAIL   1       //失败
#define CDTP_PARA_ERROR 2     //输入参数错误
#define CDTP_WRONG_MSGTYPE 3     //报文类型错误
#define CDTP_OUTBUF_TOO_SMALL   4   //输入缓存区过小
#define CDTP_PRO_NOMATCHED 5    //不符合协议要求
#define CDTP_WRONG_VERSION  6       //版本号不正确
#define CDTP_WRONG_MSGLEN  7       //报文长度不正确

/*
  *跨域传输协议头
  */
typedef struct CDTPTL_HEADER
{
    UINT16 version;     //版本号,协议封装时该字段填0
    UINT16 msglen;      //报文长度,协议封装时该字段填0
    BYTE msgType;       //报文类型
    BYTE businessType;  //业务类型
    BYTE msgFlag;       //报文标志,从高位开始第1位为ACK标识；第2～4位为优先级;第5位为报文响应类型,
                                //第6位表示服务是否立即响应客户端的传输请求
    BYTE reserved;      //保留
    UINT32 msgNo;       //报文编号
    UINT32 packAppId;   //组包应用标识
    UINT32 sendNodeId;  //发方节点标识
    UINT32 sendAppId;   //发送应用标识
    UINT32 recvNodeId;  //收方节点标识
    UINT32 recvAppId;   //收方应用标识
}CDTPTL_HEADER;

/*
  *简单报文
  */
typedef struct SIMPLEBODY
{
    UINT32 sdatalen;
    BYTE data[CDTP_DATABUF_LEN];
}SIMPLEBODY;

/*
  *分组报文头
  */
typedef struct GROUPHEADER
{
    UINT32 packCount;
    UINT32 packIndex;
}GROUPHEADER;

/*
  *分组报文
  */
typedef struct GROUPDATABODY
{
    GROUPHEADER grpHeader;
    UINT32 grpdatalen;
    BYTE grpdata[CDTP_DATABUF_LEN];
}GROUPDATABODY;

/*
  *简单报文回执
  */
typedef struct SIMACKBODY
{
    UINT32 srcMsgNo;       //源报文编号
    BYTE recCode;       //回执码
}SIMACKBODY;

/*
  *分组报文回执
  */
typedef struct GRPACKBODY
{
    UINT32 srcMsgNo;       //源报文编号
    UINT32 ackPackIndex;       //确认数据报号
    BYTE ackType;       //回执类型，0－确认数据包号前的报文均成功接收
                        //          1-确认数据包号前的有一批数据包需要重发
    BYTE failPackCount;    //重发数据包数量
    UINT32 failPackIndex[CDTP_FAILPACKBUF_LEN]; //重发数据包流水号
}GRPACKBODY;

/*
  *链路故障检测报文
  */
typedef struct LINKFAULTBODY
{
    UINT32  outNetIp;  //故障设备的外网IP地址
    UINT16 inOutFlag;   //内外网标识，0表示内网；1表示外网
    UINT16 faultCode;   //故障原因编码，0表示链路畅通，由跨域传输服务发送，其他错误码都有产生故障的安全接入控制设备发送
}LINKFAULTBODY;

/*
  *（内部）登录消息结构体
  */
typedef struct CDTLOGINBODY
{
    UINT32 appId;   //应用标识
}CDTLOGINBODY;

/**
 * （内部）服务间连接初始化
 */
typedef struct CDTSLINKINBODY
{
    UINT16 port;   //服务监听端口
}CDTSLINKINBODY;
/*
  *代理与服务之间的内部协议
  */
typedef struct INNERBODY
{
    BYTE innerType;     //内部报文类型
    union{
        CDTLOGINBODY login;     //代理登录报文，（即innerType==CDTP_INNER_TYPE_LOGIN）
        CDTSLINKINBODY slinkin; //服务间连接初始化报文，（即innerType==CDTP_INNER_TYPE_SLINKIN）
    };
}INNERBODY;

/*
  *跨域传输数据结构体
  */
typedef struct CDTPData
{
    CDTPTL_HEADER header;
    union {
        SIMPLEBODY sdata;     //报文类型为简单报文（即header->msgType==CDTP_MSGTYPE_SIMPLE）
        GROUPDATABODY grpdata;          //业务类型为分组报文（即header->msgType==CDTP_MSGTYPE_GROUP）
        LINKFAULTBODY linkfault;    //链路故障检测报文（即header->msgType==CDTP_MSGTYPE_LINKFAULT）
        SIMACKBODY srdata;      //业务类型为简单报文回执（即header->msgType==CDTP_MSGTYPE_SIMREC）
        GRPACKBODY grdata;      //业务类型为分组报文回执（即header->msgType==CDTP_MSGTYPE_GRPREC）
        INNERBODY inner;        //代理与服务之间的内部协议（即header->msgType==CDTP_MSGTYPE_INNER）
    }body;
    BYTE auth[CDTP_AUTHDATA_LEN];
}CDTPData;

/**
  跨域传输数据序列化
  参数：
  [in] data: 跨域传输数据结构体
  [out] outbuf: 输出数据首地址
  [in/out] outbuflen: in-输出缓存区大小;out-输出数据长度
  返回值：
  结果码，0成功，非0失败
  */
int serialize_CDTProtocol( const CDTPData* data, char* outbuf, int* outbuflen );

/**
  跨域传输数据反序列化
  参数：
  [in] inbuf: 跨域传输数据缓存首地址
  [in] inbuflen: 跨域传输数据缓存长度
  [out] out: 输出跨域传输数据结构体
  返回值：
  结果码，0成功，非0失败
  */
int deserialize_CDTProtocol( const char *inbuf, int inbuflen, CDTPData* out );

/**
  跨域传输数据反序列化,仅提取跨域传输头
  参数：
  [in] inbuf: 跨域传输数据缓存首地址
  [in] inbuflen: 跨域传输数据缓存长度
  [out] out: 输出跨域传输头
  返回值：
  结果码，0成功，非0失败
  */
int deserialize_CDTHeader( const char *inbuf, int inbuflen, CDTPTL_HEADER *out );

/**
  更新跨域传输协议头中的发方节点标识
  参数：
  [in] inbuf: 跨域传输数据缓存首地址
  [in] inbuflen: 跨域传输数据缓存长度
  [in] sendNodeId: 新的发方节点标识
  返回值：
  结果码，0成功，非0失败
  */
int update_CDTPSendNodId( char *inbuf, int inbuflen, UINT32 sendNodeId );

/**
  更新跨域传输协议头中的收方节点标识
  参数：
  [in] inbuf: 跨域传输数据缓存首地址
  [in] inbuflen: 跨域传输数据缓存长度
  [in] recvNodeId: 新的收方节点标识
  返回值：
  结果码，0成功，非0失败
  */
int update_CDTPRecvAppId( char *inbuf, int inbuflen, UINT32 recvAppId );

#endif // CDT_PROTOCOL_H

