#ifndef IPROTCLCOMMON_H
#define IPROTCLCOMMON_H

/*
 *作用：协议处理公共库相关接口定义
 *作者：张心臻
 *创建日期：2014-01-15
*/

#include "protclType.h"

namespace ptclcomm{
class ptlcUtil{
public:
    //UINT16转换为BYTE数组
    static void UINT16ToArray(UINT16 i, BYTE* out );

    //UINT32转换为BYTE数组
    static void UINT32ToArray(UINT32 i, BYTE* out);

    //BYTE数组转换为UINT16
    static UINT16 Decode_UINT16(BYTE * in);

    //BYTE数组转换为UINT32
    static UINT32 Decode_UINT32(BYTE * in);

    //动态创建数组
    static BYTE* NewArray( int len );

    //删除数组内存
    static void DeleteArray(BYTE *in);
};
}

#endif // IPROTCLCOMMON_H
