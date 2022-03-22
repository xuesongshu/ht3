#include "descr.h"
#include "cyu3usbconst.h"

unsigned char g_usb30_device[] __attribute__ ((aligned(32))) = {
    0x12,                         //描述符长度
    CY_U3P_USB_DEVICE_DESCR,      //描述符类型
    0x01, 0x03,                   //USB版本
    0x00,                         //设备类
    0x00,                         //设备子类
    0x00,                         //设备协议
    0x09,                         //EP0端点小包的最大长度
    0xB4, 0x04,                   //VID
    0xF1, 0x00,                   //PID
    MINOR_VERSION, MAJOR_VERSION, //固件版本
    0x01,                         //制造商字符串
    0x02,                         //产品字符串
    0x00,                         //序列号字符串
    0x01                          //配置数量
};

unsigned char g_usb20_device[] __attribute__ ((aligned(32))) = {
    0x12,                         //描述符长度
    CY_U3P_USB_DEVICE_DESCR,      //描述符类型
    0x01, 0x02,                   //USB版本
    0x00,                         //设备类
    0x00,                         //设备子类
    0x00,                         //设备协议
    0x09,                         //EP0端点小包的最大长度
    0xB4, 0x04,                   //VID
    0xF1, 0x00,                   //PID
    MINOR_VERSION, MAJOR_VERSION, //固件版本
    0x01,                         //制造商字符串
    0x02,                         //产品字符串
    0x00,                         //序列号字符串
    0x01                          //配置数量
};

unsigned char g_bos[] __attribute__ ((aligned(32))) = {
    0x05,                        //描述符第一段长度
    CY_U3P_BOS_DESCR,            //描述符类型
    0x16, 0x00,                  //整个描述的长度
    0x02,                        //设备能力描述符数量

    0x07,                        //描述述第二段长度
    CY_U3P_DEVICE_CAPB_DESCR,    //描述符第二段类型
    CY_U3P_USB2_EXTN_CAPB_TYPE,  //USB扩展能力
    0x1E, 0x64, 0x00, 0x00,      //启用LPM、BESL、Deep BESL三个特性，BESL为400us，Deep BESL为1000us

    0x0A,                        //描述符第三段长度
    CY_U3P_DEVICE_CAPB_DESCR,    //描述符第三段类型
    CY_U3P_SS_USB_CAPB_TYPE,     //USB超速能力
    0x00,                        //设备等级特性
    0x0E, 0x00,                  //USB速度特性，0E是0x02、0x04、0x08的或运算结果
    0x03,                        //功能支持
    0x00,                        //U1退出延迟
    0x00, 0x00                   //U2退出延迟
};

unsigned char g_device_qual[] __attribute__ ((aligned(32))) = {
    0x0A,                        //描述符长度
    CY_U3P_USB_DEVQUAL_DESCR,    //描述符类型
    0x00, 0x02,                  //USB 2.0
    0x00,                        //设备类
    0x00,                        //设备子类
    0x00,                        //设备协议
    0x40,                        //EP0端点小包最大尺寸
    0x01,                        //配置数量
    0x00,                        //保留字段
};

unsigned char g_ss_config[] __attribute__ ((aligned(32))) = {
    0x09,                       //描述符长度
    CY_U3P_USB_CONFIG_DESCR,    //描述符类型
    0x2C, 0x00,                 //描述符总长度
    0x01,                       //接口数量
    0x01,                       //配置数量
    0x00,                       //配置字符串索引
    0x80,                       //总线供电特征配置
    0x32,                       //最大电流，对于USB 3.0及以上，单位是8mA，此配置表示400mA

    0x09,                       //描述符接口段长度
    CY_U3P_USB_INTRFC_DESCR,    //描述符接口段类型
    0x00,                       //接口序号
    0x00,                       //替代设置序号
    ENDPOINT_COUNT,             //端点（End point）数量
    0xFF,                       //接口类
    0x00,                       //接口子类
    0x00,                       //接口协议
    0x00,                       //接口描述字符串索引

    0x07,                       //描述符端点段长度
    CY_U3P_USB_ENDPNT_DESCR,    //描述符端点段类型
    PRODUCER1,                  //端点序号，程序以此编号识别、控制、管理端点
    CY_U3P_USB_EP_BULK,         //端点数据类型
    0x00, 0x04,                 //小包最大尺寸
    0x00,                       //数据传输服务间隔，对于块传递，此配置为0

    0x06,                       //描述符端点超速段长度
    CY_U3P_SS_EP_COMPN_DESCR,   //描述符端点超速段类型
    0x00,                       //小包一次最大突发量，实际突发量为此配置值+1
    0x00,                       //端点最大流量，0表示没有流
    0x00, 0x00,                 //端点服务间隔，0表示块（BULK）

    0x07,                       //描述符端点段长度
    CY_U3P_USB_ENDPNT_DESCR,    //描述符端点段类型
    CONSUMER1,                  //端点序号，程序以此编号识别、控制、管理端点
    CY_U3P_USB_EP_BULK,         //端点数据类型
    0x00, 0x04,                 //小包最大尺寸
    0x00,                       //数据传输服务间隔，对于块传递，此配置为0

    0x06,                       //描述符端点超速段长度
    CY_U3P_SS_EP_COMPN_DESCR,   //描述符端点超速段类型
    0x00,                       //小包一次最大突发量，实际突发量为此配置值+1
    0x00,                       //端点最大流量，0表示没有流
    0x00, 0x00                  //端点服务间隔，0表示块（BULK）
};

unsigned char g_hs_config[] __attribute__ ((aligned(32))) = {
    0x09,                       //描述符长度
    CY_U3P_USB_CONFIG_DESCR,    //描述符类型
    0x20, 0x00,                 //描述符总长度
    0x01,                       //接口数量
    0x01,                       //配置数量
    0x00,                       //配置字符串索引
    0x80,                       //总线供电特征配置
    0x32,                       //最大电流，对于USB 2.0，单位是2mA，此配置表示100mA

    0x09,                       //描述符接口段长度
    CY_U3P_USB_INTRFC_DESCR,    //描述符接口段类型
    0x00,                       //接口序号
    0x00,                       //替代设置序号
    ENDPOINT_COUNT,             //端点（End point）数量
    0xFF,                       //接口类
    0x00,                       //接口子类
    0x00,                       //接口协议
    0x00,                       //接口描述字符串索引

    0x07,                       //描述符端点段长度
    CY_U3P_USB_ENDPNT_DESCR,    //描述符端点段类型
    PRODUCER1,                  //端点序号，程序以此编号识别、控制、管理端点
    CY_U3P_USB_EP_BULK,         //端点数据类型
    0x00, 0x02,                 //小包最大尺寸
    0x00,                       //数据传输服务间隔，对于块传递，此配置为0

    0x07,                       //描述符端点段长度
    CY_U3P_USB_ENDPNT_DESCR,    //描述符端点段类型
    CONSUMER1,                  //端点序号，程序以此编号识别、控制、管理端点
    CY_U3P_USB_EP_BULK,         //端点数据类型
    0x00, 0x02,                 //小包最大尺寸
    0x00,                       //数据传输服务间隔，对于块传递，此配置为0
};

unsigned char g_fs_config[] __attribute__ ((aligned(32))) = {
    0x09,                       //描述符长度
    CY_U3P_USB_CONFIG_DESCR,    //描述符类型
    0x20, 0x00,                 //描述符总长度
    0x01,                       //接口数量
    0x01,                       //配置数量
    0x00,                       //配置字符串索引
    0x80,                       //总线供电特征配置
    0x32,                       //最大电流，对于USB 2.0，单位是2mA，此配置表示100mA

    0x09,                       //描述符接口段长度
    CY_U3P_USB_INTRFC_DESCR,    //描述符接口段类型
    0x00,                       //接口序号
    0x00,                       //替代设置序号
    ENDPOINT_COUNT,             //端点（End point）数量
    0xFF,                       //接口类
    0x00,                       //接口子类
    0x00,                       //接口协议
    0x00,                       //接口描述字符串索引

    0x07,                       //描述符端点段长度
    CY_U3P_USB_ENDPNT_DESCR,    //描述符端点段类型
    PRODUCER1,                  //端点序号，程序以此编号识别、控制、管理端点
    CY_U3P_USB_EP_BULK,         //端点数据类型
    0x40, 0x00,                 //小包最大尺寸
    0x00,                       //数据传输服务间隔，对于块传递，此配置为0

    0x07,                       //描述符端点段长度
    CY_U3P_USB_ENDPNT_DESCR,    //描述符端点段类型
    CONSUMER1,                  //端点序号，程序以此编号识别、控制、管理端点
    CY_U3P_USB_EP_BULK,         //端点数据类型
    0x40, 0x00,                 //小包最大尺寸
    0x00,                       //数据传输服务间隔，对于块传递，此配置为0
};

unsigned char g_string_lang[] __attribute__((aligned(32))) = {
    0x04,                      //描述符长度
    CY_U3P_USB_STRING_DESCR,   //描述符类型
    0x09, 0x04                 //区域语言
};

unsigned char g_manufacture[] __attribute__((aligned(32))) = {
    0x06,                     //描述符长度
    CY_U3P_USB_STRING_DESCR,  //描述符类型
    'H', 0x00,                //字串的一个字符，字的2个字节倒序存放
    'T', 0x00                 //字串的一个字符，字的2个字节倒序存放
};

unsigned char g_product[] __attribute__((aligned(32))) = {
    0x06,                     //描述符长度
    CY_U3P_USB_STRING_DESCR,  //描述符类型
    'H', 0x00,                //字串的一个字符，字的2个字节倒序存放
    'T', 0x00                 //字串的一个字符，字的2个字节倒序存放
};

//描述符对齐方式
unsigned char g_descriptor_align[32] __attribute__((aligned(32)));

unsigned char g_serial_number[] __attribute__((aligned(32))) = {
    0x1C,
    CY_U3P_USB_STRING_DESCR,
    '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0,
    '0', 0, '0', 0, '0', 0, '1', 0, '2', 0, '3', 0,
    0, 0,
};
