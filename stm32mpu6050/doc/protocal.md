# 上位机的通讯协议

## 格式
1. 帧头 HEAD 1byte 0xAA
2. 目标地址 D_ADDR 1byte 0xFF
3. 功能码 ID 1byte 0x03
4. 数据长度 LEN 7
5. 数据内容 DATA 
  5.1 ROL * 100 int16
  5.2 PIT * 100 int16
  5.3 YAW * 100 int16
  5.4 FUSION_DATA  uint8 融合状态
6. 校验和 SC 1byte
7. 附加校验 AC 1byte

## 校验和算法
从帧头0xAA开始一直到DATA区域结束，对每一个字节进行累加操作，只取低8位

## 附加校验算法
计算校验和时，每进行一字节的加法运算，同时进行一次 SUM CHECK 的累加操作，只取低 8 位。

## 校验计算示例
假设数据帧缓存为 data_buf 数组，0xAA 存放于数组起始位置，那么 data_buf[3]存放的是数据长度，校验程序
如下：
```
U8 sumcheck = 0;
U8 addcheck = 0;
For(u8 i=0; I < (data_buf[3]+4); i++)
{
  sumcheck += data_buf[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
  addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
}
//如果计算出的sumcheck和addcheck和接收到的check数据相等，代表校验通过，反之数据有误
if(sumcheck == data_buf[data_buf[3]+4] && addcheck == data_buf[data_buf[3]+5])
return true; //校验通过
else
return false; //校验失败
```

# 相关参考
1. https://blog.csdn.net/qq_30150579/article/details/139247643
2. https://github.com/creative-apple/STM32-MPU6050