#ifndef QLP_PROTOCOL_H
#define QLP_PROTOCOL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>
#include <stdint.h>

/*
 * 帧格式（小端）：
 * | SOF(1) | ADDR(1) | SRC(1) | FC(1) | LEN(2) | DATA(N) | CRC16(2) |
 *
 * 约定：
 * - SOF 固定为 '#'(0x23)，用于快速找帧起点/重同步
 * - LEN 为 DATA 字节数，小端
 * - CRC16 覆盖范围：ADDR..DATA（不含 SOF，不含 CRC 本身）
 */
#define QLP_SOF 0x23U
#define QLP_HEADER_SIZE 6U
#define QLP_CRC_SIZE 2U
#define QLP_OVERHEAD_SIZE (QLP_HEADER_SIZE + QLP_CRC_SIZE)
#define QLP_MAX_DATA_LEN 256U
#define QLP_MAX_FRAME_SIZE (QLP_OVERHEAD_SIZE + QLP_MAX_DATA_LEN)

  typedef enum
  {
    QLP_PARSE_IDLE = 0,
    QLP_PARSE_HEADER,
    QLP_PARSE_DATA,
    QLP_PARSE_CRC
  } QlpParseState;

  /*
   * 合并后的协议对象：
   * - addr/src/fc/len/data 保存最近一次成功解析出的帧
   * - state/index/data_len/buffer 保存解析状态
   */
  typedef struct
  {
    uint8_t addr;
    uint8_t src;
    uint8_t fc;
    uint16_t len;
    const uint8_t *data;

    QlpParseState state;                // 数据接收状态
    uint16_t index;                     // 全部数据临时序号
    uint16_t data_len;                  // 不带头尾的长度
    uint8_t buffer[QLP_MAX_FRAME_SIZE]; // 全部数据存放
  } QlpParser;

  void Qlp_ParserInit(QlpParser *parser);

  /* CRC16: 选用常见的 CRC-16/MODBUS（poly=0xA001, init=0xFFFF） */
  uint16_t Qlp_Crc16(const uint8_t *data, uint16_t len);

  /* 打包：写入 out，返回帧总长度；失败返回 0 */
  uint16_t Qlp_Pack(uint8_t *out, uint16_t out_size, const QlpParser *frame);

  /* 解析：逐字节喂入；当凑齐并校验通过时返回 1，结果保存在 parser 自身 */
  int Qlp_ParseByte(QlpParser *parser, uint8_t byte);

  /* 回调：解析出一帧时通知上层 */
  typedef void (*QlpFrameHandler)(QlpParser *frame);

  /* 解析：一次处理一段字节流（适合 DMA/ReceiveToIdle） */
  void Qlp_ParseBytes(QlpParser *parser,
                      uint8_t *data,
                      uint16_t len,
                      QlpFrameHandler handler);

  static inline void Qlp_WriteU16LE(uint8_t *buf, uint16_t value)
  {
    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
  }

  static inline uint16_t Qlp_ReadU16LE(const uint8_t *buf)
  {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
  }

  static inline void Qlp_WriteFloatLE(uint8_t *buf, float value)
  {
    /* float32 按 IEEE-754 原始字节写入，小端 */
    union
    {
      float f;
      uint8_t b[4];
    } conv;

    conv.f = value;
    buf[0] = conv.b[0];
    buf[1] = conv.b[1];
    buf[2] = conv.b[2];
    buf[3] = conv.b[3];
  }

  static inline float Qlp_ReadFloatLE(const uint8_t *buf)
  {
    /* float32 按 IEEE-754 原始字节读取，小端 */
    union
    {
      float f;
      uint8_t b[4];
    } conv;

    conv.b[0] = buf[0];
    conv.b[1] = buf[1];
    conv.b[2] = buf[2];
    conv.b[3] = buf[3];
    return conv.f;
  }

#ifdef __cplusplus
}
#endif

#endif
