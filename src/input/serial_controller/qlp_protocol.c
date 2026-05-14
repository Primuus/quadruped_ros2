#include "qlp_protocol.h"

static void Qlp_ResetParserState(QlpParser *parser)
{
  /* 回到空闲态：等待下一个 SOF */
  parser->state = QLP_PARSE_IDLE;
  parser->index = 0U;
  parser->data_len = 0U;
}

void Qlp_ParserInit(QlpParser *parser)
{
  if (parser == NULL)
  {
    return;
  }

  parser->addr = 0U;
  parser->src = 0U;
  parser->fc = 0U;
  parser->len = 0U;
  parser->data = NULL;

  Qlp_ResetParserState(parser);
}

uint16_t Qlp_Crc16(const uint8_t *data, uint16_t len)
{
  /* CRC-16/MODBUS：实现短小，适合裸机移植 */
  uint16_t crc = 0xFFFFU;

  for (uint16_t i = 0U; i < len; ++i)
  {
    crc ^= data[i];
    for (uint8_t j = 0U; j < 8U; ++j)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

uint16_t Qlp_Pack(uint8_t *out, uint16_t out_size, const QlpParser *frame)
{
  /*
   * 打包成：
   * [0]=SOF '#'，
   * [1]=ADDR [2]=SRC [3]=FC [4..5]=LEN(le)
   * [6..]=DATA
   * [end-2..end-1]=CRC16(le)，CRC 覆盖 out[1..(6+LEN-1)]
   */
  uint16_t crc;
  uint16_t total_len;

  if ((out == NULL) || (frame == NULL))
  {
    return 0U;
  }

  if (frame->len > QLP_MAX_DATA_LEN)
  {
    return 0U;
  }

  total_len = (uint16_t)(QLP_OVERHEAD_SIZE + frame->len);
  if (out_size < total_len)
  {
    return 0U;
  }

  out[0] = QLP_SOF;
  out[1] = frame->addr;
  out[2] = frame->src;
  out[3] = frame->fc;
  Qlp_WriteU16LE(&out[4], frame->len);

  for (uint16_t i = 0U; i < frame->len; ++i)
  {
    out[6U + i] = frame->data[i];
  }

  crc = Qlp_Crc16(&out[1], (uint16_t)(5U + frame->len));
  Qlp_WriteU16LE(&out[6U + frame->len], crc);

  return total_len;
}

int Qlp_ParseByte(QlpParser *parser, uint8_t byte)
{
  /*
   * 逐字节解析状态机：
   * - 任意时刻遇到 SOF('#') 立即重同步，认为是新帧起点（简单高效）
   * - 先收满固定头(6)得出 LEN，再收 DATA，再收 CRC16
   * - CRC 校验通过才输出一帧
   */
  uint16_t frame_len;
  uint16_t crc_recv;
  uint16_t crc_calc;

  if (parser == NULL)
  {
    return 0;
  }

  if (byte == QLP_SOF)
  {
    /* 重同步：把当前字节当作帧头，丢弃之前未完成的数据 */
    parser->buffer[0] = byte;
    parser->state = QLP_PARSE_HEADER;
    parser->index = 1U;
    parser->data_len = 0U;
    return 0;
  }

  if (parser->state == QLP_PARSE_IDLE)
  {
    /* 空闲态只等 SOF */
    return 0;
  }

  if (parser->index >= QLP_MAX_FRAME_SIZE)
  {
    /* 防止越界（理论上不应发生），直接复位 */
    Qlp_ResetParserState(parser);
    return 0;
  }

  parser->buffer[parser->index++] = byte;

  if ((parser->state == QLP_PARSE_HEADER) && (parser->index >= QLP_HEADER_SIZE))
  {
    /* 头部收齐，读取 LEN（小端） */
    parser->data_len = Qlp_ReadU16LE(&parser->buffer[4]);
    if (parser->data_len > QLP_MAX_DATA_LEN)
    {
      /* 长度非法，丢弃 */
      Qlp_ResetParserState(parser);
      return 0;
    }

    parser->state = (parser->data_len == 0U) ? QLP_PARSE_CRC : QLP_PARSE_DATA;
  }

  if ((parser->state == QLP_PARSE_DATA) &&
      (parser->index >= (uint16_t)(QLP_HEADER_SIZE + parser->data_len)))
  {
    parser->state = QLP_PARSE_CRC;
  }

  frame_len = (uint16_t)(QLP_OVERHEAD_SIZE + parser->data_len);
  if ((parser->state == QLP_PARSE_CRC) && (parser->index >= frame_len))
  {
    crc_recv = Qlp_ReadU16LE(&parser->buffer[frame_len - QLP_CRC_SIZE]);
    /* CRC 覆盖 ADDR..DATA：总长度减去 SOF(1) 和 CRC(2) */
    crc_calc = Qlp_Crc16(&parser->buffer[1], (uint16_t)(frame_len - 3U));
    if (crc_recv != crc_calc)
    {
      /* CRC 不通过直接丢弃（不做纠错/重传策略） */
      Qlp_ResetParserState(parser);
      return 0;
    }

    /*
     * 输出一帧：结果写回 parser 自身。
     * 注意：parser->data 指向 parser->buffer 内存，下一次成功解析会覆盖 buffer。
     */
    parser->addr = parser->buffer[1];
    parser->src = parser->buffer[2];
    parser->fc = parser->buffer[3];
    parser->len = parser->data_len;
    parser->data = &parser->buffer[6];
    return 1;
  }

  return 0;
}

void Qlp_ParseBytes(QlpParser *parser,
                    uint8_t *data,
                    uint16_t len,
                    QlpFrameHandler handler)
{
  if ((parser == NULL) || (data == NULL) || (handler == NULL))
  {
    return;
  }

  for (uint16_t i = 0U; i < len; ++i)
  {
    if (Qlp_ParseByte(parser, data[i]) != 0)
    {
      handler((QlpParser *)parser);
			/* 只复位解析状态，不清空帧字段，便于上层在回调/返回后读取 */
			Qlp_ResetParserState(parser);
    }
  }
}
