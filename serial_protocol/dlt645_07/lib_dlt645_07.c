#include "sys.h"

/*******************************************************************************
 * 数据结构的定义
 *******************************************************************************/
enum SERIAL_STATE
{
    STATE_IDLE = 0,
    STATE_SEND = 1,
    STATE_WAIT = 2,
    STATE_MAX
};

#define MAX_CONTEXT_NUM 1
#define MAX_BUF_LEN 256

struct CMD_TIEM
{
    unsigned long long sn;      /* 8bytes to store sn */
    unsigned int data_type;     /* 4 bytes */
    unsigned short reg_address; /* 寄存器地址 */
    unsigned short
        reg_size; /* 用多少个寄存器存储数据，限定1（16位），2（32位），4（64位） */
    unsigned short time_out;
    unsigned short delay_time;
}; /* total 20bytes */

struct SERIAL_CONTEXT
{
    enum SERIAL_STATE state;
    int timer_id;
    int serial_id;
    int cmd_id;
    int cmd_total_num;
    int buf_len;
    int proto_start;
    int retry_count;
    unsigned char serial_buf[MAX_BUF_LEN];
    struct CMD_TIEM cmd_list[$MAX_CMD_NUM];
};

typedef void (*state_handler)(struct SERIAL_CONTEXT *pContext);

struct PROTOCOL_DLT645_07
{
    int nTotalContextNum;
    struct SERIAL_CONTEXT context[$MAX_SERIAL_NUM];
    state_handler handler_list[STATE_MAX];
};

/*******************************************************************************
 * 供外部模块调用的函数，需要在前面加上extern
 *******************************************************************************/
extern void DLT64507轮询主程序(void);
extern int 添加指令(int 串口号, unsigned long long 表号, unsigned int 数据标识,
                    int 寄存器地址, int 数据长度, int 回复超时, int 指令延时);
/*******************************************************************************
 * 本地函数
 *******************************************************************************/
static void state_handler_idle(struct SERIAL_CONTEXT *pContext);
static void state_handler_send(struct SERIAL_CONTEXT *pContext);
static void state_handler_wait(struct SERIAL_CONTEXT *pContext);

/*
 Global variable
*/
static struct PROTOCOL_DLT645_07 proto_dlt645 = {.nTotalContextNum = 0,
                                                 //.context = {0},
                                                 .handler_list = {
                                                     [STATE_IDLE] = state_handler_idle,
                                                     [STATE_SEND] = state_handler_send,
                                                     [STATE_WAIT] = state_handler_wait,
                                                 }};

int dlt645_07_add_cmd(struct SERIAL_CONTEXT *pContext, unsigned long long sn,
                      unsigned int data_type, int reg_address, int reg_size, int time_out,
                      int delay_time)
{
    int id = pContext->cmd_total_num;
    if (pContext->cmd_total_num >= $MAX_CMD_NUM)
    {
        sysPrintf("Reach max command number %d", $MAX_CMD_NUM);
        return -1;
    }
    pContext->cmd_list[id].sn = sn;
    pContext->cmd_list[id].data_type = data_type;
    pContext->cmd_list[id].reg_address = reg_address;
    pContext->cmd_list[id].reg_size = reg_size;
    pContext->cmd_list[id].time_out = time_out;
    pContext->cmd_list[id].delay_time = delay_time;
    pContext->cmd_total_num++;
    return 0;
}

int dlt645_07_add(int nSerial, unsigned long long sn, unsigned int data_type,
                  int reg_address, int reg_size, int time_out, int delay_time)
{
    int i = 0, id = 0;
    if (nSerial <= 0)
    {
        sysPrintf("Serial ID starts at 1");
        return -1;
    }

    /* $MAX_SERIAL_NUM在常量表中设置，表示实际会使用串口的数量 */
    for (i = 0; i < $MAX_SERIAL_NUM; i++)
    {
        if (proto_dlt645.context[i].serial_id == nSerial)
            break;
    }

    id = i;
    if (i >= $MAX_SERIAL_NUM)
    {
        if (proto_dlt645.nTotalContextNum < $MAX_SERIAL_NUM)
        {
            /* init context */
            id = proto_dlt645.nTotalContextNum;
            proto_dlt645.context[id].state = STATE_IDLE;
            proto_dlt645.context[id].timer_id = i; /* 分配定时器ID */
            proto_dlt645.context[id].serial_id = nSerial;
            proto_dlt645.context[id].cmd_id = 0;
            proto_dlt645.context[id].cmd_total_num = 0;
            proto_dlt645.context[id].buf_len = 0;
            proto_dlt645.context[id].proto_start = 0;
            proto_dlt645.context[id].retry_count = 0;
            proto_dlt645.nTotalContextNum++;
        }
        else
        {
            sysPrintf("Reach configured max serial num %d", $MAX_SERIAL_NUM);
            return -1;
        }
    }
    /* add command */
    return dlt645_07_add_cmd(&proto_dlt645.context[id], sn, data_type, reg_address,
                             reg_size, time_out, delay_time);
}

/*******************************************************************************
 * 函数定义
 *******************************************************************************/
static void state_handler_idle(struct SERIAL_CONTEXT *pContext)
{
    pContext->cmd_id++;
    if (pContext->cmd_id >= pContext->cmd_total_num)
        pContext->cmd_id = 0;

    /* change state machine */
    pContext->state = STATE_SEND;
}

static void state_handler_send(struct SERIAL_CONTEXT *pContext)
{
    int nTxLen = 0;
    unsigned char nCRC;
    /* delay time is not out */
    if (!sysTON(pContext->timer_id, 1, pContext->cmd_list[pContext->cmd_id].delay_time))
    {
        return;
    }
    /* reset timer */
    sysTRst(pContext->timer_id);

    pContext->serial_buf[nTxLen++] = 0xFE;
    pContext->serial_buf[nTxLen++] = 0xFE;
    pContext->serial_buf[nTxLen++] = 0xFE;
    pContext->serial_buf[nTxLen++] = 0xFE;
    pContext->serial_buf[nTxLen++] = 0x68;
    pContext->serial_buf[nTxLen++] =
        (unsigned char)(pContext->cmd_list[pContext->cmd_id].sn & 0xFF);
    pContext->serial_buf[nTxLen++] =
        (unsigned char)((pContext->cmd_list[pContext->cmd_id].sn & 0xFF00) >> 8);
    pContext->serial_buf[nTxLen++] =
        (unsigned char)((pContext->cmd_list[pContext->cmd_id].sn & 0xFF0000) >> 16);
    pContext->serial_buf[nTxLen++] =
        (unsigned char)((pContext->cmd_list[pContext->cmd_id].sn & 0xFF000000) >> 24);
    pContext->serial_buf[nTxLen++] =
        (unsigned char)((pContext->cmd_list[pContext->cmd_id].sn & 0xFF00000000) >> 32);
    pContext->serial_buf[nTxLen++] =
        (unsigned char)((pContext->cmd_list[pContext->cmd_id].sn & 0xFF0000000000) >> 40);
    pContext->serial_buf[nTxLen++] = 0x68;
    pContext->serial_buf[nTxLen++] = 0x11; /* 控制码 */

    pContext->serial_buf[nTxLen++] = 0x4; /* 数据域长度 */

    /* 初始化数据域 */
    pContext->serial_buf[nTxLen++] =
        (unsigned char)(pContext->cmd_list[pContext->cmd_id].data_type & 0xFF) + 0x33;
    pContext->serial_buf[nTxLen++] =
        (unsigned char)((pContext->cmd_list[pContext->cmd_id].data_type & 0xFF00) >> 8) +
        0x33;
    pContext->serial_buf[nTxLen++] =
        (unsigned char)((pContext->cmd_list[pContext->cmd_id].data_type & 0xFF0000) >>
                        16) +
        0x33;
    pContext->serial_buf[nTxLen++] =
        (unsigned char)((pContext->cmd_list[pContext->cmd_id].data_type & 0xFF000000) >>
                        24) +
        0x33;
    /* 计算CRC */
    nCRC = 0;
    for (int i = 4; i < nTxLen; i++)
    {
        nCRC = nCRC + pContext->serial_buf[i];
    }
    pContext->serial_buf[nTxLen++] = nCRC;
    pContext->serial_buf[nTxLen++] = 0x16; /* 协议结束符 */

    sysUartFlush(pContext->serial_id, 0); /* clear uart rx buffer */
    sysUartWrite(pContext->serial_id, (char *)&pContext->serial_buf[0], nTxLen);

    /* change state machine */
    pContext->state = STATE_WAIT;
}

static void handle_rsp_error(struct SERIAL_CONTEXT *pContext)
{
    /* time is out to await response */
    pContext->retry_count++;
    pContext->buf_len = 0;
    /* 复位等待超时定时器 */
    sysTRst(pContext->timer_id);

    if (pContext->retry_count >= 3)
    {
        /* 回复超时处理 */
        sysPrintf("ERROR: serial[%d] cmd[%d]", pContext->serial_id, pContext->cmd_id);
        pContext->retry_count = 0;
        pContext->state = STATE_IDLE; /* try next request */
        return;
    }

    /* try again */
    pContext->state = STATE_SEND;
}

static void store_data(int reg_addr, int reg_len, unsigned long long value)
{
    switch (reg_len)
    {
    case 1:
        VW[WORD(reg_addr)] = value;
        break;
    case 2:
        VD[WORD(reg_addr)] = value;
        break;
    case 4:
        VLD[WORD(reg_addr)] = value;
        break;
    default:
        sysPrintf("Invalid reg_len", reg_len);
        break;
    }
}

static void handle_valid_rsp(struct SERIAL_CONTEXT *pContext)
{
    unsigned long long nValue;
    unsigned char nByte, x0, x1;
    int offset;
    /* 减去数据域长度4 */
    int len = pContext->serial_buf[pContext->proto_start + 9] - 4;

    nValue = 0;
    /* 开始解析BCD码 */
    offset = pContext->proto_start + 10 + 4;
    /* 从后往前算 */
    for (int i = len - 1; i >= 0; i--)
    {
        // sysPrintf("[%d] %02X", i, pContext->serial_buf[offset + i]);
        nByte = pContext->serial_buf[offset + i] - 0x33;
        x0 = nByte & 0x0F;
        x1 = (nByte & 0xF0) >> 4;
        nValue = nValue * 100 + x1 * 10 + x0;
    }

    // sysPrintf("nValue=%llu", nValue);
    store_data(pContext->cmd_list[pContext->cmd_id].reg_address,
               pContext->cmd_list[pContext->cmd_id].reg_size, nValue);
    /* time is out to await response */
    pContext->retry_count = 0;
    pContext->buf_len = 0;
    /* 复位等待超时定时器 */
    sysTRst(pContext->timer_id);

    pContext->state = STATE_IDLE;
}

static void state_handler_wait(struct SERIAL_CONTEXT *pContext)
{
    int i;
    unsigned char nCRC;
    int len =
        sysUartRead(pContext->serial_id, (char *)&pContext->serial_buf[pContext->buf_len],
                    MAX_BUF_LEN - pContext->buf_len);

    if (len == 0)
    {
        /* 没有超时，继续等待 */
        if (!sysTON(pContext->timer_id, 1, pContext->cmd_list[pContext->cmd_id].time_out))
        {
            return;
        }
        /* 超时处理 */
        handle_rsp_error(pContext);
        return;
    }

    pContext->buf_len += len;

    /* 报文长度还不够，再等等 */
    if (pContext->buf_len < 3)
    {
        return;
    }

    /* 如果报文不对 */
    if ((pContext->serial_buf[0] != 0xFE) && (pContext->serial_buf[0] != 0x68))
    {
        handle_rsp_error(pContext);
        return;
    }

    /* 如果报文还没有结束，继续等待 */
    if (pContext->serial_buf[pContext->buf_len - 1] != 0x16)
    {
        return;
    }

    /* 开始CRC校验 */
    /* 1. 跳过0xFE标识符 */
    for (i = 0; i < pContext->buf_len - 2; i++)
    {
        if (pContext->serial_buf[i] != 0xFE)
            break;
    }

    /* 保存协议真正开始的地址 */
    pContext->proto_start = i;
    /* 2. 开始计算CRC */
    nCRC = 0;
    for (; i < pContext->buf_len - 2; i++)
    {
        nCRC = nCRC + pContext->serial_buf[i];
    }

    /* CRC校验失败，处理错误报文 */
    if (pContext->serial_buf[pContext->buf_len - 2] != nCRC)
    {
        handle_rsp_error(pContext);
        return;
    }

    /* 到此，收到完整的正确报文，开始解析数据并存到内部寄存器 */
    handle_valid_rsp(pContext);
}

void libdlt645_07_run(void)
{
    /* run state machine */
    for (int i = 0; i < $MAX_SERIAL_NUM; i++)
    {
        if (proto_dlt645.context[i].cmd_total_num <= 0)
            continue;

        if (proto_dlt645.context[i].state < STATE_MAX)
        {
            proto_dlt645.handler_list[proto_dlt645.context[i].state](
                &proto_dlt645.context[i]);
        }
    }
}

void DLT64507轮询主程序(void)
{
    libdlt645_07_run();
}

int 添加指令(int 串口号, unsigned long long 表号, unsigned int 数据标识, int 寄存器地址,
             int 数据长度, int 回复超时, int 指令延时)
{
    return dlt645_07_add(串口号, 表号, 数据标识, 寄存器地址, 数据长度, 回复超时,
                         指令延时);
}
