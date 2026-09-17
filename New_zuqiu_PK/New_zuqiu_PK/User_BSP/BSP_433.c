#include "BSP_433.h"

/*
 * 接收方案说明
 * ------------
 * 1. 配置的串口每收到 1 个字节就进入 HAL_UART_RxCpltCallback()。
 * 2. 中断只负责把“字节 + 接收时刻”放入环形缓冲区，避免在中断中解析。
 * 3. BSP_433_GetKeyState() 在主循环上下文中按两个字节组合键码。
 * 4. 若两个字节不能组成任何已知键码，只丢弃第一个字节，再滑动一位重试。
 *    这样即使偶尔丢失或插入一个字节，也能在下一个合法键码处重新同步。
 *
 * IDLE 只能表示总线暂时空闲，不能保证落在两字节键码的边界上，因此这里
 * 不使用 IDLE 作为分帧条件。粘包、连续发送以及键码跨回调拆分都由环形缓冲
 * 和滑动解析统一处理。
 */

/*
 * 环形缓冲区实际最多保存 BSP_433_RX_RING_SIZE - 1 个字节。
 * 按示例中的遥控器连发速率，128 项可以缓存数秒数据；每项保留时间戳，是为了
 * 防止主循环长时间未调度后，把缓冲区中的旧报文错误地当成“刚刚按下”。
 */
#define BSP_433_RX_RING_SIZE          (128U)

/* 同一键码的两个字节在 9600 波特率下约相隔 1 ms，20 ms 已留有充分余量。 */
#define BSP_433_MAX_BYTE_GAP_MS       (20U)

/* 433 模块模式切换和配置命令所需的近似等待时间。 */
#define BSP_433_MODE_DELAY_MS          (500U)

typedef struct
{
    uint8_t data;
    uint32_t received_tick;
} BSP_433_RxByte_t;

typedef struct
{
    uint16_t command;
    uint32_t last_received_tick;
    bool is_pressed;
} BSP_433_KeyState_t;

/*
 * 环形缓冲区是单生产者/单消费者结构：
 * - 串口接收中断只修改 s_rx_write_index；
 * - 主循环只修改 s_rx_read_index。
 * volatile 用于保证两个执行上下文每次都读取最新的索引和数据。
 */
static volatile BSP_433_RxByte_t s_rx_ring[BSP_433_RX_RING_SIZE];
static volatile uint16_t s_rx_write_index = 0U;
static volatile uint16_t s_rx_read_index = 0U;

/*
 * 串口错误或环形缓冲区溢出意味着数据流可能已经不完整。主循环看到该标志后，
 * 会丢弃待解析数据并释放所有按键，防止小车继续执行一个已经过期的动作。
 */
static volatile bool s_rx_fault_pending = false;

/* HAL 的单字节中断接收目标。 */
static uint8_t s_uart_rx_byte = 0U;

/*
 * 缓冲区满时保留旧数据并丢弃新字节。该计数仅供在线调试器观察；
 * 正常情况下它应始终为 0。解析器采用滑动重同步，所以单次溢出不会永久错位。
 */
static volatile uint32_t s_rx_overflow_count = 0U;

/* 每个按键都有独立的状态和最后接收时间，因此多个按键不会相互覆盖。 */
static BSP_433_KeyState_t s_key_states[] =
{
    {CMD_Cross_Up,    0U, false},
    {CMD_Cross_Down,  0U, false},
    {CMD_Cross_LEFT,  0U, false},
    {CMD_Cross_RIGHT, 0U, false},
    {CMD_Forward,     0U, false},
    {CMD_Back,        0U, false},
    {CMD_One,         0U, false},
    {CMD_Two,         0U, false}
};

#define BSP_433_KEY_COUNT \
    ((uint16_t)(sizeof(s_key_states) / sizeof(s_key_states[0])))

static void BSP_433_BusyWaitMs(uint32_t delay_ms);
static uint16_t BSP_433_RingNextIndex(uint16_t index);
static void BSP_433_RingPushFromISR(uint8_t data, uint32_t received_tick);
static BSP_433_KeyState_t *BSP_433_FindKeyState(uint16_t command);
static void BSP_433_ClearKeyStates(void);
static void BSP_433_ResetReceiver(void);
static void BSP_433_HandlePendingFault(void);
static void BSP_433_ProcessReceivedData(void);
static void BSP_433_ExpireKeyStates(uint32_t now);

/**
 * @brief 不依赖 SysTick 的 CPU 忙等待延时。
 *
 * STM32F103 的 Cortex-M3 内核带有 DWT 周期计数器。计数器寄存器由 CMSIS
 * 声明为 volatile，循环内还保留 __NOP()，所以即使打开编译优化，这段等待也
 * 不会被优化器删除。函数只让当前执行流忙等，不会关闭中断；UART、DMA 等中断
 * 仍可在等待期间正常响应。
 *
 * 与 HAL_Delay() 不同，本函数不依赖 SysTick 中断推进系统节拍，因此即使调用
 * 位置的中断优先级高于 SysTick，也不会因为 HAL tick 无法更新而永久卡住。
 */
static void BSP_433_BusyWaitMs(uint32_t delay_ms)
{
    uint32_t cycles_per_ms;
    uint32_t start_cycle;

    if (delay_ms == 0U)
    {
        return;
    }

    /* 打开内核跟踪模块和周期计数器；不清零 CYCCNT，避免影响其他性能测量。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cycles_per_ms = SystemCoreClock / 1000U;

    while (delay_ms > 0U)
    {
        start_cycle = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start_cycle) < cycles_per_ms)
        {
            __NOP();
        }

        delay_ms--;
    }
}

/**
 * @brief 计算环形缓冲区中的下一个索引。
 */
static uint16_t BSP_433_RingNextIndex(uint16_t index)
{
    index++;
    if (index >= BSP_433_RX_RING_SIZE)
    {
        index = 0U;
    }

    return index;
}

/**
 * @brief 从 USART 中断向环形缓冲区写入一个字节。
 * @note  先写数据和时间戳，最后再发布写索引，主循环不会读到半写入的数据。
 */
static void BSP_433_RingPushFromISR(uint8_t data, uint32_t received_tick)
{
    uint16_t write_index = s_rx_write_index;
    uint16_t next_write_index = BSP_433_RingNextIndex(write_index);

    if (next_write_index == s_rx_read_index)
    {
        /* 缓冲区已满。丢弃当前字节，避免中断和主循环同时修改读索引。 */
        s_rx_overflow_count++;
        s_rx_fault_pending = true;
        return;
    }

    s_rx_ring[write_index].data = data;
    s_rx_ring[write_index].received_tick = received_tick;
    s_rx_write_index = next_write_index;
}

/**
 * @brief 在内部按键表中查找命令。
 */
static BSP_433_KeyState_t *BSP_433_FindKeyState(uint16_t command)
{
    uint16_t index;

    for (index = 0U; index < BSP_433_KEY_COUNT; index++)
    {
        if (s_key_states[index].command == command)
        {
            return &s_key_states[index];
        }
    }

    return NULL;
}

/**
 * @brief 释放所有按键状态。
 */
static void BSP_433_ClearKeyStates(void)
{
    uint16_t index;

    for (index = 0U; index < BSP_433_KEY_COUNT; index++)
    {
        s_key_states[index].last_received_tick = 0U;
        s_key_states[index].is_pressed = false;
    }
}

/**
 * @brief 清空缓冲区以及所有按键状态。
 * @note  仅在 USART 接收尚未启动或已经停止时调用。
 */
static void BSP_433_ResetReceiver(void)
{
    s_rx_write_index = 0U;
    s_rx_read_index = 0U;
    s_rx_fault_pending = false;
    s_rx_overflow_count = 0U;
    s_uart_rx_byte = 0U;
    BSP_433_ClearKeyStates();
}

/**
 * @brief 处理 ISR 上报的数据不完整故障，并让全部按键进入安全的释放状态。
 *
 * 临界区只用于同步两个环形索引和故障标志，耗时固定且极短；按键表只有主循环
 * 会修改，因此可在恢复中断后清零。新到达的数据会留在队列中供随后解析。
 */
static void BSP_433_HandlePendingFault(void)
{
    uint32_t interrupt_mask;

    if (!s_rx_fault_pending)
    {
        return;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
    s_rx_read_index = s_rx_write_index;
    s_rx_fault_pending = false;
    if (interrupt_mask == 0U)
    {
        __enable_irq();
    }

    BSP_433_ClearKeyStates();
}

/**
 * @brief 解析环形缓冲区中所有完整的两字节键码。
 *
 * 若只剩一个字节则保留，等待下一次接收。候选键码无效或两个字节间隔过大时，
 * 只前移一个字节进行滑动重同步，而不是直接丢弃两个字节。
 */
static void BSP_433_ProcessReceivedData(void)
{
    uint16_t read_index;
    uint16_t second_index;
    uint16_t write_index;
    uint16_t command;
    uint32_t first_tick;
    uint32_t second_tick;
    BSP_433_KeyState_t *key_state;

    while (true)
    {
        read_index = s_rx_read_index;
        write_index = s_rx_write_index;

        if (read_index == write_index)
        {
            /* 缓冲区为空。 */
            break;
        }

        second_index = BSP_433_RingNextIndex(read_index);
        if (second_index == write_index)
        {
            /* 只收到键码的第一个字节，留到下一次调用继续解析。 */
            break;
        }

        first_tick = s_rx_ring[read_index].received_tick;
        second_tick = s_rx_ring[second_index].received_tick;
        command = (uint16_t)(((uint16_t)s_rx_ring[read_index].data << 8U) |
                             (uint16_t)s_rx_ring[second_index].data);
        key_state = BSP_433_FindKeyState(command);

        if ((key_state != NULL) &&
            ((uint32_t)(second_tick - first_tick) <= BSP_433_MAX_BYTE_GAP_MS))
        {
            /*
             * 每个键只刷新自己的时间戳。交替收到 CMD_Forward 与
             * CMD_Cross_LEFT/RIGHT 时，两者会同时保持为 true。
             */
            key_state->last_received_tick = second_tick;
            key_state->is_pressed = true;

            /* 合法键码消费两个字节。 */
            s_rx_read_index = BSP_433_RingNextIndex(second_index);
        }
        else
        {
            /* 无效组合只消费一个字节，以便从下一字节重新尝试配对。 */
            s_rx_read_index = second_index;
        }
    }
}

/**
 * @brief 根据各按键自己的最后接收时间，独立释放超时按键。
 * @note  无符号减法可正确处理 HAL_GetTick() 大约 49.7 天一次的回绕。
 */
static void BSP_433_ExpireKeyStates(uint32_t now)
{
    uint16_t index;

    for (index = 0U; index < BSP_433_KEY_COUNT; index++)
    {
        if (s_key_states[index].is_pressed &&
            ((uint32_t)(now - s_key_states[index].last_received_tick) >=
             BSP_433_KEY_TIMEOUT_MS))
        {
            s_key_states[index].is_pressed = false;
        }
    }
}

/*
 * 配置 433 模块并启动接收。
 * 原有配置时序保持不变：M0=0、M1=1 进入设置模式，发送配置后双脚拉低
 * 回到透明收发模式。最后才启动 RX 中断，避免把设置过程误当作按键数据。
 */
void Set_uart_433_Init(void)
{
    static uint8_t uart_tx_buffer[] = {0xC0U, 0xFFU, 0xFFU,
                                       0x19U, 0x3EU, 0x00U};

    /* 允许重复调用初始化函数：先停止上一次可能仍在运行的接收。 */
    (void)HAL_UART_AbortReceive(&BSP_433_UART_HANDLE);
    BSP_433_ResetReceiver();

    BSP_433_BusyWaitMs(BSP_433_MODE_DELAY_MS);

    /* M0 拉低、M1 拉高，使 433 模块进入设置模式。 */
    HAL_GPIO_WritePin(BSP_433_M0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BSP_433_M1, GPIO_PIN_SET);

    BSP_433_BusyWaitMs(BSP_433_MODE_DELAY_MS);
    if (HAL_UART_Transmit_DMA(&BSP_433_UART_HANDLE, uart_tx_buffer,
                              (uint16_t)sizeof(uart_tx_buffer)) != HAL_OK)
    {
        Error_Handler();
    }

    /* 等待短报文发送完成，再切换模块工作模式。 */
    BSP_433_BusyWaitMs(BSP_433_MODE_DELAY_MS);

    /* M0、M1 双拉低，回到正常透明收发模式。 */
    HAL_GPIO_WritePin(BSP_433_M0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BSP_433_M1, GPIO_PIN_RESET);

    /* 清除设置阶段可能遗留在 DR 中的字节和 ORE 标志，再开始接收按键。 */
    __HAL_UART_CLEAR_OREFLAG(&BSP_433_UART_HANDLE);

    /* 从此以后每次接收 1 字节，回调写入环形缓冲区后立即续接。 */
    if (HAL_UART_Receive_IT(&BSP_433_UART_HANDLE, &s_uart_rx_byte, 1U) != HAL_OK)
    {
        Error_Handler();
    }
}

bool BSP_433_GetKeyState(uint16_t key_cmd)
{
    BSP_433_KeyState_t *key_state;

    BSP_433_HandlePendingFault();
    BSP_433_ProcessReceivedData();
    /* 若解析期间刚好发生溢出，也在本次查询中立即进入安全状态。 */
    BSP_433_HandlePendingFault();
    BSP_433_ExpireKeyStates(HAL_GetTick());

    key_state = BSP_433_FindKeyState(key_cmd);
    if (key_state == NULL)
    {
        return false;
    }

    return key_state->is_pressed;
}

/*
 * HAL 的 UART 接收完成回调在整个工程中只能有一个强定义。
 * 当前模块只处理 BSP_433_UART_HANDLE；以后若增加其他串口，请继续在此分发。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart != &BSP_433_UART_HANDLE))
    {
        return;
    }

    BSP_433_RingPushFromISR(s_uart_rx_byte, HAL_GetTick());

    /* HAL 在进入完成回调前已把 RxState 恢复为 READY，可以立即续接。 */
    (void)HAL_UART_Receive_IT(huart, &s_uart_rx_byte, 1U);
}

/*
 * ORE（溢出）等阻塞型串口错误会让 HAL 停止当前 IT 接收；若不在错误回调中
 * 重新启动，433 接收会永久停住。非阻塞错误下接收可能仍为 BUSY，此时下面的
 * HAL_UART_Receive_IT() 会返回 HAL_BUSY，而原接收过程会继续运行。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart != &BSP_433_UART_HANDLE))
    {
        return;
    }

    s_rx_fault_pending = true;
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }
    (void)HAL_UART_Receive_IT(huart, &s_uart_rx_byte, 1U);
}

/*
 * 读取模块配置参数的调试函数。
 * 模块会响应 6 个字节，本模块不解析该响应；可使用 CH340 和串口助手观察。
 */
void Red_uart_433(void)
{
    static uint8_t uart_tx_buffer[] = {0xC1U, 0xC1U, 0xC1U};

    /* M0 拉低、M1 拉高，进入设置模式。 */
    HAL_GPIO_WritePin(BSP_433_M0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BSP_433_M1, GPIO_PIN_SET);

    BSP_433_BusyWaitMs(BSP_433_MODE_DELAY_MS);
    (void)HAL_UART_Transmit_DMA(&BSP_433_UART_HANDLE, uart_tx_buffer,
                                (uint16_t)sizeof(uart_tx_buffer));

    /* DMA 发送是异步的，切换模式前必须给配置查询报文留出发送时间。 */
    BSP_433_BusyWaitMs(BSP_433_MODE_DELAY_MS);

    /* 恢复双拉低的正常透明收发模式。 */
    HAL_GPIO_WritePin(BSP_433_M0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BSP_433_M1, GPIO_PIN_RESET);
}

/*
 * 发送固定测试报文 0xAA、0xBB、0xCC，用于检查无线数据链路。
 */
void Witch_uart_433(void)
{
    static uint8_t uart_tx_buffer[] = {0xAAU, 0xBBU, 0xCCU};

    /* 双拉低为正常透明收发模式。 */
    HAL_GPIO_WritePin(BSP_433_M0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BSP_433_M1, GPIO_PIN_RESET);

    BSP_433_BusyWaitMs(BSP_433_MODE_DELAY_MS);
    (void)HAL_UART_Transmit_DMA(&BSP_433_UART_HANDLE, uart_tx_buffer,
                                (uint16_t)sizeof(uart_tx_buffer));
}
