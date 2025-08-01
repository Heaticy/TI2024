//-----------------------------------------------------------------
// 程序描述:
//     LT驱动程序（继电器控制）
// 作    者: Heaticy
// 当前版本: V1.0
// 历史版本:
//  - V1.0: LT驱动程序
// 调试工具: 硬木课堂 STM32H750 核心板、LT 模块
// 说明: 使用 STM32H7 的 PB8、PB9 两个 GPIO 引脚控制 LT 模块（2 位控制信号）
//-----------------------------------------------------------------

#include "LT.h"
#include "stm32h7xx_hal.h"

//-----------------------------------------------------------------
// 函数名: LT_Init
//-----------------------------------------------------------------
//
// 【功能说明】
//   - 初始化 LT 模块所需的 GPIO 引脚（PB8、PB9）。
//   - 配置为推挽输出模式，并默认输出低电平（继电器关闭）。
//
// 【参数】
//   - 无
//
// 【返回值】
//   - 无
//
// 【注意事项】
//   - 本函数需在 main() 初始化时调用一次。
//   - 确保在调用前，STM32 HAL 库已初始化（HAL_Init() 已调用）。
//
//-----------------------------------------------------------------
void LT_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_GPIOB_CLK_ENABLE();       // 1. 使能 GPIOB 时钟（PB8、PB9 属于 GPIOB 端口）

    GPIO_Initure.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;  // 2. 设置为推挽输出模式
    GPIO_Initure.Pull = GPIO_NOPULL;          // 3. 无上下拉
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // 4. 设为高速输出（继电器驱动足够快）
    HAL_GPIO_Init(GPIOB, &GPIO_Initure);

    // 5. 初始化为低电平，确保继电器默认关闭状态
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
}

//-----------------------------------------------------------------
// 函数名: LTset
//-----------------------------------------------------------------
//
// 【功能说明】
//   - 根据输入的数值 num (0~3)，控制 PB8 和 PB9 的高低电平，
//     从而输出 2 位二进制信号给 LT 模块，实现 4 种状态切换。
//
// 【参数】
//   - int num: 范围 0~3（对应二进制 00, 01, 10, 11）
//       0 → PB8=0, PB9=0
//       1 → PB8=0, PB9=1
//       2 → PB8=1, PB9=0
//       3 → PB8=1, PB9=1
//
// 【返回值】
//   - 无
//
// 【注意事项】
//   - num 之外的值不会报错，但无意义（按二进制取最低两位输出）。
//   - 适用于控制 2 路继电器或多路输入选择（例如模拟开关）。
//
//-----------------------------------------------------------------
void LTset(int num)
{
    // 提取 PB8 的状态（num 的第 2 位）
    GPIO_PinState pin8state = (num & 2) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    // 提取 PB9 的状态（num 的第 1 位）
    GPIO_PinState pin9state = (num & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    // 输出到 PB8 和 PB9
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, pin8state);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, pin9state);
}
