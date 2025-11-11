#include "bsp_key.h"
#include "bsp_tim.h"
#include "bsp_tps26400.h"
#include "bsp_led.h"

bool Enable_Process = false;
bool BootJump_Process = false;
TestedObj_e TestedObj_Current = TestedObj_None;
uint8_t toggle_switch_bits[3] = {0};

static TestedObj_e mode_format(uint8_t* bits, uint8_t len);

static TestedObj_e mode_format(uint8_t* bits, uint8_t len) 
{
    if (len != 3) return TestedObj_None;
    uint8_t mode = 0;
    mode = (bits[2] << 2) | (bits[1] << 1) | bits[0];
    //筛选模式
    switch (mode) {
        case 0:
            return TestedObj_None;//无测试对象
        case 1:
            return TestedObj_Spinal;//脊柱板测试
        case 2:
            return TestedObj_Driver;//驱动板测试
        case 3:
            return TestedObj_MAWire;//带磁编线测试
        case 4:
            return TestedObj_NMAWire;//不带磁编线测试
        case 5:
            return TestedObj_IMU;//IMU传感器测试
        default:
            return TestedObj_None;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
    switch (GPIO_Pin) 
    {
        case GPIO_PIN_12://KEY1  人工触发（按钮3）
            if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
            {
                //force disable
                TPS26400_Disable();
                Enable_Process = false;
                YELLOW_ON();
            }
            break;
        case GPIO_PIN_13://KEY2  人工触发（按钮2）
            if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
            {
                //enable process
                Enable_Process = true;
            }
            break;
        case GPIO_PIN_14://KEY3  人工触发（按钮1）
            if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
            {
                //force enable process
                //TPS26400_Delay_Enable_Signal();
                BootJump_Process = true;
//                GREEN_ON();
//                FLASH_ON;
            }
            break;
        case GPIO_PIN_15://KEY4  上电自动触发
            if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET)
            {
                //detected cover closing
                TPS26400_Delay_Enable_Signal();
//                BootJump_Process = true;//感觉不需要
//                GREEN_ON();
//                FLASH_ON;
            }
            else
            {
                //detected cover opening
                TPS26400_Disable();
                YELLOW_ON();
            }
            break;
        default:
            break;
    }
}

void Read_Toggle_Switch(void) 
{
    uint8_t *bits = toggle_switch_bits;//获取板上开关数值
    //反转0和1
    bits[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET;//bit3
    bits[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET;//bit2
    bits[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET;//bit1
    TestedObj_Current = mode_format(bits, 3);
}

//测试项配置函数：根据测试对象设置需要执行的测试项目（）
//参数：obj - 当前测试对象（枚举类型）;test_item - 输出参数，按位存储配置项
void TestItem_Config(TestedObj_e obj, TestItem_t *test_item)
{
    TestItem_t item_configure = {0};
    uint16_t temp = 0;
    switch(obj)
    {
        case TestedObj_None://无测试对象
            break;
        case TestedObj_Spinal://脊髓板测试
            temp = (1 << ITEM_RS485_BIT) | (1 << ITEM_INPUT_ENCODER_BIT);//RS485通信+输入编码器测试
            item_configure.raw_value = temp;
            break;
        case TestedObj_Driver://驱动板测试
            temp =  (1 << ITEM_BUS_VOLT_BIT) | //总电压测试
                    (1 << ITEM_RS485_BIT) |    //RS485通信测试
                    (1 << ITEM_INPUT_ENCODER_BIT) |  //输入编码器测试
                    (1 << ITEM_OUTPUT_ENCODER_BIT) | //输出编码器测试
                    (1 << ITEM_VREF_BIT) |           //参考电压测试
                    (1 << ITEM_PHASE_CURRENT_BIT) |  //相电流测试
                    (1 << ITEM_DRV8316_STATE_BIT) |  //DRV8316状态寄存器测试
					(1 << ITEM_DRV8316_DRIVE_BIT);             //DRV8316驱动功能测试
            item_configure.raw_value = temp;         //保存配置结果
            break;
        case TestedObj_MAWire://带磁编排线测试
            temp = 0;
            item_configure.raw_value = temp;
            break;
        case TestedObj_NMAWire://不带磁编排线测试
            temp = 0;
            item_configure.raw_value = temp;
            break;
        case TestedObj_IMU://IMU传感器测试
            temp = 0;
            item_configure.raw_value = temp;
            break;
        default:
            break;
    }
    *test_item = item_configure;
}

