#include "i2c_bitbang.h"
#include "main.h"
#include "usbd_cdc_if.h"

i2c_t i2c_handler;

/*Important: When using I2C bitbanging, you have to configure I2C GPIO in ioc
 * to input pull-up resistor
 */

/**
 * The function `DWT_Clock_Enable` enables the DWT cycle counter if it is not already enabled.
 */
void DWT_Clock_Enable(void)
{
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Bật Trace
        DWT->CYCCNT = 0;                                // Reset bộ đếm
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Bật bộ đếm chu kỳ
    }
}

void I2C_Bitbang_Init(void)
{
    /*Configure SDA pin to input interrupt falling edge
     * after detect start condition, then disable interrupt.
     * When receive 8 bit address, change it to output open-drain float
     */

    /*Configure SCL pin as input first
     * after detecting start condition, then change it to interrupt rising edge (input)
     */

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE7);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE6);

    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_7);

    /* Configure GPIO*/

    LL_GPIO_SetPinMode(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_PULL_NO);

    /*Configure GPIO*/
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_6);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_6);
    LL_GPIO_SetPinMode(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(I2C_GPIO_PORT, I2C_SCL_PIN, LL_GPIO_PULL_NO);

    NVIC_SetPriority(EXTI9_5_IRQn, 2);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void I2C_Bitbang_config(void)
{
    i2c_handler.index_tx = 0;
    i2c_handler.index_rx = 0;
    i2c_handler.start_condition = false;
    i2c_handler.stop_condition = false;
    i2c_handler.restart_i2c = false;
    i2c_handler.isAddress_correct = false;
}

/**
 * @brief  This function provides a delay (in microseconds)
 * @param  microseconds: delay in microseconds
 */
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
    uint32_t clk_cycle_start = DWT->CYCCNT;
    microseconds *= (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - clk_cycle_start) < microseconds)
        ;
}

__STATIC_INLINE void I2C_SDA_High(void)
{
    LL_GPIO_SetOutputPin(I2C_GPIO_PORT, I2C_SDA_PIN);
}

__STATIC_INLINE void I2C_SDA_Low(void)
{
    LL_GPIO_ResetOutputPin(I2C_GPIO_PORT, I2C_SDA_PIN);
}

__STATIC_INLINE unsigned char I2C_Read_SDA(void)
{
    unsigned char state = LL_GPIO_IsInputPinSet(I2C_GPIO_PORT, I2C_SDA_PIN);
    return state;
}

__STATIC_INLINE unsigned char I2C_Read_SCL(void)
{
    unsigned char state = LL_GPIO_IsInputPinSet(I2C_GPIO_PORT, I2C_SCL_PIN);
    return state;
}

__STATIC_INLINE void I2C_Write_Bit(bool bit)
{
    if (bit)
    {
        LL_GPIO_SetOutputPin(I2C_GPIO_PORT, I2C_SDA_PIN);
    }
    else
    {
        LL_GPIO_ResetOutputPin(I2C_GPIO_PORT, I2C_SDA_PIN);
    }
}

__STATIC_INLINE void i2c_set_sda_opendrain()
{
    LL_GPIO_SetPinMode(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinSpeed(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinPull(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_PULL_NO);
}

__STATIC_INLINE void i2c_set_sda_input()
{
    LL_GPIO_SetPinMode(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(I2C_GPIO_PORT, I2C_SDA_PIN, LL_GPIO_PULL_NO);
}

__STATIC_INLINE void i2c_enable_scl_rising()
{
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);
}

__STATIC_INLINE void i2c_disable_scl_rising()
{
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_6);
}

__STATIC_INLINE void i2c_disable_sda_falling()
{
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_7);
}

__STATIC_INLINE void i2c_enable_sda_falling()
{
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);
}

I2C_State i2c_state = I2C_IDLE;
uint8_t count_bit = 0;
uint8_t slave_address = 0x00;
unsigned char Slave_rxdata[256] = {0};
uint8_t index_rxdata = 0;
uint8_t Slave_txdata[256] = {0};
uint8_t index_txdata = 0;
bool start_condition = false;
bool check_if_stop = false;
unsigned char bit;
unsigned char bit_RW;
bool restart_i2c = false;
uint8_t ReceivedData[100];
uint8_t list_addr_slave[5] = {0x68, 0x76, 0x58, 0x68, 0x55};
bool correct_address = false;
uint32_t timeout = 1000;
uint32_t count = 0;
/* I2C master clock speed = 100KHz*/
void I2C_Check_Start_Condition()
{
	if (I2C_Read_SCL() && !I2C_Read_SDA())
	{
		i2c_state = I2C_ADDRESS_RECEIVING;
		start_condition = true;
		count_bit=0;
	}
	else
	{
		i2c_state = I2C_DATA_RECEIVING;
	}
	while (I2C_Read_SCL())
		;
	i2c_disable_sda_falling();
	i2c_enable_scl_rising();
}

void I2C_Handle_Event()
{
    bit = I2C_Read_SDA();
    if (start_condition)
    {
        switch (i2c_state)
        {
        case I2C_ADDRESS_RECEIVING:
            slave_address = (slave_address << 1 | bit);
            if (++count_bit == 8)
            {
                DWT_Delay_us(5);
                i2c_set_sda_opendrain();
                for (int i = 0; i < 5; i++)
                {
                    if (slave_address >> 1 == list_addr_slave[i])
                    {
                        correct_address = true;
                        break;
                    }
                }
                if (correct_address)
                {
                    I2C_SDA_Low(); // Send ACK
                    bit_RW = slave_address & 0x01;
                    i2c_state = I2C_SET_SDA_INPUT_ONLY;
                }
                else
                {
                    I2C_SDA_High();
                    restart_i2c = true;
                }
            }
            break;
        case I2C_SET_SDA_INPUT_ONLY:
            if (!bit_RW)
            {
                count_bit = 0;
                i2c_state = I2C_DATA_RECEIVING;
                while (I2C_Read_SCL())
                    ;
                i2c_set_sda_input();
            }
            else
            {
                count_bit = 7;
                i2c_state = I2C_DATA_SENDING;
                while (I2C_Read_SCL())
                    ;
                I2C_Write_Bit((Slave_txdata[index_txdata] >> count_bit) & 0x01);
                --count_bit;
            }

            break;
        case I2C_DATA_RECEIVING:
            if (check_if_stop && !I2C_Read_SDA() && I2C_Read_SCL())
            {
                uint32_t count = 0;
                while (I2C_Read_SCL())
                {
                    ++count;
                    if (count > 1000)
                    {
                        break;
                    }
                }
                if (I2C_Read_SCL() && I2C_Read_SDA() && check_if_stop)
                    restart_i2c = true;
            }
            else
            {
                check_if_stop = false;
            }
            Slave_rxdata[index_rxdata] = (Slave_rxdata[index_rxdata] << 1) | bit;
            if (++count_bit % 8 == 0)
            {
                while (I2C_Read_SCL())
                    ;
                i2c_set_sda_opendrain();
                I2C_SDA_Low(); // Send ACK
                ++index_rxdata;
                check_if_stop = true;
                i2c_state = I2C_SENDING_ACK;
            }
            break;

        case I2C_SENDING_ACK:
            i2c_state = I2C_DATA_RECEIVING;
            while (I2C_Read_SCL())
                ;
            i2c_set_sda_input();
            if(index_rxdata==1)
            	i2c_enable_sda_falling();
            break;

        case I2C_DATA_SENDING:
            while (I2C_Read_SCL())
                ;
            I2C_Write_Bit((Slave_txdata[index_txdata] >> count_bit) & 0x01);
            if (count_bit-- == 0)
            {
                check_if_stop = true;
                ++index_txdata;
                count_bit = 7;
                i2c_state = I2C_CONFIG_SDA_INPUT;
            }
            break;
        case I2C_CONFIG_SDA_INPUT:
            while (I2C_Read_SCL())
                ;
            i2c_set_sda_input();
            i2c_state = I2C_ACK_RECEIVING;
            break;

        case I2C_ACK_RECEIVING:
            i2c_state = I2C_DATA_SENDING;
            if (I2C_Read_SDA())
            {
                i2c_state = I2C_STOP;
            }
            else
            {
                while (I2C_Read_SCL())
                    ;
                i2c_set_sda_opendrain();
                I2C_Write_Bit((Slave_txdata[index_txdata] >> count_bit) & 0x01);
                --count_bit;
            }
            break;

        case I2C_STOP:
            while (!I2C_Read_SCL())
                ;
            if (check_if_stop && !I2C_Read_SDA() && I2C_Read_SCL())
            {
                DWT_Delay_us(10);
                if (I2C_Read_SDA() && I2C_Read_SCL())
                    restart_i2c = true;
            }
            else
            {
                check_if_stop = false;
            }
            break;
        default:
            break;
        }
    }
    if (restart_i2c)
    {
        count_bit = 0;
        start_condition = false;
        check_if_stop = false;
        restart_i2c = false;
        correct_address = false;
        i2c_state = I2C_IDLE;
        for (int i = 0; i < index_rxdata; i++)
        {
            uart_printf("d%d=0x%02X\r\n", i, Slave_rxdata[i]);
            Slave_rxdata[i] = 0;
        }
        index_rxdata = 0;
        //        index_txdata = 0;
        I2C_Bitbang_Init();
    }
}

// void I2C_Check_Start_Condition()
//{
//     if (I2C_Read_SCL() && !I2C_Read_SDA())
//     {
//         start_condtion = true;
//         i2c_state = I2C_ADDRESS_RECEIVING;
//         while (I2C_Read_SCL()) // Don`t delete
//             ;
//         i2c_disable_sda_falling();
//         i2c_enable_scl_rising();
//     }
// }
//
// void I2C_Handle_Event()
//{
//     bit = I2C_Read_Bit();
//     if (start_condtion)
//     {
//         switch (i2c_state)
//         {
//         case I2C_ADDRESS_RECEIVING:
//             Slave_Address = (Slave_Address << 1 | bit);
//             if (++count_bit == 8)
//             {
//             	DWT_Delay_us(5);
//                 i2c_set_sda_opendrain();
//                 for (int i = 0; i < 5; i++)
//                 {
//                     if (Slave_Address >> 1 == list_addr_slave[i])
//                     {
//                         correct_address = true;
//                         break;
//                     }
//                 }
//                 if (correct_address)
//                 {
//                 	I2C_SDA_Low(); // Send ACK
//                     bit_RW = Slave_Address & 0x01;
//                     i2c_state = I2C_SET_SDA_INPUT_ONLY;
//                 }
//                 else
//                 {
//                     I2C_SDA_High();
//                     restart_i2c = true;
//                 }
//             }
//             break;
//         case I2C_SET_SDA_INPUT_ONLY:
//             if (!bit_RW)
//             {
//                 count_bit = 0;
//                 i2c_state = I2C_DATA_RECEIVING;
//                 while (I2C_Read_SCL())
//                     ;
//                 i2c_set_sda_input();
//             }
//             else
//             {
//                 count_bit = 7;
//                 i2c_state = I2C_DATA_SENDING;
//                 while (I2C_Read_SCL())
//                     ;
//                 I2C_Write_Bit((Slave_txdata[index_txdata] >> count_bit) & 0x01);
//                 --count_bit;
//             }
//
//             break;
//         case I2C_DATA_RECEIVING:
//             if (check_if_stop && !I2C_Read_SDA() && I2C_Read_SCL())
//             {
//                 uint32_t count = 0;
//                 while (I2C_Read_SCL())
//                 {
//                     ++count;
//                     if (count > 1000)
//                     {
//                         break;
//                     }
//                 }
//                 if (I2C_Read_SCL() && I2C_Read_SDA() && check_if_stop)
//                     restart_i2c = true;
//             }
//             else
//             {
//                 check_if_stop = false;
//             }
//             Slave_rxdata[index_rxdata] = (Slave_rxdata[index_rxdata] << 1) | bit;
//             if (++count_bit % 8 == 0)
//             {
//                 while (I2C_Read_SCL())
//                     ;
//                 i2c_set_sda_opendrain();
//                 I2C_SDA_Low(); // Send ACK
//                 ++index_rxdata;
//                 check_if_stop = true;
//                 i2c_state = I2C_SENDING_ACK;
//             }
//             break;
//
//         case I2C_SENDING_ACK:
//             i2c_state = I2C_DATA_RECEIVING;
//             while (I2C_Read_SCL())
//                 ;
//             i2c_set_sda_input();
//             break;
//
//         case I2C_DATA_SENDING:
//             while (I2C_Read_SCL())
//                 ;
//             I2C_Write_Bit((Slave_txdata[index_txdata] >> count_bit) & 0x01);
//             if (count_bit-- == 0)
//             {
//                 check_if_stop = true;
//                 ++index_txdata;
//                 count_bit = 7;
//                 i2c_state = I2C_CONFIG_SDA_INPUT;
//             }
//             break;
//         case I2C_CONFIG_SDA_INPUT:
//             while (I2C_Read_SCL())
//                 ;
//             i2c_set_sda_input();
//             i2c_state = I2C_ACK_RECEIVING;
//             break;
//         case I2C_ACK_RECEIVING:
//             i2c_state = I2C_DATA_SENDING;
//             if (I2C_Read_SDA())
//             {
//                 i2c_state = I2C_STOP;
//             }
//             else
//             {
//                 while (I2C_Read_SCL())
//                     ;
//                 i2c_set_sda_opendrain();
//                 I2C_Write_Bit((Slave_txdata[index_txdata] >> count_bit) & 0x01);
//                 --count_bit;
//             }
//             break;
//
//         case I2C_STOP:
//             while (!I2C_Read_SCL())
//                 ;
//             if (check_if_stop && !I2C_Read_SDA() && I2C_Read_SCL())
//             {
//                 DWT_Delay_us(10);
//                 if (I2C_Read_SDA() && I2C_Read_SCL())
//                     restart_i2c = true;
//             }
//             else
//             {
//                 check_if_stop = false;
//             }
//             break;
//         default:
//             break;
//         }
//     }
//     if (restart_i2c)
//     {
//         count_bit = 0;
//         start_condtion = false;
//         check_if_stop = false;
//         restart_i2c = false;
//         correct_address = false;
//         i2c_state = I2C_IDLE;
//         for (int i = 0; i < index_rxdata; i++)
//         {
//             uart_printf("d%d=0x%02X\r\n", i, Slave_rxdata[i]);
//             Slave_rxdata[i] = 0;
//         }
//         index_rxdata = 0;
//         index_txdata = 0;
//         I2C_Bitbang_Init();
//     }
// }
