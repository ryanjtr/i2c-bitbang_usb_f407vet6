#include "i2c_bitbang.h"
#include "main.h"
#include "usbd_cdc_if.h"

i2c_t i2c_handler;

/*Important: When using I2C bitbanging, you have to configure I2C GPIO in ioc
 * to input pull-up resistor
 */




static void Init_EEPROM_MEM(void)
{
    memset(i2c_handler.slave_txdata, 0xFF, sizeof(i2c_handler.slave_txdata));  // Điền toàn bộ mảng với giá trị 0xFF
}


/**
 * The function `DWT_Clock_Enable` enables the DWT cycle counter if it is not already enabled.
 */
static void DWT_Clock_Enable(void)
{
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Bật Trace
        DWT->CYCCNT = 0;                                // Reset bộ đếm
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Bật bộ đếm chu kỳ
    }
}

static void I2C_Bitbang_Init_GPIO(void)
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

static void I2C_Bitbang_config(void)
{
    i2c_handler.index_tx = 0;
    i2c_handler.index_rx = 0;
    i2c_handler.start_condition = false;
    i2c_handler.stop_condition = false;
    i2c_handler.restart_i2c = false;
    i2c_handler.isAddress_correct = false;
    i2c_handler.current_address=0x00;
    i2c_handler.start_condition_count=0;
    i2c_handler.usb_trans_flag=0;
}

void I2C_BITBANG_Init(void)
{
	  I2C_Bitbang_Init_GPIO();
	  I2C_Bitbang_config();
	  DWT_Clock_Enable();
	  Init_EEPROM_MEM();
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



__STATIC_INLINE void i2c_disable_sda_falling()
{
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_7);
}

__STATIC_INLINE void i2c_enable_sda_falling()
{
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);
}


uint8_t start=0;
/* I2C master clock speed = 100KHz*/
void I2C_Check_Start_Condition()
{
	if (I2C_Read_SCL() && !I2C_Read_SDA())
	{
		i2c_handler.state = I2C_ADDRESS_RECEIVING;
		i2c_handler.start_condition = true;
		i2c_handler.count_bit=0;
	}
	else
	{
		i2c_handler.state = I2C_DATA_RECEIVING;
	}
	while (I2C_Read_SCL())
		;
	i2c_disable_sda_falling();
	i2c_enable_scl_rising();

}

void I2C_Handle_Event()
{
    i2c_handler.bit = I2C_Read_SDA();
    if (i2c_handler.start_condition)
    {
        switch (i2c_handler.state)
        {
        case I2C_ADDRESS_RECEIVING:
            i2c_handler.slave_address = (i2c_handler.slave_address << 1 | i2c_handler.bit);
            if (++i2c_handler.count_bit == 8)
            {
                for (int i = 0; i < i2c_handler.num_of_address; i++)
                {
                    if (i2c_handler.slave_address >> 1 == i2c_handler.list_addr_slave[i])
                    {
                        i2c_handler.isAddress_correct = true;
                        break;
                    }
                }
				while (I2C_Read_SCL())
					;

				i2c_set_sda_opendrain();
                if (i2c_handler.isAddress_correct)
                {

                    I2C_SDA_Low(); // Send ACK
                    i2c_handler.bit_RW = i2c_handler.slave_address & 0x01;
                    i2c_handler.state = I2C_SET_SDA_INPUT_ONLY;

                }
                else
                {
                    I2C_SDA_High();
                    i2c_handler.restart_i2c = true;
                }
            }
            break;
        case I2C_SET_SDA_INPUT_ONLY:
            if (!i2c_handler.bit_RW)
            {
                i2c_handler.count_bit = 0;
                i2c_handler.state = I2C_DATA_RECEIVING;
                while (I2C_Read_SCL())
                    ;
                i2c_set_sda_input();
            }
            else
            {
            	if(i2c_handler.index_rx!=0)
            	{
            		i2c_handler.index_tx=i2c_handler.slave_rxdata[0];
            	}
            	else
            		i2c_handler.index_tx=i2c_handler.current_address;
                i2c_handler.count_bit = 7;
                i2c_handler.state = I2C_DATA_SENDING;
                while (I2C_Read_SCL())
                    ;
                I2C_Write_Bit((i2c_handler.slave_txdata[i2c_handler.index_tx] >> i2c_handler.count_bit) & 0x01);
                --i2c_handler.count_bit;
            }
            break;


        case I2C_DATA_RECEIVING:
            if (i2c_handler.stop_condition && !I2C_Read_SDA() && I2C_Read_SCL())
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
                if (I2C_Read_SCL() && I2C_Read_SDA() && i2c_handler.stop_condition)
                	i2c_handler.restart_i2c = true;
            }
            else
            {
            	i2c_handler.stop_condition = false;
            }
            i2c_handler.slave_rxdata[i2c_handler.index_rx] = (i2c_handler.slave_rxdata[i2c_handler.index_rx] << 1) | i2c_handler.bit;
            if (++i2c_handler.count_bit % 8 == 0)
            {
                while (I2C_Read_SCL())
                    ;
                i2c_set_sda_opendrain();
                I2C_SDA_Low(); // Send ACK
                ++i2c_handler.index_rx;
                i2c_handler.stop_condition = true;
                i2c_handler.state = I2C_SENDING_ACK;

            }
            break;

        case I2C_SENDING_ACK:
        	i2c_handler.state = I2C_DATA_RECEIVING;
            while (I2C_Read_SCL())
                ;
            i2c_set_sda_input();
            if(i2c_handler.index_rx==1)
            {
            	while (I2C_Read_SCL())
                    ;
            	i2c_enable_sda_falling();
            }
            break;

        case I2C_DATA_SENDING:
            while (I2C_Read_SCL())
                ;
            I2C_Write_Bit((i2c_handler.slave_txdata[i2c_handler.index_tx] >> i2c_handler.count_bit) & 0x01);
            if (i2c_handler.count_bit-- == 0)
            {
            	i2c_handler.stop_condition = true;
                ++i2c_handler.index_tx;
                i2c_handler.count_bit = 7;
                i2c_handler.state = I2C_CONFIG_SDA_INPUT;

            }
            break;
        case I2C_CONFIG_SDA_INPUT:
            while (I2C_Read_SCL())
                ;
            i2c_set_sda_input();
            i2c_handler.state = I2C_ACK_RECEIVING;
            break;

        case I2C_ACK_RECEIVING:
        	i2c_handler.state = I2C_DATA_SENDING;
            if (I2C_Read_SDA())
            {
            	i2c_handler.state = I2C_STOP;
            }
            else
            {
                while (I2C_Read_SCL())
                    ;
                i2c_set_sda_opendrain();
                I2C_Write_Bit((i2c_handler.slave_txdata[i2c_handler.index_tx] >> i2c_handler.count_bit) & 0x01);
                --i2c_handler.count_bit;
            }
            break;

        case I2C_STOP:
            while (!I2C_Read_SCL())
                ;
            if (i2c_handler.stop_condition && !I2C_Read_SDA() && I2C_Read_SCL())
            {
                DWT_Delay_us(10);
                if (I2C_Read_SDA() && I2C_Read_SCL())
                	i2c_handler.restart_i2c = true;
            }
            else
            {
            	i2c_handler.stop_condition = false;
            }
            break;
        default:
            break;
        }
    }
    if (i2c_handler.restart_i2c)
    {
    	i2c_handler.count_bit = 0;
    	i2c_handler.start_condition = false;
    	i2c_handler.stop_condition= false;
    	i2c_handler.restart_i2c = false;
    	i2c_handler.isAddress_correct = false;
    	i2c_handler.state = I2C_IDLE;

    	if(i2c_handler.index_rx && (i2c_handler.index_tx==0))
    	{
//    		uint8_t trans_packet[256]={0xAA,0x02,0x02};
//    		trans_packet[3]=i2c_handler.slave_address;
//    		trans_packet[4]=i2c_handler.slave_rxdata[0];//address`s value
//    		trans_packet[5]=i2c_handler.index_rx;
//    		for(int i=1;i<i2c_handler.index_rx;i++)
//    			trans_packet[i+5]=i2c_handler.slave_rxdata[i];
//    		CDC_Transmit_FS(trans_packet,i2c_handler.index_rx+6);
    		uint8_t index=1;
    		for(uint8_t address=i2c_handler.slave_rxdata[0];address<i2c_handler.index_rx+i2c_handler.slave_rxdata[0]-1;address++)
    		{
    			i2c_handler.slave_txdata[address] = i2c_handler.slave_rxdata[index++];
    		}
    		i2c_handler.current_address=i2c_handler.slave_rxdata[0];//address`s value
    	}
    	i2c_handler.index_rx = 0;
    	i2c_handler.index_tx = 0;
        I2C_Bitbang_Init_GPIO();
    }
}
