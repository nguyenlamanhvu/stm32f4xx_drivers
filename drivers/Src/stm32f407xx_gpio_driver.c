#include "stm32f407xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)	GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)	GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)	GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)	GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF)	GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG)	GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH)	GPIOH_PCLK_EN();
		else if(pGPIOx == GPIOI)	GPIOI_PCLK_EN();
	} else
	{
		if(pGPIOx == GPIOA)			GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB)	GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC)	GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD)	GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE)	GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOF)	GPIOF_PCLK_DI();
		else if(pGPIOx == GPIOG)	GPIOG_PCLK_DI();
		else if(pGPIOx == GPIOH)	GPIOH_PCLK_DI();
		else if(pGPIOx == GPIOI)	GPIOI_PCLK_DI();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes GPIOx
 *
 * @param[in]         - Handle structure of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0; 	//temp register
	//enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	//1. configure the mode of GPIO PIN
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)	// the non interrupt mode
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;		//setting
	} else																// this is interrupt mode
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode <<(temp2 * 4);

		//3 . enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting
	//3. configure the PUPD settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; //setting
	//4. configure the output type (push-pull or open-drain)
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; //setting
	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function registers.
		uint8_t temp1, temp2;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);		//to know ALT[0] or ALT[1]
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);		//to know the pin number in the ALT[0] or ALT[1]

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));		//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode 	<< (4 * temp2));;
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function deinitializes GPIOx
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)			GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)	GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC)	GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD)	GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE)	GPIOE_REG_RESET();
	else if(pGPIOx == GPIOF)	GPIOF_REG_RESET();
	else if(pGPIOx == GPIOG)	GPIOG_REG_RESET();
	else if(pGPIOx == GPIOH)	GPIOH_REG_RESET();
	else if(pGPIOx == GPIOI)	GPIOI_REG_RESET();
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Read data from an input pin of the given GPIO port
 *
 * @param[in]         -	base address of the gpio peripheral
 * @param[in]         - number of the gpio pin
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              - none

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t Value;
	Value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1) ;
	return Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             -	Read data from the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - uint16_t (data of 16 pins)
 *
 * @Note              - none

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t Value;
	Value = (uint16_t)(pGPIOx->IDR);
	return Value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Write data to an output pin of the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - number of the gpio pin
 * @param[in]         - value is written to output pin: GPIO_PIN_SET & GPIO_PIN_RESET
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value ==  GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	} else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - Write data to the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - value is written to output port
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR =  Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - Toggle data of the pin in the given port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - number of the gpio pin
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQInteruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if(IRQNumber >= 64 && IRQNumber <96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	} else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if(IRQNumber >= 64 && IRQNumber <96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_IRQPriotrityConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQPriotrityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first let find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}
