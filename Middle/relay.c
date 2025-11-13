/*******************************************************************************
 *				 _ _                                             _ _
				|   |                                           (_ _)
				|   |        _ _     _ _   _ _ _ _ _ _ _ _ _ _   _ _
				|   |       |   |   |   | |    _ _     _ _    | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |_ _ _  |   |_ _|   | |   |   |   |   |   | |   |
				|_ _ _ _ _| |_ _ _ _ _ _| |_ _|   |_ _|   |_ _| |_ _|
								(C)2022 Lumi
 * Copyright (c) 2022
 * Lumi, JSC.
 * All Rights Reserved
 *
 * File name: Example.h
 *
 * Description:
 *
 *
 * Last Changed By:  $Author: trungnt $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $April 15, 2022
 *
 * Code sample:
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <main.h>
#include "relay.h"

/******************************************************************************/
/*                     PRIVATE TYPES and DEFINITIONS                         */
/******************************************************************************/


/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/


/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/
/**
 * @func   relayInit
 * @brief
 * @param  None
 * @retval None
 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} RelayMap_t;

static const RelayMap_t relayMap[] = {
    {GPIOE, GPIO_PIN_1},
    {GPIOE, GPIO_PIN_0},
    {GPIOB, GPIO_PIN_9},
    {GPIOD, GPIO_PIN_3},
    {GPIOD, GPIO_PIN_4},
    {GPIOD, GPIO_PIN_5},
    {GPIOD, GPIO_PIN_6},
	{GPIOD, GPIO_PIN_7},
    {GPIOB, GPIO_PIN_3},
    {GPIOB, GPIO_PIN_4},
    {GPIOB, GPIO_PIN_5},
	{GPIOB, GPIO_PIN_6},
};


void relayInit(void)
{
    for (uint8_t i = 0; i < (sizeof(relayMap) / sizeof(RelayMap_t)); i++) {
        HAL_GPIO_WritePin(relayMap[i].port, relayMap[i].pin, GPIO_PIN_RESET);
    }
}


/**
 * @func   setRelayIndex
 * @brief
 * @param  None
 * @retval None
 */
void setRelayIndex(uint8_t idRelay, uint8_t setRelay)
{
    if (idRelay >= (sizeof(relayMap) / sizeof(RelayMap_t))) return;

    HAL_GPIO_WritePin(
        relayMap[idRelay].port,
        relayMap[idRelay].pin,
        (setRelay ? GPIO_PIN_SET : GPIO_PIN_RESET)
    );
}
