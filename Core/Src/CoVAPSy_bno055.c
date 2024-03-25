#include <CoVAPSy_bno055.h>
#include "i2c.h"
// Initialisation du BNO055
void bno055_init(void) {
	uint8_t donnees_Tx_i2c[8];
	uint8_t donnees_Rx_i2c[8];

	donnees_Tx_i2c[0] = 0x3F;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)		;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 1, 1000);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Rx_i2c,	1, 1);

	HAL_Delay(500); //delai 500 ms

	donnees_Tx_i2c[1] = 32;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)	;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	HAL_Delay(2000); //delai 2000 ms

	// A envoyer lors du premier test
	donnees_Tx_i2c[0] = PAGE_SWAP;
	donnees_Tx_i2c[1] = 1;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = ACC_CONF;
	donnees_Tx_i2c[1] = 0x08;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = GYR_CONF_0;
	donnees_Tx_i2c[1] = 0x23;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)	;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = GYR_CONF_1;
	donnees_Tx_i2c[1] = 0;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = MAG_CONF;
	donnees_Tx_i2c[1] = 0x1B;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)	;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = PAGE_SWAP;
	donnees_Tx_i2c[1] = 0;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)	;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = TEMP_SOURCE;
	donnees_Tx_i2c[1] = 0x01;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)	;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = UNIT_SEL;
	donnees_Tx_i2c[1] = 0x01;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)	;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = PWR_MODE;
	donnees_Tx_i2c[1] = 0x00;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)	;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);

	donnees_Tx_i2c[0] = MODE_REG;
	donnees_Tx_i2c[1] = FUSION_MODE;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)	;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 2, 1000);
}

int16_t bno055_lecture_16bits(uint8_t num_registre) {
	uint8_t donnees_Tx_i2c[8];
	uint8_t donnees_Rx_i2c[8];
	donnees_Tx_i2c[0] = num_registre;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Tx_i2c, 1, 1000);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t) ADRESSE_BNO << 1, donnees_Rx_i2c,	2, 1);
	return ((uint16_t)(donnees_Rx_i2c[1]<<8) + donnees_Rx_i2c[0]);
}
uint8_t bno055_lecture_8bits(uint8_t num_registre) {
	uint8_t donnees_Tx_i2c[8];
	uint8_t donnees_Rx_i2c[8];
	donnees_Tx_i2c[0]=num_registre;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)ADRESSE_BNO<<1, donnees_Tx_i2c, 1, 1000);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)ADRESSE_BNO<<1, donnees_Rx_i2c, 1, 1);
	return (donnees_Rx_i2c[0]);

}

