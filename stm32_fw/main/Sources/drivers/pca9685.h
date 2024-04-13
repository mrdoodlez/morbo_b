#ifndef _PCA9685_H_
#define _PCA9685_H_

int PCA9685_Init(uint8_t i2cDev, uint8_t i2cAddr, uint8_t prescale);
void PCA9685_reset();
void PCA9685_Sleep();
void PCA9685_Wakeup();
void PCA9685_Wakeup();
void PCA9685_SetPWMFreq(float freq);
void PCA9685_SetOutputMode(uint8_t totempole);
uint16_t PCA9685_GetPWM(uint8_t num, uint8_t off);
uint8_t PCA9685_SetPWM(uint8_t num, uint16_t on, uint16_t off);
void PCA9685_SetPin(uint8_t num, uint16_t val, uint8_t invert);
uint8_t PCA9685_ReadPrescale(void);
void PCA9685_WriteMicroseconds(uint8_t num, uint16_t Microseconds);
uint32_t PCA9685_GetOscillatorFrequency(void);
void PCA9685_SetOscillatorFrequency(uint32_t freq);


#endif //_PCA9685_H_
