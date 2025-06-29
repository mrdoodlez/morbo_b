#include "main.h"
#include "pca9685.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c.h"

#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */

// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */

// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1 0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04  /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08     /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10   /**< Output logic state inverted */

#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

#define PCA9685_NUM_CHANNELS 16

/******************************************************************************/

static struct
{
    uint8_t i2cAddr;
    uint8_t i2c;
    uint32_t oscillator_freq;

    uint16_t us[PCA9685_NUM_CHANNELS];
} _ctx;

/******************************************************************************/

inline static uint8_t PCA9685_Read8(uint8_t addr)
{
    uint8_t val;
    if (I2C_Read(_ctx.i2c, _ctx.i2cAddr, addr, I2C_RegAddrLen_8, &val, 1) != 1)
        ; // TODO: handle error

    return val;
}

inline static void PCA9685_Write8(uint8_t addr, uint8_t val)
{
    if (I2C_Write(_ctx.i2c, _ctx.i2cAddr, addr, I2C_RegAddrLen_8, &val, 1) != 1)
        ; // TODO: handle error
}

/******************************************************************************/

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
void PCA9685_SetExtClk(uint8_t prescale)
{
    uint8_t oldmode = PCA9685_Read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
    PCA9685_Write8(PCA9685_MODE1, newmode);                     // go to sleep, turn off internal oscillator

    // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
    // use the external clock.
    PCA9685_Write8(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

    PCA9685_Write8(PCA9685_PRESCALE, prescale); // set the prescaler

    vTaskDelay(5);
    // clear the SLEEP bit to start
    PCA9685_Write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);
}

/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
void PCA9685_Reset()
{
    PCA9685_Write8(PCA9685_MODE1, MODE1_RESTART);
    vTaskDelay(10);
}

/*!
 *  @brief  Setups the I2C interface and hardware
 *  @param  prescale
 *          Sets External Clock (Optional)
 *  @return true if successful, otherwise false
 */
int PCA9685_Init(uint8_t i2cDev, uint8_t i2cAddr, uint8_t prescale)
{
    _ctx.i2cAddr = i2cAddr;
    _ctx.i2c = i2cDev;

    PCA9685_Reset();

    // set the default internal frequency
    PCA9685_SetOscillatorFrequency(FREQUENCY_OSCILLATOR);

    if (prescale)
    {
        PCA9685_SetExtClk(prescale);
    }
    else
    {
        // set a default frequency
        PCA9685_SetPWMFreq(1000);
    }

    return 0;
}

/*!
 *  @brief  Puts board into sleep mode
 */
void PCA9685_Sleep()
{
    uint8_t awake = PCA9685_Read8(PCA9685_MODE1);
    uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
    PCA9685_Write8(PCA9685_MODE1, sleep);
    vTaskDelay(5); // wait until cycle ends for sleep to be active
}

/*!
 *  @brief  Wakes board from sleep
 */
void PCA9685_Wakeup()
{
    uint8_t sleep = PCA9685_Read8(PCA9685_MODE1);
    uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
    PCA9685_Write8(PCA9685_MODE1, wakeup);
}

/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
 *  @param  freq Floating point frequency that we will attempt to match
 */
void PCA9685_SetPWMFreq(float freq)
{
    // Range output modulation frequency is dependant on oscillator
    if (freq < 1)
        freq = 1;
    if (freq > 3500)
        freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

    float prescaleval = ((_ctx.oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN)
        prescaleval = PCA9685_PRESCALE_MIN;
    if (prescaleval > PCA9685_PRESCALE_MAX)
        prescaleval = PCA9685_PRESCALE_MAX;
    uint8_t prescale = (uint8_t)prescaleval;

    uint8_t oldmode = PCA9685_Read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
    PCA9685_Write8(PCA9685_MODE1, newmode);                     // go to sleep
    PCA9685_Write8(PCA9685_PRESCALE, prescale);                 // set the prescaler
    PCA9685_Write8(PCA9685_MODE1, oldmode);
    vTaskDelay(5);
    // This sets the MODE1 register to turn on auto increment.
    PCA9685_Write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull / totempole.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
void PCA9685_SetOutputMode(uint8_t totempole)
{
    uint8_t oldmode = PCA9685_Read8(PCA9685_MODE2);
    uint8_t newmode;
    if (totempole)
    {
        newmode = oldmode | MODE2_OUTDRV;
    }
    else
    {
        newmode = oldmode & ~MODE2_OUTDRV;
    }
    PCA9685_Write8(PCA9685_MODE2, newmode);
}

/*!
 *  @brief  Reads set Prescale from PCA9685
 *  @return prescale value
 */
uint8_t PCA9685_ReadPrescale(void)
{
    return PCA9685_Read8(PCA9685_PRESCALE);
}

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  off If true, returns PWM OFF value, otherwise PWM ON
 *  @return requested PWM output value
 */
uint16_t PCA9685_GetPWM(uint8_t num, uint8_t off)
{
    uint16_t val;
    uint8_t regAddr = PCA9685_LED0_ON_L + 4 * num + off ? 2 : 0;
    if (I2C_Read(_ctx.i2c, _ctx.i2cAddr, regAddr, I2C_RegAddrLen_8, (uint8_t *)&val, 2) != 2)
        ; // TODO: handle error

    return val;
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 *  @return 0 if successful, otherwise 1
 */
uint8_t PCA9685_SetPWM(uint8_t num, uint16_t on, uint16_t off)
{
    uint8_t buffer[5];
    buffer[0] = PCA9685_LED0_ON_L + 4 * num;
    buffer[1] = on;
    buffer[2] = on >> 8;
    buffer[3] = off;
    buffer[4] = off >> 8;

    return I2C_Write(_ctx.i2c, _ctx.i2cAddr, 0,
                     I2C_RegAddrLen_0, buffer, 5) == 5
               ? 0
               : -1;
}

/*!
 *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
 * on/off tick placement and properly handles a zero value as completely off and
 * 4095 as completely on.  Optional invert parameter supports inverting the
 * pulse for sinking to ground.
 *   @param  num One of the PWM output pins, from 0 to 15
 *   @param  val The number of ticks out of 4096 to be active, should be a value
 * from 0 to 4095 inclusive.
 *   @param  invert If true, inverts the output, defaults to 'false'
 */
void PCA9685_SetPin(uint8_t num, uint16_t val, uint8_t invert)
{
    // Clamp value between 0 and 4095 inclusive.
    if (val > 4095)
        val = 4095;
    if (invert)
    {
        if (val == 0)
        {
            // Special value for signal fully on.
            PCA9685_SetPWM(num, 4096, 0);
        }
        else if (val == 4095)
        {
            // Special value for signal fully off.
            PCA9685_SetPWM(num, 0, 4096);
        }
        else
        {
            PCA9685_SetPWM(num, 0, 4095 - val);
        }
    }
    else
    {
        if (val == 4095)
        {
            // Special value for signal fully on.
            PCA9685_SetPWM(num, 4096, 0);
        }
        else if (val == 0)
        {
            // Special value for signal fully off.
            PCA9685_SetPWM(num, 0, 4096);
        }
        else
        {
            PCA9685_SetPWM(num, 0, val);
        }
    }
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
 * microseconds, output is not precise
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
 */
void PCA9685_WriteMicroseconds(uint8_t num, uint16_t Microseconds)
{
    if (_ctx.us[num] == Microseconds)
        return;

    double pulse = Microseconds;
    double pulselength;
    pulselength = 1000000; // 1,000,000 us per second

    // Read prescale
    uint16_t prescale = PCA9685_ReadPrescale();

    // Calculate the pulse for PWM based on Equation 1 from the datasheet section
    // 7.3.5
    prescale += 1;
    pulselength *= prescale;
    pulselength /= _ctx.oscillator_freq;

    pulse /= pulselength;

    PCA9685_SetPWM(num, 0, pulse);

    _ctx.us[num] = Microseconds;
}

/*!
 *  @brief  Getter for the internally tracked oscillator used for freq
 * calculations
 *  @returns The frequency the PCA9685 thinks it is running at (it cannot
 * introspect)
 */
uint32_t PCA9685_GetOscillatorFrequency(void)
{
    return _ctx.oscillator_freq;
}

/*!
 *  @brief Setter for the internally tracked oscillator used for freq
 * calculations
 *  @param freq The frequency the PCA9685 should use for frequency calculations
 */
void PCA9685_SetOscillatorFrequency(uint32_t freq)
{
    _ctx.oscillator_freq = freq;
}