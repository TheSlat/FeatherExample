#include "bma250x.h"
#include "bma250x_private.h"
#include "atmel_start.h"
#include "stdio.h"

#define	bit_test( val, bitno )		(((val) & ((uint32_t)1 << (bitno))) ? 1 : 0)
#define bit_set( val, bitno )		((val) |= ((uint32_t)1<<(bitno)) )
#define bit_clear( val, bitno )		((val) &= ~((uint32_t)1<<(bitno)) )

static char debugBuf[64];

// static variable to specify accelerometer model number
static int8_t isBMA250e   = -1;
static uint32_t lowPowerMode1 = -1;

/* read a single register */
static inline int32_t bma_readRegister8(const uint8_t REG, uint8_t* data)
{
    return ( i2c_m_sync_cmd_read(&wire, REG, data, 1) == ERR_NONE );
}

/* write a single register */
static inline int32_t bma_writeRegister8(const uint8_t REG, uint8_t val)
{
    struct _i2c_m_msg msg;
    int32_t           ret;
    uint8_t buff[2] = {REG, val};

    msg.addr   = wire.slave_addr;
    msg.len    = 2;
    msg.flags  = I2C_M_STOP;
    msg.buffer = &buff;

    ret = _i2c_m_sync_transfer(&wire.device, &msg);
    acc_writeDelayShort();
    return ret == ERR_NONE;
}

static inline int32_t bma_readContinous(uint8_t STARTING_REG, uint8_t* buf, const uint32_t LEN)
{
    return ( i2c_m_sync_cmd_read(&wire, STARTING_REG, buf, LEN) == ERR_NONE);
}

static void bma_printreg( const uint8_t regNo ) {
    uint8_t regVal = 0;
    i2c_m_sync_cmd_read(&wire, regNo, &regVal, 1);
    sprintf(debugBuf, "REG 0x%02X == 0x%02X\n", regNo, regVal);
    usb_write(debugBuf, strlen(debugBuf));
}

uint32_t acc_fullPower(void)
{
    // set to full power first, since the rest relies on this.
    const uint32_t success = bma_writeRegister8(PMU_LPW, 0);

    // wakeup can take up to 1.8ms, then set WDT
    acc_wakeupLong();
    bma_writeRegister8(INTERFACE_CONFIG, ACC_WDT_50MS);

    if(isBMA250e < 0) {
        uint8_t id;                 // store the returned chip ID

        // get the chip ID and set the hardware variable accordingly
        if( bma_readRegister8(CHIP_ID, &id) ) {
            isBMA250e = (id == BMA250E_ID_VALUE);
        }
    }

    return success;
}

uint32_t acc_lowPower( const uint32_t MODE1 ) {
    // ensure the chip is in full power mode and hardware version is known
    uint32_t success = acc_fullPower();

    if(success) {
        // save the power mode to use
        lowPowerMode1 = MODE1;

        // set up the FIFO to be in STREAM mode
        success &= bma_writeRegister8( FIFO_CONFIG_1, ACC_FIFO_STREAM );

        // First set the low power mode to mode 2 and time based sampling
        if(lowPowerMode1) {
            success &= bma_writeRegister8( PMU_LOW_POWER, (ACC_LP_MODE1 | ACC_LP_EQUAL_TIME) );
        }
        else {
            success &= bma_writeRegister8( PMU_LOW_POWER, (ACC_LP_MODE2 | ACC_LP_EQUAL_TIME) );
        }

        // THEN set to low power mode, with 10Hz sample rate
        success &= bma_writeRegister8( PMU_LPW, (ACC_MODE_LP | ACC_PERIOD) );
    }

    return success;
}

void acc_DeepSuspend(void) {
    bma_writeRegister8( PMU_LPW, 0x20 );
}

uint32_t BMA250e(void) {
    i2c_m_sync_enable(&wire);
    i2c_m_sync_set_slaveaddr(&wire, BMA_I2C_ADDRESS, I2C_M_SEVEN);

    // check chip ID, get from the chip if not yet set.
    if( isBMA250e < 0 ){
        acc_fullPower();
        acc_DeepSuspend();
    }
    return isBMA250e > 0;
}

// Scale the two's-complement (10 bit) result to a positive 10 bit value
// scaled from 0 -> 1023 Resulting gain is 2/512, offset 2
static uint16_t scale( uint16_t value )
{
    if( bit_test( value, 9 ) ){
        value = 512 - ((~value + 1) & 0x1FF);
    }
    else{
        bit_set( value, 9 );
    }

    return value;
}

uint8_t acc_readFIFO(ACCELEROMETER accBuf[], const uint8_t LEN) {
    accelFrame accData;                    // a single frame of data from the accelerometer
    uint8_t count = 0;                     // fifo status
    uint8_t i;                             // the return value and loop control variable

    bma_readRegister8(FIFO_STATUS, &count);

    sprintf(debugBuf, "FIFO-stat: %d, %d\n", (count & 0x7F), (count>>7) );
    usb_write(debugBuf, strlen(debugBuf));

    count &= 0x7F;
    count = (count <= 30 ? count : 30);

    sprintf(debugBuf, "FIFO-rx: %d\n", count );
    usb_write(debugBuf, strlen(debugBuf));

    if(lowPowerMode1) {
        //bma_writeRegister8(PMU_LPW, ACC_PERIOD);
        bma_writeRegister8(SOFT_RESET, BMA250E_RESET_VALUE);
        acc_wakeupLong();
        //         bma_printreg(FIFO_CONFIG_1);
        //         bma_printreg(PMU_LOW_POWER);
        //         bma_printreg(PMU_LPW);
    }

    // read a frame out of the FIFO for each index in the acceleration buffer
    for(i = 0; i < LEN && i < count; ++i) {
        bma_readContinous(FIFO_DATA, &accData, sizeof(accelFrame));

        //DEBUG("  X:%d Y:%d Z:%d", accData.x.value, accData.y.value, accData.z.value);
        //DEBUG(" - new:%d,%d,%d\n", accData.x.newDataFlag, accData.y.newDataFlag, accData.z.newDataFlag);
//         sprintf(debugBuf, "  X:%04X (%d) Y:%04X (%d) Z:%04X (%d)\n", accData.x, accData.x.value, accData.y, accData.y.value, accData.z, accData.z.value);
//         usb_write(debugBuf, strlen(debugBuf));

        // bail if the next sample is not new data.
        if(!accData.x.newDataFlag || !accData.y.newDataFlag || !accData.z.newDataFlag) {
            break;
        }

        // Axis are scaled and changed, on the mPAT the x and z axis are switched
        accBuf[i].x = scale( accData.z.value );
        accBuf[i].y = scale( accData.y.value );

        int16_t rawZ = accData.x.value;
        if( bit_test( rawZ, 9 ) ){
            rawZ |= 0xFC00;
        }
        accBuf[i].z = scale( -rawZ );
    }

    if(lowPowerMode1) {
        //bma_writeRegister8( FIFO_CONFIG_1, ACC_FIFO_STREAM );

        // First set the low power mode to mode 1 and time based sampling
       // bma_writeRegister8( PMU_LOW_POWER, (ACC_LP_MODE1 | ACC_LP_EQUAL_TIME) );
        //bma_writeRegister8( PMU_LPW, (ACC_MODE_LP | ACC_PERIOD) );
        acc_lowPower(1);
    }

    // writing to the FIFO config register will clear the FIFO
    //bma_writeRegister8( FIFO_CONFIG_1, ACC_FIFO_STREAM );
    sprintf(debugBuf, "FIFO-use: %d (%d)\n", i, (i == count) );
    usb_write(debugBuf, strlen(debugBuf));
    return i;
}

void GetAcceleration( ACCELEROMETER *a ) {
    a->all = 0;

    if( acc_fullPower() ) {
        uint8_t haveReadSincePowerOn = 0;   // tracks if we have tried to read yet
        accelAxis axis[3];                  // a single frame of data from the accelerometer

        for( uint16_t count = 10; count; --count ) {
            const uint8_t regNo = XAXIS_LSB;

            if( !bma_readContinous( regNo, axis, sizeof(axis) ) ){
                break;
            }

            // Check each channel for valid data.  Done when all three have data
            if( axis[0].newDataFlag & axis[1].newDataFlag & axis[2].newDataFlag ) {
                if( !haveReadSincePowerOn ) {
                    haveReadSincePowerOn = 1;
                    continue;
                }

                a->x = scale( axis[2].value );
                a->y = scale( axis[1].value );

                int16_t rawZ = axis[0].value;
                if( bit_test( rawZ, 9 ) ){
                    rawZ |= 0xFC00;
                }
                a->z = scale( -rawZ );
                break;
            }
        }
        acc_DeepSuspend();    // turn chip fully off
    }
}
