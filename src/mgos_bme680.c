#include <mgos.h>

#include <mgos_i2c.h>
#include <mgos_spi.h>

#include "mgos_bme680.h"
#include "mgos_system.h"

#include "BME680_driver/bme680.h"

const double MGOS_BME680_ERROR = -128.0;

static int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
    //int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    struct mgos_i2c* i2c = mgos_i2c_get_global();
    if (NULL == i2c) {
        LOG(LL_INFO, ("Could not get i2c global instance"));
        return -1;
    }
    bool ok = mgos_i2c_read_reg_n(i2c, dev_id, reg_addr, len, reg_data);
    if(!ok) {LOG(LL_INFO, ("Could not read i2c"));}
    return ok ? 0 : -2;
}

static int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
    //int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    struct mgos_i2c* i2c = mgos_i2c_get_global();
    if (NULL == i2c) {
        LOG(LL_INFO, ("Could not get i2c global instance"));
        return -1;
    }

    bool ok = mgos_i2c_write_reg_n(i2c, dev_id, reg_addr, len, reg_data);
    if(!ok) {LOG(LL_INFO, ("Could not write i2c"));}
    return ok ? 0 : -2;
}

static int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |----------------+---------------------+-------------|
     * | MOSI           | MISO                | Chip Select |
     * |----------------+---------------------|-------------|
     * | (don't care)   | (don't care)        | HIGH        |
     * | (reg_addr)     | (don't care)        | LOW         |
     * | (don't care)   | (reg_data[0])       | LOW         |
     * | (....)         | (....)              | LOW         |
     * | (don't care)   | (reg_data[len - 1]) | LOW         |
     * | (don't care)   | (don't care)        | HIGH        |
     * |----------------+---------------------|-------------|
     */
    //int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    (void) dev_id; /* Not used. Using default spi.cs0_gpio */

    struct mgos_spi* spi = mgos_spi_get_global();
    if (NULL == spi) {
        LOG(LL_INFO, ("Could not get SPI global instance"));
        return -1;
    }

    struct mgos_spi_txn txn;
    memset(&txn, 0, sizeof (txn));
    txn.cs = 0; /* Using default spi.cs0_gpio */
    txn.mode = 0; /* Mode 0 or 3*/
    txn.freq = 1000000;
    txn.hd.tx_data = &reg_addr;
    txn.hd.tx_len = 1;
    txn.hd.rx_data = reg_data;
    txn.hd.rx_len = len;

    if (!mgos_spi_run_txn(spi, false, &txn)) {
        LOG(LL_INFO, ("SPI transaction failed"));
        return -1;
    }

    return BME680_OK;
}

static int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |---------------------+--------------+-------------|
     * | MOSI                | MISO         | Chip Select |
     * |---------------------+--------------|-------------|
     * | (don't care)        | (don't care) | HIGH        |
     * | (reg_addr)          | (don't care) | LOW         |
     * | (reg_data[0])       | (don't care) | LOW         |
     * | (....)              | (....)       | LOW         |
     * | (reg_data[len - 1]) | (don't care) | LOW         |
     * | (don't care)        | (don't care) | HIGH        |
     * |---------------------+--------------|-------------|
     */
    //int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    (void) dev_id; /* Not used. Using default spi.cs0_gpio */

    struct mgos_spi* spi = mgos_spi_get_global();
    if (NULL == spi) {
        LOG(LL_INFO, ("user_spi_write: Could not get SPI global instance"));
        return -1;
    }

    struct mgos_spi_txn txn;
    memset(&txn, 0, sizeof (txn));
    txn.cs = 0; /* Using default spi.cs0_gpio */
    txn.mode = 0; /* Mode 0 or 3*/
    txn.freq = 1000000;

    uint8_t temp_buff[20]; /* Typically not to write more than 10 registers */
    temp_buff[0] = reg_addr;
    temp_buff[1] = reg_data[0];
    if (len >= 2) {
        memcpy(temp_buff + 2, reg_data + 1, len - 1);
    }
    txn.hd.tx_data = temp_buff;
    txn.hd.tx_len = len;

    if (!mgos_spi_run_txn(spi, false, &txn)) {
        LOG(LL_INFO, ("user_spi_write: SPI transaction failed"));
        return -1;
    }

    return BME680_OK;
}

void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
   mgos_msleep(period);
}

struct mgos_bme680 {
    struct bme680_dev dev;
};

static int8_t commonInit(struct mgos_bme680* bme)
{
    int8_t rslt = bme680_init(&bme->dev);
    if (BME680_OK != rslt) {
        LOG(LL_INFO, ("BME680 device not found - %hhd", rslt));
        return rslt;
    }

    /* Recommended mode of operation: Indoor navigation */
    bme->dev.tph_sett.os_hum = BME680_OS_2X;
    bme->dev.tph_sett.os_pres = BME680_OS_4X;
    bme->dev.tph_sett.os_temp= BME680_OS_8X;
    bme->dev.tph_sett.filter = BME680_FILTER_SIZE_3;

    bme->dev.power_mode = BME680_FORCED_MODE;

    /* Set the remaining gas sensor settings and link the heating profile */
    bme->dev.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    bme->dev.gas_sett.heatr_temp = 320; /* degree Celsius */
    bme->dev.gas_sett.heatr_dur = 150; /* milliseconds */

    uint8_t settings_sel = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL
                           | BME680_GAS_SENSOR_SEL;

    rslt = bme680_set_sensor_settings(settings_sel, &bme->dev);
    if (BME680_OK != rslt) {
        LOG(LL_INFO, ("Could not set sensor settings"));
        return rslt;
    }

    rslt = bme680_set_sensor_mode( &bme->dev);
    if (BME680_OK != rslt) {
        LOG(LL_INFO, ("Could not set sensor mode"));
        return rslt;
    }

    /* Get the total measurement duration so as to sleep or wait till the
	 * measurement is complete */
    uint16_t meas_period;
    bme680_get_profile_dur(&meas_period, &bme->dev  );
    user_delay_ms(meas_period); /* Delay till the measurement is ready */
    return BME680_OK;
}

struct mgos_bme680* mgos_bme680_i2c_create(uint8_t addr)
{
    // Is I2C enabled?
    if (!mgos_sys_config_get_i2c_enable()) {
        LOG(LL_INFO, ("I2C is disabled."));
        return NULL;
    }

    struct mgos_bme680* bme = calloc(1, sizeof (struct mgos_bme680));
    if (NULL == bme) {
        LOG(LL_INFO, ("Could not allocate mgos_bme680 structure."));
        return NULL;
    }

    //initialize the structure
    bme->dev.dev_id = addr;
    bme->dev.intf = BME680_I2C_INTF;
    bme->dev.read = user_i2c_read;
    bme->dev.write = user_i2c_write;
    bme->dev.delay_ms = mgos_msleep;

    int8_t rslt = commonInit(bme);
    if (BME680_OK != rslt) {
        free(bme);
        return NULL;
    }

    return bme;
}

struct mgos_bme680* mgos_bme680_spi_create()
{
    // Is I2C enabled?
    if (!mgos_sys_config_get_spi_enable()) {
        LOG(LL_INFO, ("SPI is disabled."));
        return NULL;
    }

    struct mgos_bme680* bme = calloc(1, sizeof (struct mgos_bme680));
    if (NULL == bme) {
        LOG(LL_INFO, ("Could not allocate mgos_bme680 structure."));
        return NULL;
    }

    //initialize the structure
    bme->dev.dev_id = 0;
    bme->dev.intf = BME680_SPI_INTF;
    bme->dev.read = user_spi_read;
    bme->dev.write = user_spi_write;
    bme->dev.delay_ms = mgos_msleep;

    int8_t rslt = commonInit(bme);
    if (BME680_OK != rslt) {
        free(bme);
        return NULL;
    }

    return bme;
}

int8_t mgos_bme680_read(struct mgos_bme680* bme, struct mgos_bme680_data* mdata)
{
    if (NULL == bme) {
        return -1;
    }

    struct bme680_field_data data;

    int8_t rslt = bme680_get_sensor_data( &data, &bme->dev);
    if (BME680_OK == rslt) {
#ifdef BME680_FLOAT_ENABLE
        mdata->temp = comp_data.temperature;
        mdata->press = comp_data.pressure;
        mdata->humid = comp_data.humidity;
        /* Avoid using measurements from an unstable heating setup */
        if(data.status & BME680_GASM_VALID_MSK)
            mdata->gasResist =  data.gas_resistance;
#else
        mdata->temp = data.temperature / 100.0;
        mdata->press = data.pressure / 100.0;
        mdata->humid = data.humidity / 1000.0;
        /* Avoid using measurements from an unstable heating setup */
        if(data.status & BME680_GASM_VALID_MSK)
            mdata->gasResist =  data.gas_resistance;
#endif
    }
    return rslt;
}

void mgos_bme680_delete(struct mgos_bme680* bme)
{
    if (NULL != bme) {
        free(bme);
    }
}

bool mgos_bme680_is_bme680(struct mgos_bme680* bme)
{
    if (NULL != bme) {
        return BME680_CHIP_ID == bme->dev.chip_id;
    }
    return false;
}
