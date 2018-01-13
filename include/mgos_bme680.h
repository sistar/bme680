#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
struct mgos_bme680;

/*
 * `temp` - temperature in degrees Celsius
 * `press` - pressure in Pa
 * `humid` - humidity in %
 */
struct mgos_bme680_data
{
    double temp;
    double press;
    double humid;
    double gasResist;
};

extern const double MGOS_BME680_ERROR;

/*
 * Creates a `struct mgos_bme680` for the device with I2C `addr` address
 * Uses global i2c. If different pins than the default ones are used,
 * the user should define them in mos.yml. Eg.
 * ```
 *  - ["i2c.sda_gpio", 12]
 *  - ["i2c.scl_gpio", 14]
 * ```
 * Returns opaque handle (NULL if an error occurred)
 */
struct mgos_bme680* mgos_bme680_i2c_create(uint8_t addr);

/*
 * Creates a `struct mgos_bme680` for the device
 * Uses global spi with `spi.cs0_gpio`
 * If different pins than the default ones are used,
 * the user should define them in mos.yml. Eg.
 * ```
 *  - ["spi.miso_gpio", 19]
 *  - ["spi.mosi_gpio", 23]
 *  - ["spi.sclk_gpio", 18]
 *  - ["spi.cs0_gpio", 5]
 * ```
 * Returns opaque handle (NULL if an error occurred)
 */
struct mgos_bme680* mgos_bme680_spi_create();

/*
 * Deletes the handle and frees resources.
 */
void mgos_bme680_delete(struct mgos_bme680* bme);

/*
 * Reads the temperature, pressure and humidity in the provided `data` structure.
 * Returns 0 if success
 */
int8_t mgos_bme680_read(struct mgos_bme680* bme, struct mgos_bme680_data* data);



/*
 * Returns true if a BME680 device is connected
 */
bool mgos_bme680_is_bme680(struct mgos_bme680* bme);

#ifdef __cplusplus
}
#endif
