#pragma once
#include "mgos_bme680.h"

class BME680
{
public:

    /*
     * Creates the BME680 object for the device with `addr` address
     */
    BME680(uint8_t addr, bool spi = false)
    : _bme(spi ? mgos_bme680_spi_create() : mgos_bme680_i2c_create(addr))
    {
    }

    /*
     * Deletes the object and frees resources.
     */
    ~BME680()
    {
        mgos_bme680_delete(_bme);
    }

    /*
     * Reads the temperature, pressure and humidity in the provided `data` structure.
     * If the device is BMP280, the humidity will be 0.
     */
    int8_t read(struct mgos_bme680_data& data)
    {
        return mgos_bme680_read(_bme, &data);
    }

    
    /*
     * Returns true if a BME680 device is connected
     */
    bool isBME680()
    {
        return mgos_bme680_is_bme680(_bme);
    }
private:
    struct mgos_bme680* _bme;
};
