
#include "dth11_driver.h"
#include "hal/nrf_gpio.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/***************************** Function declaration*****************************/

static void DelayUSec(int usec);
static void Data_SetInput(const struct device *gpio_dev);
static uint32_t Data_GetVal(const struct device *gpio_dev);
static void Data_SetOutput(const struct device *gpio_dev);
static void Data_ClrVal(const struct device *gpio_dev);
static void Data_setVal(const struct device *gpio_dev);
static void DelayMSec(int usec);

/********************************************************************************/

/*******************************************************************************
 * @brief Function for configure specific pin as INPUT with Pull-UP
 *
 * @param[in] gpio_dev Pointer to the driver device structure.
 *
 * @retval    None.
 *******************************************************************************/
static void Data_SetInput(const struct device *gpio_dev)
{

  gpio_pin_configure(gpio_dev, DHT11_PIN, GPIO_INPUT | GPIO_PULL_UP);
}

/*******************************************************************************
 * @brief Function for read specific pin state
 *
 * @param[in] gpio_dev Pointer to the driver device structure.
 *
 * @retval    pin state 1 or 0.
 *******************************************************************************/
static uint32_t Data_GetVal(const struct device *gpio_dev)
{

  return nrf_gpio_pin_read(DHT11_PIN);
}

/*******************************************************************************
 * @brief Function for set OUTPUT and make HIGH specific pin
 *
 * @param[in] gpio_dev Pointer to the driver device structure.
 *
 * @retval    None.
 *******************************************************************************/
static void Data_SetOutput(const struct device *gpio_dev)
{

  gpio_pin_configure(gpio_dev, DHT11_PIN, GPIO_OUTPUT);
  gpio_pin_set(gpio_dev, DHT11_PIN, HIGH);
}

/*******************************************************************************
 * @brief Function for make LOW specific pin
 *
 * @param[in] gpio_dev Pointer to the driver device structure.
 *
 * @retval    None.
 *******************************************************************************/
static void Data_ClrVal(const struct device *gpio_dev)
{

  gpio_pin_set(gpio_dev, DHT11_PIN, LOW);
}

/*******************************************************************************
 * @brief Function for make High specific pin
 *
 * @param[in] gpio_dev Pointer to the driver device structure.
 *
 * @retval    None.
 *******************************************************************************/
static void Data_setVal(const struct device *gpio_dev)
{

  gpio_pin_set(gpio_dev, DHT11_PIN, HIGH);
}

/*******************************************************************************
 * @brief Function for generate microsecond delay
 *
 * @param[in] usec variable for store microsecond delay argument.
 *
 * @retval    None.
 *******************************************************************************/
static void DelayUSec(int usec)
{

  nrf_delay_us(usec);
}

/*******************************************************************************
 * @brief Function for generate millisecond delay
 *
 * @param[in] msec variable for store millisecond delay argument.
 *
 * @retval    None.
 *******************************************************************************/
static void DelayMSec(int msec)
{

  nrf_delay_ms(msec);
}

/*******************************************************************************
 * @brief Function for initialise DHT11 sensor and Read the temperarture and humidity
 *
 * @param[in] temperatureCentigrade  pointer variable for store temperature data.
 * @param[in] humidityCentipercent   pointer variable for store humidity data
 *
 * @retval      0=>  DHT11_OK,         //!< OK
 *              1=>  DHT11_NO_PULLUP   //!< no pull-up present
 *              2=>  DHT11_NO_ACK_0    //< no 0 acknowledge detected
 *              3=>  DHT11_NO_ACK_1    //!< no 1 acknowledge detected
 *              4=>  DHT11_NO_DATA_0   //< low level expected during data transmission
 *              5=>  DHT11_NO_DATA_1   //< high level expected during data transmission
 *              6=>  DHT11_BAD_CRC     //< bad CRC
 *******************************************************************************/
DHTxx_ErrorCode DHTxx_Read(uint16_t *temperatureCentigrade, uint16_t *humidityCentipercent)
{
  int cntr;
  int loopBits;
  uint8_t buffer[5];
  int i;
  int data;
  const struct device *gpio_dev = device_get_binding(GPIO_NAME);

  /* init buffer */
  for (i = 0; i < sizeof(buffer); i++)
  {
    buffer[i] = 0;
  }

  /* set to input and check if the signal gets pulled up */
  Data_SetInput(gpio_dev);
  DelayUSec(50);
  if (Data_GetVal(gpio_dev) == 0)
  {
    return DHT11_NO_PULLUP;
  }

  /* send start signal */
  Data_SetOutput(gpio_dev);
  Data_ClrVal(gpio_dev);
  DelayMSec(20); /* keep signal low for at least 18 ms */
  Data_SetInput(gpio_dev);
  DelayUSec(50);

  /* check for acknowledge signal */
  if (Data_GetVal(gpio_dev) != 0)
  { /* signal must be pulled low by the sensor */
    return DHT11_NO_ACK_0;
  }
  /* wait max 100 us for the ack signal from the sensor */
  cntr = 18;
  while (Data_GetVal(gpio_dev) == 0)
  { /* wait until signal goes up */
    DelayUSec(5);
    if (--cntr == 0)
    {
      return DHT11_NO_ACK_1; /* signal should be up for the ACK here */
    }
  }
  /* wait until it goes down again, end of ack sequence */
  cntr = 18;
  while (Data_GetVal(gpio_dev) != 0)
  { /* wait until signal goes down */
    DelayUSec(5);
    if (--cntr == 0)
    {
      return DHT11_NO_ACK_0; /* signal should be down to zero again here */
    }
  }
  /* now read the 40 bit data */
  i = 0;
  data = 0;
  loopBits = 40;
  do
  {
    cntr = 11; /* wait max 55 us */
    while (Data_GetVal(gpio_dev) == 0)
    {
      DelayUSec(5);
      if (--cntr == 0)
      {
        return DHT11_NO_DATA_0;
      }
    }
    cntr = 15; /* wait max 75 us */
    while (Data_GetVal(gpio_dev) != 0)
    {
      DelayUSec(5);
      if (--cntr == 0)
      {
        return DHT11_NO_DATA_1;
      }
    }
    data <<= 1; /* next data bit */
    if (cntr < 10)
    { /* data signal high > 30 us ==> data bit 1 */
      data |= 1;
    }
    if ((loopBits & 0x7) == 1)
    { /* next byte */
      buffer[i] = data;
      i++;
      data = 0;
    }
  } while (--loopBits != 0);

  /* now we have the 40 bit (5 bytes) data:
   * byte 1: humidity integer data
   * byte 2: humidity decimal data (not used for DTH11, always zero)
   * byte 3: temperature integer data
   * byte 4: temperature fractional data (not used for DTH11, always zero)
   * byte 5: checksum, the sum of byte 1 + 2 + 3 + 4
   */
  /* test CRC */
  if ((uint8_t)(buffer[0] + buffer[1] + buffer[2] + buffer[3]) != buffer[4])
  {
    return DHT11_BAD_CRC;
  }

  /* store data values for caller */
  *humidityCentipercent = ((int)buffer[0]) * 100 + buffer[1];
  *temperatureCentigrade = ((int)buffer[2]) * 100 + buffer[3];
  nrf_delay_ms(2000);

  return DHT11_OK;
}
