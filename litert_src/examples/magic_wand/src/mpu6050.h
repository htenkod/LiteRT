/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    mpu6050.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "MPU6050_Initialize" and "MPU6050_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "MPU6050_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _MPU6050_H
#define _MPU6050_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    /* Application's state machine's initial state. */
    MPU6050_STATE_INIT=0,
    MPU6050_STATE_SERVICE_TASKS,
    /* TODO: Define states used by the application state machine. */

} MPU6050_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    MPU6050_STATES state;

    /* TODO: Define any additional data used by the application. */

} MPU6050_DATA;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MPU6050_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    MPU6050_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    MPU6050_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void MPU6050_Initialize ( void );


/*******************************************************************************
  Function:
    void MPU6050_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    MPU6050_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void MPU6050_Tasks( void );



/**
 * @defgroup mpu6050_example_driver mpu6050 example driver function
 * @brief    mpu6050 example driver modules
 * @ingroup  mpu6050_driver
 * @{
 */

/**
 * @brief mpu6050 basic example default definition
 */
#define MPU6050_BASIC_DEFAULT_CLOCK_SOURCE                   MPU6050_CLOCK_SOURCE_PLL_X_GYRO           /**< gyro pll x */
#define MPU6050_BASIC_DEFAULT_RATE                           50                                        /**< 50Hz */
#define MPU6050_BASIC_DEFAULT_LOW_PASS_FILTER                MPU6050_LOW_PASS_FILTER_3                 /**< low pass filter 3 */
#define MPU6050_BASIC_DEFAULT_CYCLE_WAKE_UP                  MPU6050_BOOL_FALSE                        /**< disable cycle wake up */
#define MPU6050_BASIC_DEFAULT_WAKE_UP_FREQUENCY              MPU6050_WAKE_UP_FREQUENCY_1P25_HZ         /**< 1.25Hz */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_PIN_LEVEL            MPU6050_PIN_LEVEL_LOW                     /**< low level */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_PIN_TYPE             MPU6050_PIN_TYPE_PUSH_PULL                /**< push pull */
#define MPU6050_BASIC_DEFAULT_ACCELEROMETER_RANGE            MPU6050_ACCELEROMETER_RANGE_2G            /**< 2g */
#define MPU6050_BASIC_DEFAULT_GYROSCOPE_RANGE                MPU6050_GYROSCOPE_RANGE_2000DPS           /**< 2000dps */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_MOTION               MPU6050_BOOL_FALSE                        /**< disable motion */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_FIFO_OVERFLOW        MPU6050_BOOL_FALSE                        /**< disable fifo overflow */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_DMP                  MPU6050_BOOL_FALSE                        /**< disable dmp */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_I2C_MAST             MPU6050_BOOL_FALSE                        /**< disable i2c master */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_DATA_READY           MPU6050_BOOL_FALSE                        /**< disable data ready */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_LATCH                MPU6050_BOOL_TRUE                         /**< enable latch */
#define MPU6050_BASIC_DEFAULT_INTERRUPT_READ_CLEAR           MPU6050_BOOL_TRUE                         /**< enable interrupt read clear */
#define MPU6050_BASIC_DEFAULT_EXTERN_SYNC                    MPU6050_EXTERN_SYNC_INPUT_DISABLED        /**< extern sync input disable */
#define MPU6050_BASIC_DEFAULT_FSYNC_INTERRUPT                MPU6050_BOOL_FALSE                        /**< disable fsync interrupt */
#define MPU6050_BASIC_DEFAULT_FSYNC_INTERRUPT_LEVEL          MPU6050_PIN_LEVEL_LOW                     /**< low level */
#define MPU6050_BASIC_DEFAULT_IIC_MASTER                     MPU6050_BOOL_FALSE                        /**< disable iic master */
#define MPU6050_BASIC_DEFAULT_IIC_BYPASS                     MPU6050_BOOL_FALSE                        /**< disable iic bypass */

/**
 * @brief     basic example init
 * @param[in] addr_pin iic device address
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      none
 */
uint8_t mpu6050_basic_init(mpu6050_address_t addr_pin);

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t mpu6050_basic_deinit(void);

/**
 * @brief      basic example read
 * @param[out] *g pointer to a converted data buffer
 * @param[out] *dps pointer to a converted data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mpu6050_basic_read(float g[3], float dps[3]);

/**
 * @brief      basic example read temperature
 * @param[out] *degrees pointer to a converted data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read temperature failed
 * @note       none
 */
uint8_t mpu6050_basic_read_temperature(float *degrees);


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _MPU6050_H */

/*******************************************************************************
 End of File
 */

