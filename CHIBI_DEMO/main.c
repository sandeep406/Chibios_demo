/*
    PLAY Embedded demos - Copyright (C) 2014-2017 Rocco Marco Guglielmi

    This file is part of PLAY Embedded demos.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 *  Tested under ChibiOS 17.6.x, Project version 2.0.
 *  Please open readme.txt for changelog.
 */

#include "ch.h"
#include "hal.h"

#include "math.h"

#include "chprintf.h"
#include "magnet.h"
#include "stm32Gyro.h"
#include "usertypes.h"

#define NUM_OF_DEMOS                             3

bool btn_flag = FALSE;
static uint32_t demo_mode = 0;
static BaseSequentialStream * chp = (BaseSequentialStream*) &SD2;
static char axes[3] = {'X', 'Y', 'Z'};
static GYRODataframe_t gdata;
static ACCELDataframe_t adata;
static COMPDataframe_t cdata;

/*===========================================================================*/
/* I2C Configurations                                                        */
/*===========================================================================*/

static const I2CConfig i2c_cfg = {
  OPMODE_I2C,
  400000,
  FAST_DUTY_CYCLE_2,
};

/*===========================================================================*/
/* SPI Configurations                                                        */
/*===========================================================================*/

static const SPIConfig spi_cfg = {
  NULL,
  GPIOE,                                          /*   port of CS   */
  GPIOE_L3GD20_CS,                                /*   pin of CS    */
  SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA,     /*   CR1 register */
};

/*===========================================================================*/
/* Common functions                                                          */
/*===========================================================================*/

static int32_t ftomod(float value) {

  if (value >= 0)
    return (int32_t) value;
  else
    return (int32_t) -1 * value;
}

static uint32_t ftodp(float value) {

  if (value >= 0)
    return (uint32_t) ((value - ftomod (value)) * 1000);
  else
    return (uint32_t) ((-value - ftomod (value)) * 1000);
}

static char ftosign(float value) {

  if (value >= 0)
    return '+';
  else
    return '-';
}
/*===========================================================================*/
/* Main code                                                                 */
/*===========================================================================*/

/*
 * Blinker thread #1.
 */
static THD_WORKING_AREA(waThread1, 512);
static THD_FUNCTION(Thread1, arg) {
  unsigned ii;
  uint32_t low_val, high_val;
  (void)arg;
  chRegSetThreadName("MEMS");
  while (true) {
    btn_flag = FALSE;
    switch(demo_mode) {
    default:
    case 0:
        chprintf(chp,"Gyroscope\n\r");
      /*
       * Starting SPID1 with proper configuration
       */
      spiStart(&SPID1, &spi_cfg);

      /*
       * Writing L3GD20 registers enabling MEMS functionalities
       */
      l3gd20WriteRegister(&SPID1, L3GD20_AD_CTRL_REG1, L3GD20_AE_XYZ |
                          L3GD20_PM_SLEEP_NORMAL | L3GD20_ODR_380Hz_Fc_20);
      l3gd20WriteRegister(&SPID1, L3GD20_AD_CTRL_REG4, L3GD20_BDU_CONTINOUS |
                          L3GD20_End_LITTLE);

      gdata.full_scale = 500;
      gdata.sensitivity = L3GD20_SENS_500DPS;
      while(!btn_flag) {
          chprintf(chp, "GYROSCOPE L3GD20 data:\r\n");
        /*
         * Getting data
         */
        for(ii = 0; ii< GYRO_AXES; ii++){
          low_val = l3gd20ReadRegister(&SPID1, L3GD20_AD_OUT_X_L + 2 * ii);
          high_val = l3gd20ReadRegister(&SPID1, L3GD20_AD_OUT_X_H + 2 * ii);
          gdata.raw[ii] = (high_val << 8) + low_val;
          gdata.scaled[ii] = ((float) gdata.raw[ii]) / gdata.sensitivity;
          /*
           * Printing data on a sequential stream (Serial over USB)
           */
            chprintf(chp, "%c:%c%d.%.3d deg/s\r\n",axes[ii], ftosign(gdata.scaled[ii]),
                     ftomod(gdata.scaled[ii]), ftodp(gdata.scaled[ii]));
        }
        chThdSleepMilliseconds(150);
      }
      /*
       * Writing L3GD20 registers disabling MEMS
       */
      l3gd20WriteRegister(&SPID1, L3GD20_AD_CTRL_REG1,
                          L3GD20_PM_SLEEP_NORMAL | L3GD20_AE_DISABLED);
      /*
       * Stopping SPID1
       */
      spiStop(&SPID1);
      break;
    case 1:
        chprintf(chp,"Accelerometer\n\r");
      /*
       * Starting I2C1 with proper configuration
       */
      i2cStart(&I2CD1, &i2c_cfg);
      /*
       * Writing LSM303DLHC accelerometer sub-registers registers enabling MEMS
       * functionalities
       */
      lsm303dlhcWriteRegister(&I2CD1, LSM303DLHC_SAD_ACCEL,
                                  LSM303DLHC_SUB_ACC_CTRL_REG1,
                                  LSM303DLHC_ACC_ODR_400Hz |
                                  LSM303DLHC_ACC_PM_NORMAL |
                                  LSM303DLHC_ACC_AE_XYZ,
                                  NULL);
      lsm303dlhcWriteRegister(&I2CD1, LSM303DLHC_SAD_ACCEL,
                                  LSM303DLHC_SUB_ACC_CTRL_REG4,
                                  LSM303DLHC_ACC_BDU_BLOCKED |
                                  LSM303DLHC_ACC_End_LITTLE |
                                  LSM303DLHC_ACC_FS_4G |
                                  LSM303DLHC_ACC_HR_Disabled,
                                  NULL);
      adata.full_scale = 4 * 9.8;
      adata.sensitivity = LSM303DLHC_ACC_SENS_4G;
      while(!btn_flag) {
          chprintf(chp, "ACCELEROMETER LSM303DLHC data:\r\n");
        /*
         * Getting data
         */
        for(ii = 0; ii< ACCEL_AXES; ii++){
          low_val = lsm303dlhcReadRegister(&I2CD1, LSM303DLHC_SAD_ACCEL,
                                          LSM303DLHC_SUB_ACC_OUT_X_L + 2 * ii,
                                          NULL);
          high_val = lsm303dlhcReadRegister(&I2CD1, LSM303DLHC_SAD_ACCEL,
                                            LSM303DLHC_SUB_ACC_OUT_X_H + 2 * ii,
                                            NULL);
          adata.raw[ii] = ((high_val << 8) + low_val);
          adata.scaled[ii] = ((float) adata.raw[ii]) / adata.sensitivity;
          /*
           * Printing data on a sequential stream (Serial over USB)
           */
            chprintf(chp, "%c:%c%d.%.3d m/s^2\r\n",axes[ii], ftosign(adata.scaled[ii]),
                 ftomod(adata.scaled[ii]), ftodp(adata.scaled[ii]));
        }
      }
      /*
       * Writing LSM303DLHC accelerometer sub-registers registers disabling MEMS
       */
      lsm303dlhcWriteRegister(&I2CD1,  LSM303DLHC_SAD_ACCEL,
                              LSM303DLHC_SUB_ACC_CTRL_REG1,
                              LSM303DLHC_ACC_ODR_PD, NULL);
      /*
       * Stopping I2CD1
       */
      i2cStop(&I2CD1);
      break;
    case 2:
        chprintf(chp,"Compass\n\r");
      /*
       * Starting I2C1 with proper configuration
       */
      i2cStart(&I2CD1, &i2c_cfg);
      /*
       * Writing LSM303DLHC compass sub-registers registers enabling MEMS
       * functionalities
       */
      lsm303dlhcWriteRegister(&I2CD1, LSM303DLHC_SAD_COMPASS,
                              LSM303DLHC_SUB_COMP_CRA_REG,
                              LSM303DLHC_COMP_ODR_30_Hz,
                              NULL);
      lsm303dlhcWriteRegister(&I2CD1,  LSM303DLHC_SAD_COMPASS,
                              LSM303DLHC_SUB_COMP_CRB_REG,
                              LSM303DLHC_COMP_FS_2_5_GA,
                              NULL);
      lsm303dlhcWriteRegister(&I2CD1,  LSM303DLHC_SAD_COMPASS,
                              LSM303DLHC_SUB_COMP_MR_REG,
                              LSM303DLHC_COMP_WM_CONTINUOS,
                              NULL);
      cdata.full_scale = 2.5;
      cdata.sensitivity_xy = LSM303DLHC_COMP_SENS_XY_2_5GA;
      cdata.sensitivity_z = LSM303DLHC_COMP_SENS_Z_2_5GA;
      while(!btn_flag) {
          chprintf(chp, "COMPASS LSM303DLHC data:\r\n");
        /*
         * Getting data
         */
        low_val = lsm303dlhcReadRegister(&I2CD1,LSM303DLHC_SAD_COMPASS,
                                         LSM303DLHC_SUB_COMP_OUT_X_L, NULL);
        high_val = lsm303dlhcReadRegister(&I2CD1, LSM303DLHC_SAD_COMPASS,
                                          LSM303DLHC_SUB_COMP_OUT_X_H, NULL);
        cdata.raw[0] = (high_val << 8) + low_val;
        low_val = lsm303dlhcReadRegister(&I2CD1, LSM303DLHC_SAD_COMPASS,
                                         LSM303DLHC_SUB_COMP_OUT_Y_L, NULL);
        high_val = lsm303dlhcReadRegister(&I2CD1,
                                          LSM303DLHC_SAD_COMPASS,
                                          LSM303DLHC_SUB_COMP_OUT_Y_H, NULL);
        cdata.raw[1] = (high_val << 8) + low_val;
        low_val = lsm303dlhcReadRegister(&I2CD1, LSM303DLHC_SAD_COMPASS,
                                         LSM303DLHC_SUB_COMP_OUT_Z_L, NULL);
        high_val = lsm303dlhcReadRegister(&I2CD1, LSM303DLHC_SAD_COMPASS,
                                          LSM303DLHC_SUB_COMP_OUT_Z_H, NULL);
        cdata.raw[2] = (high_val << 8) + low_val;
        for(ii = 0; ii < COMP_AXES; ii++){
          if(ii < COMP_AXES - 1)
            cdata.scaled[ii] = ((float) cdata.raw[ii]) / cdata.sensitivity_xy;
          else
            cdata.scaled[ii] = ((float) cdata.raw[ii]) / cdata.sensitivity_z;
          /*
           * Printing data on a sequential stream (Serial over USB)
           */
            chprintf(chp, "%c:%c%d.%.3d Gauss\r\n",axes[ii], ftosign(cdata.scaled[ii]),
                     ftomod(cdata.scaled[ii]), ftodp(cdata.scaled[ii]));
        }
      }
      /*
       * Writing LSM303DLHC compass sub-registers registers disabling MEMS
       */
      lsm303dlhcWriteRegister(&I2CD1, LSM303DLHC_SAD_COMPASS,
                              LSM303DLHC_SUB_COMP_MR_REG,
                              LSM303DLHC_COMP_WM_SLEEP,
                              NULL);
      /*
       * Stopping I2CD1
       */
      i2cStop(&I2CD1);
      break;
    }
  chThdSleepMilliseconds(100);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();


  sdStart(&SD2, NULL);
  /*
       * Starting Serial Driver 2 with default configuration.
       */
       palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7)); //tx for uart2
       palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7)); // rx for uart2

  chThdSleepMilliseconds(2000);
  /*
   * Creates the example threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);

  /*
   * Normal main() thread activity: button checking.
   */
  while (true) {
    if(palReadPad(GPIOA, GPIOA_BUTTON) == PAL_HIGH){
      chThdSleepMilliseconds(50);
      if(palReadPad(GPIOA, GPIOA_BUTTON) == PAL_LOW){
        btn_flag = TRUE;
        demo_mode = (demo_mode + 1) % NUM_OF_DEMOS;
        chThdSleepMilliseconds(50);
      }
    }
    chThdSleepMilliseconds(5);
  }
}
