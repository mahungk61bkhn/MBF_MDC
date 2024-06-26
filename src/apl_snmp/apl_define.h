/*
 * apl_define.h
 *
 *  Created on: 8 thg 11, 2023
 *      Author: mobifone
 */

#ifndef APL_DEFINE_H_
#define APL_DEFINE_H_

///0.==================================================
#define RS485_M_Ctr (PORT2.PODR.BIT.B7)
#define RS485_S_Ctr (PORTC.PODR.BIT.B1)
#define FLASH_CE 	(PORTC.PODR.BIT.B4)
///1.==================================================
#define ENABLE               1
#define DISABLE              0
///2.==================================================
#define DELAY_MS           60

///3.==================================================
#define BATT1_VOLT                    ".1.3.6.1.4.1.3902.2800.1.2.3.4.4.1.2.1.1"
#define BATT2_VOLT                    ".1.3.6.1.4.1.3902.2800.1.2.3.4.4.1.2.1.2"
#define BATT3_VOLT                    ".1.3.6.1.4.1.3902.2800.1.2.3.4.4.1.2.1.3"

#define SYS_DC_VOLT                   ".1.3.6.1.4.1.3902.2800.1.2.3.4.1.0"

#define BATT1_CURR                    ".1.3.6.1.4.1.3902.2800.1.2.3.4.5.1.2.1.1"
#define BATT2_CURR                    ".1.3.6.1.4.1.3902.2800.1.2.3.4.5.1.2.1.2"
#define BATT3_CURR                    ".1.3.6.1.4.1.3902.2800.1.2.3.4.5.1.2.1.3"

#define DC_LOAD                       ".1.3.6.1.4.1.3902.2800.1.2.3.4.2.0"

#define SYS_TEMP                      ".1.3.6.1.4.1.3902.2800.1.2.3.10.14.1.2.1.2"

#define POWER_1                       ".1.3.6.1.4.1.3902.2800.1.2.3.4.33.1.2.1.1"
#define POWER_2                       ".1.3.6.1.4.1.3902.2800.1.2.3.4.33.1.2.1.2"
#define POWER_3                       ".1.3.6.1.4.1.3902.2800.1.2.3.4.33.1.2.1.3"
#define POWER_4                       ".1.3.6.1.4.1.3902.2800.1.2.3.4.33.1.2.1.4"
#define POWER_5                       ".1.3.6.1.4.1.3902.2800.1.2.3.4.33.1.2.1.5"

#define TOTAL_RECT_CURR               ".1.3.6.1.4.1.3902.2800.1.2.3.3.2.1.2.1.1"
#define TOTAL_RECT_DC_POWER           ".1.3.6.1.4.1.3902.2800.1.2.3.4.40.0"

#define RECT_STATUS                   ".1.3.6.1.4.1.3902.2800.1.2.3.3.65537.1.2.1.1"

#define TIME_CHARGE_ACU1              ".1.3.6.1.4.1.3902.2800.1.2.3.4.94.1.2.1.1"
#define TIME_DISCHARGE_ACU1           ".1.3.6.1.4.1.3902.2800.1.2.3.4.12.1.2.1.1"
#define SOC1                          ".1.3.6.1.4.1.3902.2800.1.2.3.4.102.1.2.1.1"
#define SOH1                          ".1.3.6.1.4.1.3902.2800.1.2.3.4.101.1.2.1.1"
#define SATTUS_ACU1                   ".1.3.6.1.4.1.3902.2800.1.2.3.4.65559.1.2.1.1"
#define TEMP_ACU1                     ".1.3.6.1.4.1.3902.2800.1.2.3.4.100.1.2.1.1"

#define SOC2                          ".1.3.6.1.4.1.3902.2800.1.2.3.4.102.1.2.1.2"
#define SOH2                          ".1.3.6.1.4.1.3902.2800.1.2.3.4.101.1.2.1.2"
#define SATTUS_ACU2                   ".1.3.6.1.4.1.3902.2800.1.2.3.4.65559.1.2.1.2"
#define TEMP_ACU2                     ".1.3.6.1.4.1.3902.2800.1.2.3.4.100.1.2.1.2"

#define SOC3                          ".1.3.6.1.4.1.3902.2800.1.2.3.4.102.1.2.1.3"
#define SOH3                          ".1.3.6.1.4.1.3902.2800.1.2.3.4.101.1.2.1.3"
#define SATTUS_ACU3                   ".1.3.6.1.4.1.3902.2800.1.2.3.4.65559.1.2.1.3"
#define TEMP_ACU3                     ".1.3.6.1.4.1.3902.2800.1.2.3.4.100.1.2.1.3"

#define SOC4                          ".1.3.6.1.4.1.3902.2800.1.2.3.4.102.1.2.1.4"
#define SOH4                          ".1.3.6.1.4.1.3902.2800.1.2.3.4.101.1.2.1.4"
#define SATTUS_ACU4                   ".1.3.6.1.4.1.3902.2800.1.2.3.4.65559.1.2.1.4"
#define TEMP_ACU4                     ".1.3.6.1.4.1.3902.2800.1.2.3.4.100.1.2.1.4"

#define AC_VOLT                       ".1.3.6.1.4.1.3902.2800.1.2.3.2.15.1.2.1.1"
#define AC_CURR                       ".1.3.6.1.4.1.3902.2800.1.2.3.2.16.1.2.1.1"

#define BAT1_CAP                      ".1.3.6.1.4.1.3902.2800.1.2.3.4.24.1.2.1.1"
#define BAT2_CAP                      ".1.3.6.1.4.1.3902.2800.1.2.3.4.24.1.2.1.2"
#define BAT3_CAP                      ".1.3.6.1.4.1.3902.2800.1.2.3.4.24.1.2.1.3"

///
#define OID_STRING_2        ".1.3.6.1.4.1.3902.2800.1.2.1.1.13.0"
#define OID_STRING_1        ".1.3.6.1.4.1.3902.2800.1.2.2.4.2.0"

#define COMMUNITY_STRING        "public"
#define COMMUNITY_STRING_SET    "private"

#endif /* APL_DEFINE_H_ */
