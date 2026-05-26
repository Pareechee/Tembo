#ifndef __CONFIG_H
#define __CONFIG_H

//#define DEBUG_ON

#define VBAT_SHUTDOWN_THRESHOLD  0x98  // ~10,2 V §á§à§â§à§Ô §à§ä§Ü§Ý§ð§é§Ö§ß§Ú§ñ §Ò§Ñ§ä§Ñ§â§Ö§Ú (p.s. §ä§à§Ý§î§Ü§à §á§â§Ú §Ó§í§Ü§Ý§ð§é§Ö§ß§ß§à§Þ §Ò§Ý§à§Ü§Ö §á§Ú§ä§Ñ§ß§Ú§ñ.)


#define VBAT_MIN 0x41 //7040 0mV
#define VBAT_MAX 0x70 //10048 mV         (§¬§à§â§à§é§Ö §ï§ä§Ú 3 §á§Ñ§â§Ñ§Þ§Ö§ä§â§Ñ §Ó§Ú§Õ§Ú§Þ§à §ß§Ú§Ô§Õ§Ö §ß§Ö §å§é§Ñ§ã§ä§Ó§å§ð§ä, §Õ§å§Þ§Ñ§ð §Þ§à§Ø§ß§à §ß§Ö §ä§â§à§Ô§Ñ§ä§î §à§ä §Ô§â§Ö§ç§Ñ §á§à§Õ§Ñ§Ý§î§ê§Ö, §à§ß§à §Ó§â§à§Õ§Ö §ß§Ö §Þ§Ö§ê§Ñ§Ö§ä)
#define VBAT_DELTA 0x02 //+-128 mV

#define HV_CONFIG_TIMEOUT 5 //delay config BQ
#define HV_ERROR_TIMEOUT 600000 //load switch-off delay

/*--------------------------------------------------------------------------*/

#define AGM_BAT
//#define LI_ION_BAT
//#define OLD_SETTINGS

/*--------------------------------------------------------------------------*/

#ifdef AGM_BAT
#define INIT_CHARGE_OPTION_0            0x021E //EN_LDO, EN_IDPM, PWM_FREQ 800kHz, iadp40x, ibat16x
#define INIT_CHARGE_CURRENT_REGISTER    0x00C0 //192 mA
#define INIT_CHARGE_VOLTAGE_REGISTER    0x3520 //14096 mV
#define INIT_OTG_VOLTAGE_REGISTER       0x0000 //NOT USE
#define INIT_OTG_CURRENT_REGISTER       0x0000 //NOT USE
#define INIT_INPUT_VOLTAGE_REGISTER     0x3200 //16000 mV threshold VINDPM
#define INIT_MINIMUM_SYSTEM_VOLTAGE     0x2841 //10240 mV
#define INIT_INPUT_CURRENT_REGISTER     0x0F00 //650 mA
#define INIT_CHARGE_OPTION_1            0x8000 //rac rsr 10mOm, comparator disable, en_prochot_lwpwr_vsys
#define INIT_CHARGE_OPTION_2            0x023A //en_batoc 125%, en_extlim
#define INIT_CHARGE_OPTION_3            0x0800 //en_ICO
#define INIT_PROCHOT_OPTION_0           0x4A54 //ilim2_vth 150%, icrit_deg 100us, vsys_vth 6v, prochot_width 10ms, prochot_clr idle
#define INIT_PROCHOT_OPTION_1           0x0D20 //idchg_vth 1536 mA, en_prochot_icrit
#define INIT_ADC_OPTION                 0xA07F //adc_vbat, adc_vsys, adc_vbus, every 1sec

#define INIT_CHARGE_STATUS_REGISTER     0x0000
#define INIT_PROCHOT_STATUS_REGISTER    0x0000
#define INIT_INPUT_CURRENT_LIMIT_IN_USE 0x0000

#define INIT_ADCVBUS_PSYS               0x0000
#define INIT_ADCIBAT                    0x0000
#define INIT_ADCIINCMPIN                0x0000
#define INIT_ADCVSYSVBAT                0x0000
#endif  /* AGM_BAT  */

#ifdef LI_ION_BAT
#define INIT_CHARGE_OPTION_0            0x1E02
#define INIT_CHARGE_CURRENT_REGISTER    0x4002
#define INIT_CHARGE_VOLTAGE_REGISTER    0x1037
#define INIT_OTG_VOLTAGE_REGISTER       0x0000
#define INIT_OTG_CURRENT_REGISTER       0x0000
#define INIT_INPUT_VOLTAGE_REGISTER     0xC031
#define INIT_MINIMUM_SYSTEM_VOLTAGE     0x0029
#define INIT_INPUT_CURRENT_REGISTER     0x000C
#define INIT_CHARGE_OPTION_1            0x0080
#define INIT_CHARGE_OPTION_2            0x7A43
#define INIT_CHARGE_OPTION_3            0x0080
#define INIT_PROCHOT_OPTION_0           0x0000
#define INIT_PROCHOT_OPTION_1           0x0000
#define INIT_ADC_OPTION                 0x0000
#define INIT_CHARGE_STATUS_REGISTER     0x0000
#define INIT_PROCHOT_STATUS_REGISTER    0x0000
#define INIT_INPUT_CURRENT_LIMIT_IN_USE 0x0000

#define INIT_ADCVBUS_PSYS               0x0000
#define INIT_ADCIBAT                    0x0000
#define INIT_ADCIINCMPIN                0x0000
#define INIT_ADCVSYSVBAT                0x0000
#endif  /* LI_ION_BAT  */


#ifdef OLD_SETTINGS
#define INIT_CHARGE_OPTION_0            0x020A
#define INIT_CHARGE_CURRENT_REGISTER    0x0100
#define INIT_CHARGE_VOLTAGE_REGISTER    0x3070
#define INIT_OTG_VOLTAGE_REGISTER       0x0000
#define INIT_OTG_CURRENT_REGISTER       0x0000
#define INIT_INPUT_VOLTAGE_REGISTER     0x2AC0
#define INIT_MINIMUM_SYSTEM_VOLTAGE     0x3700
#define INIT_INPUT_CURRENT_REGISTER     0x0D00
#define INIT_CHARGE_OPTION_1            0x8000
#define INIT_CHARGE_OPTION_2            0x02FA
#define INIT_CHARGE_OPTION_3            0x2000
#define INIT_PROCHOT_OPTION_0           0x4A54
#define INIT_PROCHOT_OPTION_1           0x0D23
#define INIT_ADC_OPTION                 0xE0FF

#define INIT_CHARGE_STATUS_REGISTER     0x0000
#define INIT_PROCHOT_STATUS_REGISTER    0x0000
#define INIT_INPUT_CURRENT_LIMIT_IN_USE 0x0000

#define INIT_ADCVBUS_PSYS               0x0000
#define INIT_ADCIBAT                    0x0000
#define INIT_ADCIINCMPIN                0x0000
#define INIT_ADCVSYSVBAT                0x0000
#endif  /* OLD_SETTINGS  */

/*--------------------------------------------------------------------------*/

#define CHARGE_OPTION_0_ADDR            0x00  //charge options 0
#define CHARGE_CURRENT_REGISTER_ADDR    0x02  //MAX batt charge current
#define CHARGE_VOLTAGE_REGISTER_ADDR    0x04  //MAX batt voltage
#define OTG_VOLTAGE_REGISTER_ADDR       0x06  //MAX voltage OTG mode
#define OTG_CURRENT_REGISTER_ADDR       0x08  //MAX current OTG mode
#define INPUT_VOLTAGE_REGISTER_ADDR     0x0A  //VINDPM voltage limit
#define MINIMUM_SYSTEM_VOLTAGE_ADDR     0x0C  //min system voltage
#define INPUT_CURRENT_REGISTER_ADDR     0x0E  //input current limit
#define CHARGE_OPTION_1_ADDR            0x30  //charge options 1
#define CHARGE_OPTION_2_ADDR            0x32  //charge options 2
#define CHARGE_OPTION_3_ADDR            0x34  //charge options 3
#define PROCHOT_OPTION_0_ADDR           0x36  //prochot 0
#define PROCHOT_OPTION_1_ADDR           0x38  //prochot 1
#define ADC_OPTION_ADDR                 0x3A  //ADC

#define CHARGE_STATUS_REGISTER_ADDR     0x20
#define PROCHOT_STATUS_REGISTER_ADDR    0x22
#define INPUT_CURRENT_LIMIT_IN_USE_ADDR 0x24

//§á§à§Ü§Ñ§Ù§Ñ§ß§Ú§ñ §ã §Ó§ß§å§ä§â§Ö§ß§ß§Ö§Ô§à §¡§¸§±
#define ADCVBUS_PSYS_ADDR               0x26  //§á§à §Ú§ã§ç§à§Õ§ñ§ë§Ö§Þ§å §ß§Ñ§á§â§ñ§Ø§Ö§ß§Ú§ð §ã BQ
#define ADCIBAT_ADDR                    0x28  //§ä§à§Ü §Ù§Ñ§â§ñ§Õ§Ñ/§â§Ñ§Ù§â§ñ§Õ§Ñ §Ñ§Ü§Ò
#define ADCIINCMPIN_ADDR                0x2A  //§Ó§ç§à§Õ§ñ§ë§Ú§Û §ä§à§Ü §à§Ò§ë§Ú§Û
#define ADCVSYSVBAT_ADDR                0x2C  //§á§à§Ü§Ñ§Ù§Ñ§ß§Ú§ñ §á§à §Ó§ç§à§Õ§ñ§ë§Ö§Þ§å §Ó§à§Ý§î§ä§Ñ§Ø§å §ã §Ó§ß§å§ä§â. §¡§¸§± (§Ö§ã§Ý§Ú <= 0x40 §Ó §ä§Ö§é§Ö§ß§Ú§Ö §Þ§Ú§ß§å§ä§í, 10 §ß§à§Ô§å PC0 §Ó§ß§Ú§Ù)

/*--------------------------------------------------------------------------*/

#endif /* __CONFIG_H */
