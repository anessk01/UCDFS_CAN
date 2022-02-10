
// Have current measurements through analogue pins.
// Shutdown sig on UV - OV case.
// Change SPI Clock
// 3,8 and 13 also measure voltages.


/************************* Includes ***************************/
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6812.h"

/************************* Defines *****************************/
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
#define PWM 1
#define SCTL 2



/*
BMS_IC[current_ic].cells.c_codes[i] -> holds cell voltages for each ic and cell in 100uV.

*/

/**************** Local Function Declaration (Mostly functions to print to the screen) *******************/

void measurement_loop(uint8_t datalog_en); // Function that does most of our work, all we need to know is that if we want our measured values
										  // to be printed with corresponding cell/gpio number we need to have datalog_en input paramater to be 0.


void print_menu(void);
void print_wrconfig(void);
void print_wrconfigb(void);
void print_rxconfig(void);
void print_rxconfigb(void);
void print_cells(uint8_t datalog_en);
void print_aux(uint8_t datalog_en);
void print_stat(void);
void print_aux1(void);
void print_sumofcells(void);
void check_mux_fail(void);
void print_selftest_errors(uint8_t adc_reg ,int8_t error);
void print_overlap_results(int8_t error);
void print_digital_redundancy_errors(uint8_t adc_reg ,int8_t error);
void print_open_wires(void);
void print_pec_error_count(void);
int8_t select_s_pin(void);
void print_wrpwm(void);
void print_rxpwm(void);
void print_wrsctrl(void);
void print_rxsctrl(void);
void print_wrpsb(uint8_t type);
void print_rxpsb(uint8_t type);
void print_wrcomm(void);
void print_rxcomm(void);
void check_mute_bit(void);
void print_conv_time(uint32_t conv_time);
void check_error(int error);
void serial_print_text(char data[]);
void serial_print_hex(uint8_t data);
char read_hex(void); 
char get_char(void);

/*******************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/

const uint8_t TOTAL_IC = 10; //!< Number of ICs in the daisy chain

//ADC Command Configurations. See LTC681x.h for options.
const uint8_t ADC_OPT = 0; //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = 1; //!< ADC Mode -> 27Khz Fastest mode.
const uint8_t ADC_DCP = 0; //!< Discharge not Permitted 
const uint8_t CELL_CH_TO_CONVERT1 = 1; //!< Channel Selection for ADC conversion -> 1,6,11
const uint8_t CELL_CH_TO_CONVERT2 = 2; //!< Channel Selection for ADC conversion -> 2,7,12
const uint8_t AUX_CH_TO_CONVERT = 0;  //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;  //!< Channel Selection for ADC conversion                      ??????? (Send high to shutdown sig)
const uint8_t SEL_ALL_REG = 0; //!< Register Selection 
const uint8_t SEL_REG_A = REG_1; //!< Register Selection                                                         ?????
const uint8_t SEL_REG_B = REG_2; //!< Register Selection 														 ?????

const uint16_t MEASUREMENT_LOOP_TIME = 500; //!< Loop Time in milliseconds(ms)

//Under Voltage and Over Voltage Thresholds -> Ask Yev.
const uint16_t OV_THRESHOLD = 41000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = 30000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

//Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED;  //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = DISABLED; //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED; //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = ENABLED; //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = DISABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t PRINT_PEC = DISABLED; //!< This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop
/************************************
  END SETUP
*************************************/


/******************************************************
 Global Battery Variables received from 681x commands
 These variables store the results from the LTC6812											**Look up data sheet and make an example case**
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/
cell_asic BMS_IC[TOTAL_IC]; //!< Global Battery Variable






/*************************************************************************
 Set configuration register. Refer to the data sheet										** See where these have been used and set as per BMS need**
**************************************************************************/
bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = true; //!< ADC Mode option bit
bool GPIOBITS_A[5] = {false,false,true,true,true}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
bool GPIOBITS_B[4] = {false,false,false,false}; //!< GPIO Pin Control // Gpio 6,7,8,9
uint16_t UV=UV_THRESHOLD; //!< Under voltage Comparison Voltage
uint16_t OV=OV_THRESHOLD; //!< Over voltage Comparison Voltage
bool DCCBITS_A[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool DCCBITS_B[7]= {false,false,false,false}; //!< Discharge cell switch //Dcc 0,13,14,15
bool DCTOBITS[4] = {true,false,true,false}; //!< Discharge time value //Dcto 0,1,2,3  // Programed for 4 min 
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */
bool FDRF = false; //!< Force Digital Redundancy Failure Bit
bool DTMEN = true; //!< Enable Discharge Timer Monitor
bool PSBits[2]= {false,false}; //!< Digital Redundancy Path Selection//ps-0,1






/*!**********************************************************************
 \brief  Initializes hardware and variables
  @return void
 ***********************************************************************/
 
 
 void setup()
{
  Serial.begin(115200); // Baudrate.
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock --> Change to 500Khz.
  
  // Check the use case of each of these functions.
  LTC6812_init_cfg(TOTAL_IC, BMS_IC);
  LTC6812_init_cfgb(TOTAL_IC,BMS_IC);
  
  //Setting up each IC in the Daisy chain with correct configuration.
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
  {
    LTC6812_set_cfgr(current_ic,BMS_IC,REFON,ADCOPT,GPIOBITS_A,DCCBITS_A, DCTOBITS, UV, OV);
    LTC6812_set_cfgrb(current_ic,BMS_IC,FDRF,DTMEN,PSBits,GPIOBITS_B,DCCBITS_B);   
  }   
  LTC6812_reset_crc_count(TOTAL_IC,BMS_IC);
  LTC6812_init_reg_limits(TOTAL_IC,BMS_IC);
  print_menu();
}




/*!*****************************************
  \brief Executes the user command
   @return void
*******************************************/
void run_command(uint32_t cmd)
{
  uint8_t streg=0;
  int8_t error = 0;
  uint32_t conv_time = 0;
  int8_t s_pin_read=0;

  switch (cmd)
  {
    case 1: // Write and read Configuration Register
      wakeup_sleep(TOTAL_IC);
      LTC6812_wrcfg(TOTAL_IC,BMS_IC); // Write into Configuration Register
      LTC6812_wrcfgb(TOTAL_IC,BMS_IC); // Write into Configuration Register B
      print_wrconfig();
      print_wrconfigb();

      wakeup_idle(TOTAL_IC);
      error = LTC6812_rdcfg(TOTAL_IC,BMS_IC); // Read Configuration Register
      check_error(error);
      error = LTC6812_rdcfgb(TOTAL_IC,BMS_IC); // Read Configuration Register B
      check_error(error);
      print_rxconfig();
      print_rxconfigb();
      break;

    case 2: // Read Configuration Register
      wakeup_sleep(TOTAL_IC);
      error = LTC6812_rdcfg(TOTAL_IC,BMS_IC);
      check_error(error);
      error = LTC6812_rdcfgb(TOTAL_IC,BMS_IC);
      check_error(error);
      print_rxconfig();
      print_rxconfigb();
      break;

    case 3: // Start Cell ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6812_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT); // Measure All cell voltages
      conv_time = LTC6812_pollAdc(); /* This function will block operation until the ADC has finished it's conversion */
      print_conv_time(conv_time);
      break;

    case 4: // Read Cell Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6812_rdcv(SEL_ALL_REG, TOTAL_IC,BMS_IC); // Set to read back all cell voltage registers
      check_error(error);
      print_cells(DATALOG_DISABLED);
      break;

    case 5: // Start GPIO ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6812_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
      conv_time= LTC6812_pollAdc();
      print_conv_time(conv_time);
      break;

    case 6: // Read AUX Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6812_rdaux(SEL_ALL_REG,TOTAL_IC,BMS_IC); // Set to read back all aux registers
      check_error(error);
      print_aux(DATALOG_DISABLED);
      break;

    case 7: // Start Status ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6812_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
      conv_time=LTC6812_pollAdc();
      print_conv_time(conv_time);
      break;

    case 8: // Read Status registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6812_rdstat(SEL_ALL_REG,TOTAL_IC,BMS_IC); // Set to read back all stat registers
      check_error(error);
      print_stat();
      break;

    case 9 : // Start Combined Cell Voltage and GPIO1, GPIO2 Conversion and Poll Status
      wakeup_sleep(TOTAL_IC);
      LTC6812_adcvax(ADC_CONVERSION_MODE,ADC_DCP);
      conv_time = LTC6812_pollAdc();
      print_conv_time(conv_time);
      wakeup_idle(TOTAL_IC);
      error = LTC6812_rdcv(SEL_ALL_REG, TOTAL_IC,BMS_IC); // Set to read back all cell voltage registers
      check_error(error);
      print_cells(DATALOG_DISABLED);
      wakeup_idle(TOTAL_IC);
      error = LTC6812_rdaux(SEL_REG_A, TOTAL_IC,BMS_IC); // Set to read back aux register A
      check_error(error);
      print_aux1(DATALOG_DISABLED);
      break;

    case 10 : // Start Combined Cell Voltage and Sum of cells
      wakeup_sleep(TOTAL_IC);
      LTC6812_adcvsc(ADC_CONVERSION_MODE,ADC_DCP);
      conv_time = LTC6812_pollAdc();
      print_conv_time(conv_time);
      wakeup_idle(TOTAL_IC);
      error = LTC6812_rdcv(SEL_ALL_REG, TOTAL_IC,BMS_IC); // Set to read back all cell voltage registers
      check_error(error);
      print_cells(DATALOG_DISABLED);
      wakeup_idle(TOTAL_IC);
      error = LTC6812_rdstat(SEL_REG_A,TOTAL_IC,BMS_IC); // Set to read back stat register A
      check_error(error);
      print_sumofcells();
      break;

    case 11: // Loop Measurements of configuration register or cell voltages or auxiliary register or status register without data-log output
      wakeup_sleep(TOTAL_IC);
      LTC6812_wrcfg(TOTAL_IC,BMS_IC);
      LTC6812_wrcfgb(TOTAL_IC,BMS_IC);
      measurement_loop(DATALOG_DISABLED);
      print_menu();
      break;

    case 12: // Data-log print option Loop Measurements of configuration register or cell voltages or auxiliary register or status register
      wakeup_sleep(TOTAL_IC);
      LTC6812_wrcfg(TOTAL_IC,BMS_IC);
      LTC6812_wrcfgb(TOTAL_IC,BMS_IC);
      measurement_loop(DATALOG_ENABLED);
      print_menu();
      break;

    case 13: // Clear all ADC measurement registers
      wakeup_sleep(TOTAL_IC);
      LTC6812_clrcell();
      LTC6812_clraux();
      LTC6812_clrstat();
      wakeup_idle(TOTAL_IC);
      LTC6812_rdcv(SEL_ALL_REG, TOTAL_IC,BMS_IC); // read back all cell voltage registers
      print_cells(DATALOG_DISABLED);
      LTC6812_rdaux(SEL_ALL_REG,TOTAL_IC,BMS_IC); // read back all auxiliary registers
      print_aux(DATALOG_DISABLED);
      LTC6812_rdstat(SEL_ALL_REG,TOTAL_IC,BMS_IC); // read back all status registers
      print_stat();
      break;
	  
	default:
      char str_error[]="Incorrect Option \n";
      serial_print_text(str_error);
      break;
  }
}







/*!**********************************************************************************************************************************************
 \brief For writing/reading configuration data or measuring cell voltages or reading aux register or reading status register in a continuous loop  
 @return void
*************************************************************************************************************************************************/
void measurement_loop(uint8_t datalog_en)
{
  int8_t error = 0;
  char input = 0;
  
  Serial.println(F("Transmit 'm' to quit"));
  
  while (input != 'm')
  {
     if (Serial.available() > 0)
      {
        input = read_char();
      } 
  
    if (WRITE_CONFIG == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6812_wrcfg(TOTAL_IC,BMS_IC);
      LTC6812_wrcfgb(TOTAL_IC,BMS_IC);
      print_wrconfig();
      print_wrconfigb();
    }
  
    if (READ_CONFIG == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      error = LTC6812_rdcfg(TOTAL_IC,BMS_IC);
      check_error(error);
      error = LTC6812_rdcfgb(TOTAL_IC,BMS_IC);
      check_error(error);
      print_rxconfig();
      print_rxconfigb();
    }
  
    if (MEASURE_CELL == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6812_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT1);
      LTC6812_pollAdc();       // Poll Adc waits until all cells have been measured.
	  wakeup_idle(TOTAL_IC);
	  LTC6812_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT2);
      LTC6812_pollAdc();       // Poll Adc waits until all cells have been measured.
      wakeup_idle(TOTAL_IC);
      error = LTC6812_rdcv(0, TOTAL_IC,BMS_IC);	//reading the registers in which the measured cell voltages are present  
      check_error(error);
      print_cells(datalog_en);
    }
  
    if (MEASURE_AUX == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6812_adax(ADC_CONVERSION_MODE , AUX_CH_ALL);
      LTC6812_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6812_rdaux(0,TOTAL_IC,BMS_IC); // Set to read back all aux registers
      check_error(error);
      print_aux(datalog_en);
    }
  
    if (MEASURE_STAT == ENABLED)
    {
      wakeup_idle(TOTAL_IC);
      LTC6812_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);
      LTC6812_pollAdc();
      wakeup_idle(TOTAL_IC);
      error = LTC6812_rdstat(0,TOTAL_IC,BMS_IC); // Set to read back all aux registers
      check_error(error);
      print_stat();
    }
  
    if(PRINT_PEC == ENABLED)
    {
      print_pec_error_count();
    }

    delay(MEASUREMENT_LOOP_TIME);
  }
}
}






/*!*********************************
  \brief Prints the main menu
   @return void
***********************************/

void print_menu(void)
{
  Serial.println(F("List of LTC6812 Command:"));
  Serial.println(F("Write and Read Configuration: 1                            |Loop measurements with data-log output : 12                            |Set Discharge: 23   "));  
  Serial.println(F("Read Configuration: 2                                      |Clear Registers: 13                                                    |Clear Discharge: 24   "));
  Serial.println(F("Start Cell Voltage Conversion: 3                           |Run Mux Self Test: 14                                                  |Write and Read of PWM : 25"));
  Serial.println(F("Read Cell Voltages: 4                                      |Run ADC Self Test: 15                                                  |Write and  Read of S control : 26"));
  Serial.println(F("Start Aux Voltage Conversion: 5                            |ADC overlap Test : 16                                                  |Clear S control register : 27"));
  Serial.println(F("Read Aux Voltages: 6                                       |Run Digital Redundancy Test: 17                                        |SPI Communication  : 28"));
  Serial.println(F("Start Stat Voltage Conversion: 7                           |Open Wire Test for single cell detection: 18                           |I2C Communication Write to Slave :29"));
  Serial.println(F("Read Stat Voltages: 8                                      |Open Wire Test for multiple cell or two consecutive cells detection:19 |I2C Communication Read from Slave :30"));
  Serial.println(F("Start Combined Cell Voltage and GPIO1, GPIO2 Conversion: 9 |Open wire for Auxiliary Measurement: 20                                |Enable MUTE : 31"));
  Serial.println(F("Start  Cell Voltage and Sum of cells : 10                  |Print PEC Counter: 21                                                  |Disable MUTE : 32")); 
  Serial.println(F("Loop Measurements: 11                                      |Reset PEC Counter: 22                                                  |Set or reset the gpio pins: 33 \n "));
  Serial.println(F("Print 'm' for menu"));
  Serial.println(F("Please enter command: \n "));
}

/*!******************************************************************************
 \brief Prints the Configuration Register A data that is going to be written to 
 the LTC6812 to the serial port.
  @return void
 ********************************************************************************/
void print_wrconfig(void)
{
    int cfg_pec;
    Serial.println(F("Written Configuration A Register: "));
    for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
    {
      Serial.print(F("CFGA IC "));
      Serial.print(current_ic+1,DEC);
      for(int i = 0;i<6;i++)
      {
        Serial.print(F(", 0x"));
        serial_print_hex(BMS_IC[current_ic].config.tx_data[i]);
      }
      Serial.print(F(", Calculated PEC: 0x"));
      cfg_pec = pec15_calc(6,&BMS_IC[current_ic].config.tx_data[0]);
      serial_print_hex((uint8_t)(cfg_pec>>8));
      Serial.print(F(", 0x"));
      serial_print_hex((uint8_t)(cfg_pec));
      Serial.println("\n");
    }
}

/*!******************************************************************************
 \brief Prints the Configuration Register B data that is going to be written to 
 the LTC6812 to the serial port.
  @return void
 ********************************************************************************/
void print_wrconfigb(void)
{
    int cfg_pec;
    Serial.println(F("Written Configuration B Register: "));
    for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
    { 
      Serial.print(F("CFGB IC "));
      Serial.print(current_ic+1,DEC);
      for(int i = 0;i<6;i++)
      {
        Serial.print(F(", 0x"));
        serial_print_hex(BMS_IC[current_ic].configb.tx_data[i]);
      }
      Serial.print(F(", Calculated PEC: 0x"));
      cfg_pec = pec15_calc(6,&BMS_IC[current_ic].configb.tx_data[0]);
      serial_print_hex((uint8_t)(cfg_pec>>8));
      Serial.print(F(", 0x"));
      serial_print_hex((uint8_t)(cfg_pec));
      Serial.println("\n");
    }
}

/*!*****************************************************************
 \brief Prints the Configuration Register A data that was read back 
 from the LTC6812 to the serial port.
  @return void
 *******************************************************************/
void print_rxconfig(void)
{
  Serial.println(F("Received Configuration A Register: "));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGA IC "));
    Serial.print(current_ic+1,DEC);
    for(int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].config.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].config.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].config.rx_data[7]);
    Serial.println("\n");
  }
}

/*!*****************************************************************
 \brief Prints the Configuration Register B that was read back from 
 the LTC6812 to the serial port.
  @return void
 *******************************************************************/
void print_rxconfigb(void)
{
  Serial.println(F("Received Configuration B Register: "));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F("CFGB IC "));
    Serial.print(current_ic+1,DEC);
    for(int i = 0; i < 6; i++)
    {
      Serial.print(F(", 0x"));
      serial_print_hex(BMS_IC[current_ic].configb.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(BMS_IC[current_ic].configb.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].configb.rx_data[7]);
    Serial.println("\n");
  }
}

/*!************************************************************
  \brief Prints cell voltage to the serial port
   @return void
 *************************************************************/
void print_cells(uint8_t datalog_en)
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1,DEC);
      Serial.print(", ");
      for (int i=0; i<BMS_IC[0].ic_reg.cell_channels; i++)
      {
        Serial.print(" C");
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001,4/*4 decimal places*/);
        Serial.print(",");
      }
      Serial.println();
    }
    else
    {
      Serial.print(" Cells, ");
      for (int i=0; i<BMS_IC[0].ic_reg.cell_channels; i++)
      {
        Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001,4);
        Serial.print(",");
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints GPIO voltage and Vref2 voltage code onto the serial port
   @return void
 *****************************************************************************/
void print_aux(uint8_t datalog_en)
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1,DEC);
      
        
      for (int i = 0; i < 5; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
      
      for (int i=6; i < 10; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i,DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }

      Serial.print(F(" Vref2 :"));
      Serial.print(BMS_IC[current_ic].aux.a_codes[5]*0.0001,4);
      Serial.println();

      Serial.print(" OV/UV Flags : 0x");
      Serial.print((uint8_t)BMS_IC[current_ic].aux.a_codes[11],HEX);
      Serial.println();
    }
    else
    {
      Serial.print(" AUX : ");

      for (int i=0; i < 12; i++)
      {
        Serial.print(BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
   @return void
 *****************************************************************************/
void print_stat(void)
{
  double itmp;

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(" SOC:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0]*0.0001*30,4);
    Serial.print(F(","));
    Serial.print(F(" Itemp:"));
    itmp = (double)((BMS_IC[current_ic].stat.stat_codes[1] * (0.0001 / 0.0076)) - 276);   //Internal Die Temperature(°C) = ITMP • (100 µV / 7.6mV)°C - 276°C
    Serial.print(itmp,4);
    Serial.print(F(","));
    Serial.print(F(" VregA:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[2]*0.0001,4);
    Serial.print(F(","));
    Serial.print(F(" VregD:"));
    Serial.println(BMS_IC[current_ic].stat.stat_codes[3]*0.0001,4);
    Serial.print(F(" OV/UV Flags: 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.flags[2]);
    Serial.print(F(",\t Mux fail flag:"));
    Serial.print(F(" 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.mux_fail[0]);
    Serial.print(F(",\t THSD:"));
    Serial.print(F(" 0x"));
    serial_print_hex(BMS_IC[current_ic].stat.thsd[0]);
    Serial.println("\n");
  }  
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes (GPIO 1 & 2)
   @return void
 *****************************************************************************/
void print_aux1(uint8_t datalog_en)
{

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1,DEC);
      for (int i=0; i < 2; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
    }
    else
    {
      Serial.print("AUX, ");

      for (int i=0; i < 6; i++)
      {
        Serial.print(BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
        Serial.print(",");
      }
    }
  }
  Serial.println("\n");
}

/*!****************************************************************************
  \brief Prints Status voltage codes for SOC onto the serial port
   @return void
 *****************************************************************************/
void print_sumofcells(void)
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": SOC:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0]*0.0001*30,4);
    Serial.print(F(","));
  }
  Serial.println("\n");
}
