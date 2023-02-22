/*
 *
 */

typedef enum { /* register map */
     kINA219RegisterConfig        = 0x00,
     kINA219RegisterShuntVoltage  = 0x01,
     kINA219RegisterBusVoltage    = 0x02,
     kINA219RegisterPower         = 0x03,
     kINA219RegisterCurrent       = 0x04,
     kINA219RegisterCalibration   = 0x05,
 } INA219Register;

 #define INA219_CAL         0xA000
//32V 2A config
 #define INA219_CURRENT_LSB 10
#define INA219_POWER_LSB   (20 * INA219_CURRENT_LSB)

 typedef enum {
     ina219_pg_range_40mv  = 0b00,
     ina219_pg_range_80mv  = 0b01,
     ina219_pg_range_160mv = 0b10,
     ina219_pg_range_320mv = 0b11,
 } INA219Range;

 typedef enum {
     ina219_adc_mode_9bit  = 0b0000,
     ina219_adc_mode_10bit = 0b0001,
     ina219_adc_mode_11bit = 0b0010,
     ina219_adc_mode_12bit = 0b0011,

     ina219_adc_samples_1   = 0b1000,
     ina219_adc_samples_2   = 0b1001,
     ina219_adc_samples_4   = 0b1010,
     ina219_adc_samples_8   = 0b1011,
     ina219_adc_samples_16  = 0b1100,
     ina219_adc_samples_32  = 0b1101,
     ina219_adc_samples_64  = 0b1110,
     ina219_adc_samples_128 = 0b1111,
 } INA219ADCconfig;

 typedef enum {
     ina219_mode_power_down     = 0b000,
     ina219_mode_shunt_trig     = 0b001,
     ina219_mode_bus_trig       = 0b010,
     ina219_mode_shunt_bus_trig = 0b011,
     ina219_mode_adc_off        = 0b100,
     ina219_mode_shunt_cont     = 0b101,
     ina219_mode_bus_cont       = 0b110,
     ina219_mode_shunt_bus_cont = 0b111,
 } INA219Mode;

 typedef enum {
     ina219_bus_voltage_range_16v = 0,
     ina219_bus_voltage_range_32v = 1,
 } INA219BusVRange;

 typedef struct __attribute__((packed)) {
     union {
         struct {
             INA219Mode mode:      3;
             INA219ADCconfig sadc: 4;
             INA219ADCconfig badc: 4;
             INA219Range pg:       2;
             INA219BusVRange brng: 1; //bit 13 of BRNG as per datasheet
             uint16_t :            1;
             uint16_t rst:         1;
         } __attribute__((packed));
         uint16_t raw_val;
     };
 } ina219_config_t;

typedef struct __attribute__((packed)) {
    union {
        struct {
            uint16_t ovf:  1; /* math overflow flag                           */
            uint16_t cnvr: 1; /* conversion ready flag                        */
            uint16_t:      1; /* UNUSED                                       */
            uint16_t bd:  13; /* bus voltage data                             */
        } __attribute__((packed));
        struct {
            uint8_t lsb, msb;
        } __attribute__((packed));
    };
} ina219_reg_bus_voltage_t;

typedef struct {
    /* because reading the registers _should_ be performed in a certain order:
     * 0. (shunt voltage)
     * 1. bus voltage   - wait for conv ready
     * 2. current
     * 3. power         - clears conv ready
     *
     * we want to return a set of coherent readings together.
     * bus voltage also includes a flag for conv overflow,
     * so return that as an error indicator.
     */
    bool error;
    unsigned int busVoltage, power;
    int current, shuntVoltage;
} ina219_reading_set_t;

 static const ina219_config_t config = {
     .mode = ina219_mode_shunt_bus_cont,
     .sadc = ina219_adc_samples_1,
     .badc = ina219_adc_mode_12bit,
       .pg = ina219_pg_range_40mv,
     .brng = ina219_bus_voltage_range_16v,
      .rst = 0
 };

 void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);

 WarpStatus
 devINA219writeRegister(INA219Register deviceRegister, uint16_t payload);

 WarpStatus
 devINA219writeRegisterPointer(INA219Register deviceRegister);

 WarpStatus
 devINA219read(uint8_t deviceRegister, int numberOfBytes);

int
devINA219getCurrent(void);

unsigned int
devINA219getBusVoltage(void);

int
devINA219getShuntVoltage(void);

unsigned int
devINA219getPower(void);
