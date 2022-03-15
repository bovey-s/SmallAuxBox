#include "SPI.h"
#include "Arduino.h"
#include <stdint.h>

#define ADS1256

#ifdef ADS1256
#include "ads1256.h"
#endif

//#define ATMEGA
#define ESP32HUZZAH

//SPi settings
#define SPI_MODE SPI_MODE1

//define pins here for SPI bus(es)
#define VMOSI   18
#define VMISO   19
#define VSCK    5
#define VCS     23

#define HMOSI   13
#define HMISO   12
#define HSCK    14
#define HCS     15

/*
* A library written to interface with the TI ADS12xx ADC modules.
* Written by Liam Ward 2021
* Contact email: liam.ward144@gmail.com
*/

class ADS12xx {
private:
	//Attributes
	SPIClass SPIObj; //custom SPI object
	bool vspi; //if !vspi => using hspi
	uint8_t cycleOrder[4] = { AIN2_AIN3, AIN4_AIN5, AIN6_AIN7, AIN0_AIN1 };

	//Pins for ADC interface
	int _CS;
	int _DRDY;
	int _RESET; 

	//Methods
	int32_t read_int32();

	bool checkChannelChange(uint8_t AIN_PG, uint8_t AIN_NG);

	SPISettings settings = SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE);

public:

	ADS12xx(); //constructor
	ADS12xx(SPIClass& spi, bool _vspi = false); //constructor w/ custom SPI object & default vspi params

	//chip select fxns
	void CS_ON();
	void CS_OFF();

	void begin(uint8_t _CS, uint8_t _DRDY, uint8_t _RESET, uint8_t dataRate);
	void begin(uint8_t _START, uint8_t _CS, uint8_t _DRDY, uint8_t _RESET, uint8_t dataRate); //for ADS modules with START pin
	void init_setup(uint8_t dataRate);

	int32_t getConversion(bool wait);
	double getConversionSingle(void);
	double GetTotalResidualVoltageDrop(void);
	int32_t sendReadCommand(void);
	void waitForDRDY(void);
	void waitForDRDYSoftware(void);

	void writeRegister(uint8_t reg, uint8_t data);
	uint8_t readRegister(uint8_t reg);

	void sendCommand(uint8_t cmd);
	void setChannel(uint8_t AIN_P, uint8_t AIN_N); //set with any permutation of channels (8! / (8-2)!) == 56 possible permutations -> let the computer choose!
	void setDiffrentialPairChannel(uint8_t channelConfig);

	void cycleInputMultiplexers(double* storage, uint8_t num_channel_pairs);
	void cycleUPPInputMultiplexers(volatile double* storage, uint8_t* channelOrder, int index_offset, uint8_t num_channels);
	void UPPMeasCycleElement(volatile double* storage, uint8_t* channel, int index_offset, int chindex);

	int32_t sendCycleSequence(uint8_t channelSelect);

	void startSPItransaction(void);
	void stopSPItransaction(void);

	void Reset();
	void setPGA(uint8_t gainConfig);

};

//Bit masking
#define MASK(p) ((unsigned char)(1<<p))
#define SET_BIT(x, pos) (x |= MASK(pos)) //pass in 'x' as bytes, set bit at (zero-indexed) position 'pos'
#define CLEAR_BIT(x, pos) (x &= ~MASK(pos)) //pass in 'x' as bytes, clear bit at (zero-indexed) position 'pos'
#define TOGGLE_BIT(x, pos) (x ^= MASK(pos)) //pass in 'x' as bytes, toggle bit at (zero-indexed) position 'pos'

//Only store one bit versus storing a (probably)32-bit int in a bit-field
//put inside typedef so we can declare volatile
typedef struct State {
	unsigned int flag : 1;
}DRDYSTATE;


