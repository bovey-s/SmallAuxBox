#include "ADS12xx.h"
volatile DRDYSTATE DRDY_STATE;

//Non-class fxn prototypes
void DRDYInterrupt();
uint8_t getChannelAddress(uint8_t AIN_P, uint8_t AIN_N);

ADS12xx::ADS12xx() { //default constructor
} 

ADS12xx::ADS12xx(SPIClass& spi, bool _vspi) {
	this->SPIObj = spi;
	this->vspi = _vspi;
}

void ADS12xx::CS_ON() {
    digitalWrite(this->_CS, LOW);
}

void ADS12xx::CS_OFF() {
    digitalWrite(this->_CS, HIGH);
}

void ADS12xx::begin(uint8_t _CS, uint8_t _DRDY, uint8_t _RESET, uint8_t dataRate) {
    //Assign class member pins
    this->_DRDY = _DRDY;
    this->_RESET = _RESET;
    this->_CS = _CS;	
    
    pinMode(_CS, OUTPUT); // set the Chip select pin as output
    pinMode(_DRDY, INPUT); // DRDY as input
    pinMode(_RESET, OUTPUT); //set reset as output

    CS_OFF();
    SET_BIT(DRDY_STATE.flag, 0); //set bit high for initialization

    //Interrupt setup for DRDY detection
    attachInterrupt(this->_DRDY, DRDYInterrupt, FALLING);
	delay(500);

/*#ifdef ESP32HUZZAH
    if (this->vspi) {
        this->SPIObj.begin(VSCK, VMISO, VMOSI, this->_CS);
        //Serial.println("STARTED ADC on VSPI");
    }
    else { //using hspi
        this->SPIObj.begin(HSCK, HMISO, HMOSI, this->_CS);
        //Serial.println("STARTED ADC on HSPI");
    }
#endif*/

#ifdef ATMEGA
    this->SPIObj.begin();
    //Serial.println("Started ADC on ATMEGA default SPI bus");
#endif

    Reset();
    sendCommand(SELFCAL); //calibrate

    //Start conversion process
    sendCommand(SYNC);
    sendCommand(WAKEUP);

    delay(100);
    writeRegister(DRATE, dataRate);  // write data rate register
    writeRegister(MUX, AIN0_AIN1); //set diffrential pair selection to AIN0/1 as default
    setPGA(PGA_1);
    
}

void ADS12xx::init_setup(uint8_t dataRate) {
    writeRegister(DRATE, dataRate);  // write data rate register
    writeRegister(MUX, AIN0_AIN1); //set diffrential pair selection to AIN0/1 as default
}

void ADS12xx::begin(uint8_t _START, uint8_t _CS, uint8_t _DRDY, uint8_t _RESET, uint8_t dataRate) {} //TODO: Implement for ads1247/8 

int32_t ADS12xx::getConversion(bool wait) {
    if (wait) {
        waitForDRDY();
    }
    int32_t d = sendReadCommand();
	return d;
}

double ADS12xx::getConversionSingle() { //assumes ads is in standby mode first time this is called
    
    sendCommand(WAKEUP);
    delayMicroseconds(32);
#ifdef ATMEGA
    waitForDRDYSoftware();
#endif
#ifdef ESP32HUZZAH
    waitForDRDY();
#endif
    int32_t d = sendReadCommand();
    sendCommand(STANDBY);
    return (double)d;
}

int32_t ADS12xx::sendReadCommand() {
    this->CS_ON();
    this->SPIObj.beginTransaction(this->settings);
    this->SPIObj.transfer(RDATA);
    delayMicroseconds(7); //50xTCLKIN rounded up
    int32_t d = read_int32();
    this->SPIObj.endTransaction();
    this->CS_OFF();
    return d;
}

int32_t ADS12xx::read_int32() {
    /*
    uint8_t _highByte, _midByte, _lowByte;
    
    _highByte = this->SPIObj.transfer(WAKEUP);
    _midByte = this->SPIObj.transfer(WAKEUP);
    _lowByte = this->SPIObj.transfer(WAKEUP);
    */

    int32_t value = 0;
    // Combine all 3-bytes to 24-bit data using byte shifting, shift into 32-bit container
    for (int i = 0; i < 3; i++) { //24 bits -> 3 bytes
        value |= this->SPIObj.transfer(WAKEUP); //0x00   
        if (i < 2) { value <<= 8; }
    }
    
	// Combine all 3-bytes to 24-bit data using byte shifting, shift into 32-bit container
    /*
    value |= _highByte;
    Serial.println(value, HEX);
    value <<= 8;
    value |= _midByte;
    Serial.println(value, HEX);
    value <<= 8;
    value |= _lowByte;
    Serial.println(value, HEX);
    */
	return value;
}


void ADS12xx::writeRegister(uint8_t reg, uint8_t data) {
    this->SPIObj.beginTransaction(this->settings);
    this->CS_ON();
    /*
    Serial.print("Transferring: ");
    Serial.print(data, HEX);
    Serial.print(" to -> register ");
    Serial.println(reg, HEX);
    */
    this->SPIObj.transfer(WREG | reg);
    this->SPIObj.transfer(WAKEUP);
    this->SPIObj.transfer(data);
    delayMicroseconds(3); //4xTCLKIN (rounded up)    
    this->CS_OFF();
    this->SPIObj.endTransaction();
}

uint8_t ADS12xx::readRegister(uint8_t reg) {
    uint8_t read;
    this->SPIObj.beginTransaction(this->settings);
    this->CS_ON();
    this->SPIObj.transfer(RREG | reg);
    this->SPIObj.transfer(WAKEUP);
    delayMicroseconds(t6);
    read = this->SPIObj.transfer(WAKEUP);
    delayMicroseconds(t11);
    this->CS_OFF();
    this->SPIObj.endTransaction();
    return read;
}

void ADS12xx::sendCommand(uint8_t cmd) {
    this->CS_ON();
    this->SPIObj.beginTransaction(this->settings);
    this->SPIObj.transfer(cmd);
    if (cmd == SYNC) { delayMicroseconds(5); } //24xTCLKIN + 2
    else { delayMicroseconds(2); } //4xTCLKIN (rounded up) + 1
    this->SPIObj.endTransaction();
    this->CS_OFF();
}

void ADS12xx::setChannel(uint8_t AIN_P, uint8_t AIN_N) {
    uint8_t MUX_CHANNEL = getChannelAddress((uint8_t)AIN_P, (uint8_t)AIN_N);
    //Serial.print("Setting channel -> 0x");
    //Serial.println(MUX_CHANNEL, HEX);
    writeRegister(MUX, MUX_CHANNEL);
    sendCommand(SYNC);
    sendCommand(WAKEUP);
    delay(3);
    //assert(checkChannelChange(AIN_P, AIN_N) == true); 
    if (!checkChannelChange(AIN_P, AIN_N)) {
        //TODO: Remove Serial before interfacing w/ client side software
        //Serial.println("Channel switching error, could not set -> {"+String(AIN_P)+","+String(AIN_N)+"}");
    }
}

void ADS12xx::setDiffrentialPairChannel(uint8_t channelConfig) {
    //Serial.print("Setting MUX to ");
    //Serial.println(channelConfig, HEX);
    writeRegister(MUX, channelConfig);
    delay(1);
}


bool ADS12xx::checkChannelChange(uint8_t AIN_PG, uint8_t AIN_NG) {
    uint8_t MUX_CHANNEL = getChannelAddress(AIN_PG, AIN_NG);
    //TODO: Remove Serial before interfacing w/ client side software
    /*
    Serial.print("Channel check -> 0x");
    Serial.print(readRegister(MUX),HEX);
    Serial.print(" == 0x");
    Serial.print(MUX_CHANNEL,HEX);
    Serial.println("?");
    */
    return (readRegister(MUX) == MUX_CHANNEL);
}

uint8_t getChannelAddress(uint8_t AIN_P, uint8_t AIN_N) {
    uint8_t MUX_CHANNEL;
    uint8_t MUXP;
    uint8_t MUXN;
    switch (AIN_P) {
    case 0:
        MUXP = P_AIN0;
        break;
    case 1:
        MUXP = P_AIN1;
        break;
    case 2:
        MUXP = P_AIN2;
        break;
    case 3:
        MUXP = P_AIN3;
        break;
    case 4:
        MUXP = P_AIN4;
        break;
    case 5:
        MUXP = P_AIN5;
        break;
    case 6:
        MUXP = P_AIN6;
        break;
    case 7:
        MUXP = P_AIN7;
        break;
    default:
        MUXP = P_AINCOM;
    }

    switch (AIN_N) {
    case 0:
        MUXN = N_AIN0;
        break;
    case 1:
        MUXN = N_AIN1;
        break;
    case 2:
        MUXN = N_AIN2;
        break;
    case 3:
        MUXN = N_AIN3;
        break;
    case 4:
        MUXN = N_AIN4;
        break;
    case 5:
        MUXN = N_AIN5;
        break;
    case 6:
        MUXN = N_AIN6;
        break;
    case 7:
        MUXN = N_AIN7;
        break;
    default:
        MUXN = N_AINCOM;
    }

    //Byte structure is -> MUX_CHANNEL := [7:4] -> {MUXP[3:0]} [3:0] -> {MUXN[3:0]}
    MUX_CHANNEL = MUXP; //send in MUXP bits
    MUX_CHANNEL <<= 4; //shift to make MUXP bits [7:4] in MUX_CHANNEL
    MUX_CHANNEL |= MUXN; //finally, set [3:0] with MUXN bits
    return MUX_CHANNEL;
}

void ADS12xx::cycleInputMultiplexers(double* storage, uint8_t num_channel_pairs) { //assumes channel is set to AIN0/AIN1
    //IMPORTANT: Function assumes that the MUX register thats at AIN0_AIN1
    //Cycle: AIN0/1 -> AIN2/3 -> AIN4/5 -> AIN6/7

    waitForDRDY();
    storage[0] = (double)sendCycleSequence(cycleOrder[3]);//GetTotalResidualVoltageDrop(); //equivalent to data[0]

    for (int i = 0; i < num_channel_pairs; i++) {
        waitForDRDY();
        double d = (double)sendCycleSequence(this->cycleOrder[i]);
        storage[i + 1] = d;
        //Time = micros() - Time;
        //Serial.println("Loop Time -> " + String(Time));
        /*
        waitForDRDY();
        switch (i) {
            case 0: {
                setDiffrentialPairChannel(AIN0_AIN1);
                break;
            }
            case 1: {
                setDiffrentialPairChannel(AIN2_AIN3);
                break;
            }
            case 2: {
                setDiffrentialPairChannel(AIN4_AIN5);
                break;
            }
            case 3: {
                setDiffrentialPairChannel(AIN6_AIN7);
                break;
            }
        }
        sendCommand(SYNC);
        sendCommand(WAKEUP);
        int32_t data_in = getConversion(false);
        storage[i + 1] = (double)data_in;// (double)getConversion(false); //read from previous register (0/1, 2/3, 4/5, 6/7);
        */
    }
}

int32_t ADS12xx::sendCycleSequence(uint8_t channelSelect) {
    this->SPIObj.beginTransaction(this->settings);
    this->CS_ON();

    //Select Channel
    this->SPIObj.transfer(WREG | MUX);
    this->SPIObj.transfer(ZERO); //write a single register: 0000 nnnn, where nnnn:=(# of bytes to be written - 1)
    this->SPIObj.transfer(channelSelect); //transfer data byte to select channel for measurement
    delayMicroseconds(2); //4xTCLKIN (rounded up) + 1   

    //send SYNC
    this->SPIObj.transfer(SYNC);
    delayMicroseconds(5); //24xTCLKIN + 1

    //send WAKEUP
    this->SPIObj.transfer(WAKEUP);
    delayMicroseconds(2); //4xTCLKIN (rounded up) + 1

    //send RDATA
    this->SPIObj.transfer(RDATA);
    delayMicroseconds(7); //50xTCLKIN rounded up

    //Retrieve data from DOUT (MISO)
    int32_t d = read_int32();

    this->CS_OFF();
    this->SPIObj.endTransaction();

    //Serial.println(d);
    //delayMicroseconds(100);
    return d;
}

void ADS12xx::cycleUPPInputMultiplexers(volatile double* storage, uint8_t* channelOrder, int index_offset, uint8_t num_channels){
//Custom version of cycleInputMultiplexers that cycles between 2 channels (order determined by channelOrder)
    //Just like parent function, assumes that MUX register starts at AIN0_AIN1
    //Params:
    //storage -> a point to an array of doubles that is populated with measurements
    //channelOrder contains an array of uint8_t macros to cycle through (should be [AIN2_AIN3, AIN0_AIN1] but left variable for flexibility)
    //index_offset -> tells the fxn where to store data in the storage array (b/c we are using one array to store 4 pieces of data total (2 from each adc))
    
    double d;
    for (int i = 0; i < num_channels; i++) {

        //Select Channel
        this->SPIObj.transfer(WREG | MUX);
        this->SPIObj.transfer(ZERO); //write a single register: 0000 nnnn, where nnnn:=(# of bytes to be written - 1)
        this->SPIObj.transfer(channelOrder[i]); //transfer data byte to select channel for measurement
        delayMicroseconds(2); //4xTCLKIN (rounded up) + 1   

        //send SYNC
        this->SPIObj.transfer(SYNC);
        delayMicroseconds(5); //24xTCLKIN + 1

        //send WAKEUP
        this->SPIObj.transfer(WAKEUP);
        delayMicroseconds(2); //4xTCLKIN (rounded up) + 1

        //wait for data ready indication
        waitForDRDY();

        //send RDATA
        this->SPIObj.transfer(RDATA);
        delayMicroseconds(7); //50xTCLKIN rounded up

        //Retrieve data from DOUT (MISO)
        int32_t d = read_int32();

        //store data
        //Serial.println("Setting at -> " + String(i + index_offset));
        storage[i + index_offset] = d;
        delayMicroseconds(200);
    }
}


void ADS12xx::UPPMeasCycleElement(volatile double* storage, uint8_t* channel, int index_offset, int chindex) {

    waitForDRDY();
    //Select Channel
    this->SPIObj.transfer(WREG | MUX);
    this->SPIObj.transfer(ZERO); //write a single register: 0000 nnnn, where nnnn:=(# of bytes to be written - 1)
    this->SPIObj.transfer(channel[chindex]); //transfer data byte to select channel for measurement
    delayMicroseconds(2); //4xTCLKIN (rounded up) + 1   

    //send SYNC
    this->SPIObj.transfer(SYNC);
    delayMicroseconds(5); //24xTCLKIN + 1

    //send WAKEUP
    this->SPIObj.transfer(WAKEUP);
    delayMicroseconds(2); //4xTCLKIN (rounded up) + 1
    //wait for data ready indication


    //send RDATA
    this->SPIObj.transfer(RDATA);
    delayMicroseconds(7); //50xTCLKIN rounded up

    //Retrieve data from DOUT (MISO)
    int32_t d = read_int32();

    //store data
    //Serial.println("Setting at -> " + String(i + index_offset));
    storage[index_offset] = d;
}

//This fxn will get the voltage drop across the resistance network (after the biasing resistor)
//The purpose is to enable us to calculate the current through the network for sensor
//networks with connected sensors (VREF->0, 0->1, 1->2, ..., n->GND)
double ADS12xx::GetTotalResidualVoltageDrop() {
    /*
    //This is the proper way according to datasheet -> having trouble getting it to work though
    this->CS_ON();
    this->SPIObj.beginTransaction(this->settings);

    //Set channel
    this->SPIObj.transfer(WREG | MUX);
    delayMicroseconds(2); //4xTCLKIN (rounded up) + 1

    this->SPIObj.transfer(WAKEUP);
    delayMicroseconds(2); //4xTCLKIN (rounded up) + 1

    this->SPIObj.transfer(AIN0_AIN7);
    delayMicroseconds(2); //4xTCLKIN (rounded up)  

    //send SYNC
    this->SPIObj.transfer(SYNC);
    delayMicroseconds(4); //24xTCLKIN + 1

    //send WAKEUP
    this->SPIObj.transfer(WAKEUP);
    delayMicroseconds(2); //4xTCLKIN (rounded up) + 1

    //Wait for conversion to complete
    waitForDRDY();

    //send RDATA
    this->SPIObj.transfer(RDATA);
    delayMicroseconds(7); //50xTCLKIN rounded up

    //Read data
    int32_t d = read_int32();

    this->SPIObj.endTransaction();
    this->CS_OFF();
    return (double)d;
    */

    sendCommand(STANDBY);
    setDiffrentialPairChannel(AIN0_AIN7);
    double conversion = getConversionSingle();
    setDiffrentialPairChannel(AIN0_AIN1); //set back to AIN0/AIN1
    return conversion;

}

void ADS12xx::Reset() {
    //The CMD version (not the GPIO version) seems to cause some problems? I think I just don;t have the correct config to send a command before reseting all values
    /*
    this->CS_ON();
    this->SPIObj.beginTransaction(this->settings);
    this->SPIObj.transfer(_RESET);
    delay(2);
    this->SPIObj.endTransaction();
    this->CS_OFF();
    */

    //Alternative: Set GPIO RESET pin

    digitalWrite(this->_RESET, LOW);
    delayMicroseconds(t11);
    digitalWrite(this->_RESET, HIGH);
    delay(100);
    sendCommand(WAKEUP);

}

void ADS12xx::setPGA(uint8_t gainConfig) { //use the PGA_X values from ads1256.h to set the gain to X
    //Dont change bits [6:3] on ADCON Register -> Read current config first
    uint8_t ADCON_set; //to send
    uint8_t ADCON_current = readRegister(ADCON); //read current settings
    ADCON_set = ADCON_current;

    for (int i = 0; i < 3; i++) { //clear bits [0:2] (PGA Settings)
        CLEAR_BIT(ADCON_set, i);
    }


    ADCON_set |= gainConfig; //set the gain bits
    writeRegister(ADCON, ADCON_set);
}

void ADS12xx::startSPItransaction() {
    this->SPIObj.beginTransaction(this->settings);
}

void ADS12xx::stopSPItransaction() {
    this->SPIObj.endTransaction();
}

void ADS12xx::waitForDRDY() {
    while (DRDY_STATE.flag == 1) {
        delayMicroseconds(1);
    }
    noInterrupts();
    SET_BIT(DRDY_STATE.flag, 0); //SET BACK TO HIGH
    interrupts();
}

void ADS12xx::waitForDRDYSoftware() { //this needs to be used when DRDY is not on an interrupt-capable pin
    while (digitalRead(this->_DRDY) == HIGH){
        delayMicroseconds(5);
    }
}

void DRDYInterrupt() {
    CLEAR_BIT(DRDY_STATE.flag, 0);
}
