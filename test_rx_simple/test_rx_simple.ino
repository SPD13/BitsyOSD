/*
 * arduino-rx5808-testing
 * 
 * Hardware:
 * - 128x64 I2C OLED display connected to TWI (A4 & A5)
 * - RX5808 with SPI mod, connected to D4-D6 (CH1-CH3) and A2 (RSSI)
 * - SD Card reader connected to same SPI bus, chip select on D10
 *
 * SPI and RX5808 code taken from:
 * https://github.com/voroshkov/Chorus-RF-Laptimer
 */

// --------------------------------------------------------

// Constants for band selection
#define RX_BAND_R 0
#define RX_BAND_A 1
#define RX_BAND_B 2
#define RX_BAND_E 3
#define RX_BAND_F 4
#define RX_BAND_D 5

// Constants for channel selection
#define RX_CHAN_1 0
#define RX_CHAN_2 1
#define RX_CHAN_3 2
#define RX_CHAN_4 3
#define RX_CHAN_5 4
#define RX_CHAN_6 5
#define RX_CHAN_7 6
#define RX_CHAN_8 7

// Selected band & channel
#define RX_BAND RX_BAND_A
#define RX_CHAN RX_CHAN_3

// ADC & GPIO pin configuration
#define RSSI_ADC A3
#define SPI_DATA_PIN 10
#define SPI_SS_PIN 11
#define SPI_CLOCK_PIN 12

// Number of ADC reads (averaged) per iteration
#define ADC_AVERAGE_READS 5

// --------------------------------------------------------

const uint16_t channelTable[] PROGMEM = {
    // Channel 1 - 8
    0x281D, 0x288F, 0x2902, 0x2914, 0x2987, 0x2999, 0x2A0C, 0x2A1E, // Raceband
    0x2A05, 0x299B, 0x2991, 0x2987, 0x291D, 0x2913, 0x2909, 0x289F, // Band A
    0x2903, 0x290C, 0x2916, 0x291F, 0x2989, 0x2992, 0x299C, 0x2A05, // Band B
    0x2895, 0x288B, 0x2881, 0x2817, 0x2A0F, 0x2A19, 0x2A83, 0x2A8D, // Band E
    0x2906, 0x2910, 0x291A, 0x2984, 0x298E, 0x2998, 0x2A02, 0x2A0C, // Band F / Airwave
    0x2609, 0x261C, 0x268E, 0x2701, 0x2713, 0x2786, 0x2798, 0x280B // Band D / 5.3
};

const uint16_t channelFreqTable[] PROGMEM = {
    // Channel 1 - 8
    5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917, // Raceband
    5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A
    5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B
    5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E
    5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // Band F / Airwave
    5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621 // Band D / 5.3
};

void SERIAL_SENDBIT1() {
    digitalWrite(SPI_CLOCK_PIN, LOW);
    delayMicroseconds(1);

    digitalWrite(SPI_DATA_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(SPI_CLOCK_PIN, HIGH);
    delayMicroseconds(1);

    digitalWrite(SPI_CLOCK_PIN, LOW);
    delayMicroseconds(1);
}

void SERIAL_SENDBIT0() {
    digitalWrite(SPI_CLOCK_PIN, LOW);
    delayMicroseconds(1);

    digitalWrite(SPI_DATA_PIN, LOW);
    delayMicroseconds(1);
    digitalWrite(SPI_CLOCK_PIN, HIGH);
    delayMicroseconds(1);

    digitalWrite(SPI_CLOCK_PIN, LOW);
    delayMicroseconds(1);
}

void SERIAL_ENABLE_LOW() {
    delayMicroseconds(1);
    digitalWrite(SPI_SS_PIN, LOW);
    delayMicroseconds(1);
}

void SERIAL_ENABLE_HIGH() {
    delayMicroseconds(1);
    digitalWrite(SPI_SS_PIN, HIGH);
    delayMicroseconds(1);
}

void setChannelModule(uint8_t channel, uint8_t band) {
    uint8_t i;
    uint16_t channelData;
    uint16_t channelFreq;

    channelData = pgm_read_word_near(channelTable + channel + (8 * band));

    channelFreq = pgm_read_word_near(channelFreqTable + channel + (8 * band));
    Serial.print("Frequency: ");
    Serial.println(channelFreq);

    // bit bang out 25 bits of data
    // Order: A0-3, !R/W, D0-D19
    // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
    SERIAL_ENABLE_HIGH();
    delayMicroseconds(1);
    SERIAL_ENABLE_LOW();

    SERIAL_SENDBIT0();
    SERIAL_SENDBIT0();
    SERIAL_SENDBIT0();
    SERIAL_SENDBIT1();

    SERIAL_SENDBIT0();

    // remaining zeros
    for (i = 20; i > 0; i--) {
        SERIAL_SENDBIT0();
    }

    // Clock the data in
    SERIAL_ENABLE_HIGH();
    delayMicroseconds(1);
    SERIAL_ENABLE_LOW();

    // Second is the channel data from the lookup table
    // 20 bytes of register data are sent, but the MSB 4 bits are zeros
    // register address = 0x1, write, data0-15=channelData data15-19=0x0
    SERIAL_ENABLE_HIGH();
    SERIAL_ENABLE_LOW();

    // Register 0x1
    SERIAL_SENDBIT1();
    SERIAL_SENDBIT0();
    SERIAL_SENDBIT0();
    SERIAL_SENDBIT0();

    // Write to register
    SERIAL_SENDBIT1();

    // D0-D15
    //   note: loop runs backwards as more efficent on AVR
    for (i = 16; i > 0; i--) {
        // Is bit high or low?
        if (channelData & 0x1) {
            SERIAL_SENDBIT1();
        }
        else {
            SERIAL_SENDBIT0();
        }
        // Shift bits along to check the next one
        channelData >>= 1;
    }

    // Remaining D16-D19
    for (i = 4; i > 0; i--) {
        SERIAL_SENDBIT0();
    }

    // Finished clocking data in
    SERIAL_ENABLE_HIGH();
    delayMicroseconds(1);

    digitalWrite(SPI_SS_PIN, LOW);
    digitalWrite(SPI_CLOCK_PIN, LOW);
    digitalWrite(SPI_DATA_PIN, LOW);
}

void setup(void) {
    pinMode(SPI_SS_PIN, OUTPUT);
    pinMode(SPI_DATA_PIN, OUTPUT);
    pinMode(SPI_CLOCK_PIN, OUTPUT);

    Serial.begin(9600);

    // Set RX channel
    setChannelModule(RX_CHAN, RX_BAND);

    delay(3000);
    Serial.println("Started");
}

void loop(void) {
    // Read average RSSI value
    int rssi = 0;
    for (int i = 0; i < ADC_AVERAGE_READS; i++) {
        rssi += analogRead(RSSI_ADC);
    }
    rssi = rssi / ADC_AVERAGE_READS;
    Serial.println(rssi);
    delay(500);
}
