/*
 * ----------------------------------------------------------------------------
 * This is a MFRC522 library example; see https://github.com/miguelbalboa/rfid
 * for further details and other examples.
 * 
 * NOTE: The library file MFRC522.h has a lot of useful info. Please read it.
 * 
 * Released into the public domain.
 * ----------------------------------------------------------------------------
 * Example sketch/program which will try the most used default keys listed in 
 * https://code.google.com/p/mfcuk/wiki/MifareClassicDefaultKeys to dump the
 * block 0 of a MIFARE RFID card using a RFID-RC522 reader.
 * 
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno           Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 *
 */

#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

volatile bool readCard = false;
volatile bool hasStdby = false;

#define RST_PIN             9           // Configurable, see typical pin layout above
#define SS_PIN              10          // Configurable, see typical pin layout above
#define BUZZER_PIN          8
#define STDBY_TOGGLE_PIN    7
#define READY_TOGGLE_PIN    6
#define SUCCESS             0
#define STD_BY              2
#define FAIL_REPETITION     4

#define I2C_ADDR    0x27  // Define I2C Address for the PCF8574T 
//---(Following are the PCF8574 pin assignments to LCD connections )----
// This are different than earlier/different I2C LCD displays
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

#define  LED_OFF  1
#define  LED_ON  0

//LCD_I2C lcd = LCD_I2C(0x27,2,16);             // Number of lines and i2c address of the display
LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
uint8_t calcStartIndex(uint8_t strLength, uint8_t lcdCol = 16);
String lcdAlignString(String str);
void lcd_homescreen(void);
void ringBuzzer(uint8_t);
void (*resetFunc)(void) = 0;

// Number of known default keys (hard-coded)
// NOTE: Synchronize the NR_KNOWN_KEYS define with the defaultKeys[] array
#define NR_KNOWN_KEYS   8
// Known keys, see: https://code.google.com/p/mfcuk/wiki/MifareClassicDefaultKeys
byte knownKeys[NR_KNOWN_KEYS][MFRC522::MF_KEY_SIZE] =  {
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, // FF FF FF FF FF FF = factory default
    {0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5}, // A0 A1 A2 A3 A4 A5
    {0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5}, // B0 B1 B2 B3 B4 B5
    {0x4d, 0x3a, 0x99, 0xc3, 0x51, 0xdd}, // 4D 3A 99 C3 51 DD
    {0x1a, 0x98, 0x2c, 0x7e, 0x45, 0x9a}, // 1A 98 2C 7E 45 9A
    {0xd3, 0xf7, 0xd3, 0xf7, 0xd3, 0xf7}, // D3 F7 D3 F7 D3 F7
    {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff}, // AA BB CC DD EE FF
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  // 00 00 00 00 00 00
};

SoftwareSerial Serial1 = SoftwareSerial(A0,A1);
String clean_res = "";
char cTmp;

/*
 * Initialize.
 */
void setup() {
 
    lcd.begin(16,2);               // initialize the lcd 

    lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(HIGH);

    lcd_homescreen();

    pinMode(STDBY_TOGGLE_PIN, INPUT);
    pinMode(READY_TOGGLE_PIN, INPUT);
    if(digitalRead(STDBY_TOGGLE_PIN) == HIGH){
        hasStdby = true;
    }

    Serial.begin(115200);         // Initialize serial communications with the PC
    while (!Serial);            // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)  
    Serial1.begin(19200);
    //Serial1.setTimeout(3000);
    SPI.begin();                // Init SPI bus
    mfrc522.PCD_Init();         // Init MFRC522 card

    Serial.print("Ver: 0x");      
    byte readReg = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
    Serial.println(readReg, HEX);  
   
    pinMode(BUZZER_PIN, OUTPUT);
    ringBuzzer(0);
}

/*
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dump_byte_array(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
        clean_res += String(buffer[i], HEX);
    }
}

void ringBuzzer(uint8_t mode = 1){
   
  if(mode == SUCCESS){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(80);
    digitalWrite(BUZZER_PIN, LOW);
  }
  else if(mode == STD_BY){

    for(uint8_t iLoop = 0; iLoop < 2; iLoop++){

      digitalWrite(BUZZER_PIN, HIGH);
      delay(60);
      digitalWrite(BUZZER_PIN, LOW);
      delay(60);
    }

    delay(3000);
  }
  else{
    for(uint8_t iLoop = 0; iLoop < FAIL_REPETITION; iLoop++){

      digitalWrite(BUZZER_PIN, HIGH);
      delay(25);
      digitalWrite(BUZZER_PIN, LOW);
      delay(25);
    }
  }
}

/*
 * Main loop.
 */
void loop() {

    if( hasStdby ){
        ringBuzzer(STD_BY);
    }
    
    // Look for new cards
    if ( ! mfrc522.PICC_IsNewCardPresent())
        return;

    // Select one of the cards
    if ( ! mfrc522.PICC_ReadCardSerial())
        return;

    clean_res = "";
    ringBuzzer(SUCCESS);
    dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    Serial.print("@");  
    Serial.println(clean_res);

    Serial1.print("@");  
    Serial1.println(clean_res);

    Serial.println("Processing");

    lcd.home();
    lcd.clear();
    lcd.print(lcdAlignString("Sedang diproses"));
    while(digitalRead(READY_TOGGLE_PIN) != HIGH){
        ;
    };
    
    clean_res = Serial1.readString();

    StaticJsonBuffer<160> jsonBuffer;
    JsonObject &jsonObject = jsonBuffer.parseObject(clean_res);
    String msg;
    uint8_t error = jsonObject["error"];
    String cr = jsonObject["cr"];

    lcd.home();
    lcd.clear();

    if( error != 0 ){
        msg = lcdAlignString("Error INET");
        lcd.print(msg);
    }
    else{
        if( cr.length() == 0){
            msg = lcdAlignString("Error CR");
            lcd.print(msg);
        }
        else{
            msg = lcdAlignString("Sisa saldo anda");
            lcd.print(msg);
            msg = lcdAlignString("Rp " + cr);
            Serial.println(msg);
            lcd.setCursor(0,1);
            lcd.print(msg);
        }
    }

    Serial.println(clean_res);
    Serial.println("Ready");
    delay(2000);

    lcd_homescreen();
    //resetFunc();
}

void lcd_homescreen(void){
    lcd.clear();
    lcd.home();
    lcd.print( lcdAlignString("Selamat Datang"));    
    lcd.setCursor(0,1);
    lcd.print("   Rp 15,000    ");
}

uint8_t calcStartIndex(uint8_t strLength, uint8_t lcdCol ){
    uint8_t tmp = ((lcdCol - strLength) / 2) ;   
    return tmp;
}

String lcdAlignString(String str){
    String tmp;
    uint8_t idx = calcStartIndex( str.length() );

    for(int i=0; i < idx; i++){
        tmp += " ";
    }

    tmp += str;

    return tmp;
}
