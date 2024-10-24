
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN 11     // Pin où le pin DATA du DHT22 est connecté
#define DHTTYPE DHT22   // Définir le type de DHT
DHT dht(DHTPIN, DHTTYPE);
#define DHT_POWER_SUPPLY_PIN 10

#include <avr/interrupt.h> 

#define MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT A0 
#define MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN A1
#define RESET_SIGFOX_MODULE 2//on utilise le RESET du module sigfox

////////////////////////////////////////////   pour modifier l'hibernation
const unsigned int MAX_COUNTER_POWER_DOWN_WAKE_UP =116; //2 ; // MUST BE >=2 , 116 for 15mnsNumber of WATCH DOG before starting the main software  
unsigned int counter_power_down_wake_up; // 
/////////////////////////////////////////////

//  Header byte = 1st Byte transmitted int he Sigox Tram ( SIGFOX #01 )
// Bit 7 = Sigfog Debug Mode, =1 for SIGFOX DEBUG MODE
// Bit 6 = Solar Panel Luminosity, measured during software excution ( =0 Voltage solar panel < Voltage Battery, =1 > 
// Bit 5 = FUll Charging Battery status , measured during software excution
// Bit 4 = Charging on going, measured during software excution
// Bit 0 to 3 = Software version : 0000 = Debug Software, then 0001 = V1 ...
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
byte header_byte = B00000000;

/////////////////////////////////////pour modifier le débuggage
boolean DEBUG_MODE = 0 ;// =1 or debbug , then ALLOW BLINKING LED and statement on Serial Monitor throught SERIAL RX/TX UART0  
//////////////////////////////////////

// put your setup code here, to run once:
void setup() {

  pinMode(RESET_SIGFOX_MODULE,OUTPUT);

  digitalWrite(RESET_SIGFOX_MODULE,LOW);
  delay (100);
  digitalWrite(RESET_SIGFOX_MODULE,HIGH);
  Serial.begin(9600);

  Serial.println("programme master module sigfox code reference 20220211 precision balances optimisee");

  if (DEBUG_MODE) {   
    Serial.println("SETUP DONE");
    
  }

  //1er message pour voir bon fonctionnement sigfox à l'allumage système
  Serial.println("AT$SF=12345678");

  counter_power_down_wake_up = MAX_COUNTER_POWER_DOWN_WAKE_UP;

  // Message AT popur Sigfox - 
  Serial.println("AT");  // Test after wake up from sleep mode
  Serial.flush() ;
  delay(100);

  Serial.println("AT$SF=SETUP DONE");

  Serial.println("AT$P=2"); // SLeep mode
  Serial.flush() ;

  // Save Power by writing all Digital IO to LOW EXCLUDING SPECIAL PORT
  for(int i=2; i<19; i++) {
    if (i != RESET_SIGFOX_MODULE) {
      pinMode(i,OUTPUT);
      digitalWrite(i,LOW);
    }
  }

  digitalWrite(RESET_SIGFOX_MODULE,HIGH);

  //SETUP WATCHDOG TIMER
  WDTCSR = B00011000; // (24 = B00011000);//change enable and WDE − also resets
  WDTCSR = B00100001; // (33 =  B00100001);//prescalers only − get rid of the WDE and WDCE bit
  WDTCSR = (1<<6);

  // SET UP CAN ANALOGU REFERENCE @ 1,1V INTERNAL
  analogReference(INTERNAL);

  // DISABLE ADC ( ADEN bit set to 0 in ADCSRA REGISTER ) 
  ADCSRA &= B01111111;

  //noInterrupts (); // timed sequence follows
  cli();  // Interrupts impossible

  // ENABLE SLEEP 
  //set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  SMCR |= (1<<2); //SET UP  SLEEP MODE = Power Down Mode
  //sleep_enable();
  SMCR |= 1; //Enable Sleep

  //interrupts (); // guarantees next instruction executed
  sei();
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------

// put your main code here, to run repeatedly:
void loop() {

  if (DEBUG_MODE) { 
    Serial.begin(9600);
    Serial.print("LOOP CNT = ");
    Serial.println (counter_power_down_wake_up);
    Serial.flush() ;
  }

  if (counter_power_down_wake_up == 0) {
  
    counter_power_down_wake_up  = MAX_COUNTER_POWER_DOWN_WAKE_UP;
  
    // PUT HERE THE CODE TO BE EXECUTED 

    Serial.begin(9600);

    pinMode(RESET_SIGFOX_MODULE,OUTPUT);

    digitalWrite(RESET_SIGFOX_MODULE,LOW);
    delay (100);
    digitalWrite(RESET_SIGFOX_MODULE,HIGH);
    // delay (100);

    if (DEBUG_MODE) { 
      Serial.println("CODE TO BE EXCUTED");
    }
  
    
    pinMode(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN,OUTPUT);
    digitalWrite(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN,HIGH);
    pinMode(MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT,INPUT); 
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    
    // Read temperature as Celsius
    float t = dht.readTemperature();
  
    digitalWrite(DHT_POWER_SUPPLY_PIN,LOW);
    
  
    uint8_t h_byte = h;

    int8_t  t_byte = (t + 35 )*2;

    if (DEBUG_MODE) { 
     
      Serial.print(" *C SIGFOX #03: ");
      Serial.print(t_byte,HEX);
      Serial.print(" Byte   ");

      Serial.print("Humidity : "); 
      Serial.print(h);
      Serial.print(" %\t SIGFOX #04: ");
      Serial.print(h_byte,HEX);
      Serial.println(" Byte   ");
    }
    ADCSRA |= B10000000;
  
    int  test = analogRead(MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT); // Une mesure pour rien pour eviter les erreurs ( 3F delta contaté), a voir si mettre une fois dans le setup
  
    uint8_t power_supply_voltage_Arduino= (analogRead(MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT)>>2);
     
    digitalWrite(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN,LOW); 
  
    ADCSRA &= B01111111;

    if (DEBUG_MODE) { 
      Serial.print("ADMUX: ");
      Serial.print(ADMUX,BIN);

      Serial.print("  ARDUINO VOLTAGE : ");
      Serial.print(power_supply_voltage_Arduino,BIN);

      Serial.print("  SIGFOX #02: ");
      Serial.print(power_supply_voltage_Arduino,HEX);

      Serial.println();
    }

    // Message AT popur Sigfox - 
    Serial.println("AT");  // Test after wake up from sleep mode 
    Serial.flush() ;
    delay(100);
    char Sigfox_message[34] = {0};  // 26 or 34 bytes max ( 1 or 2 HX711)

    sprintf(Sigfox_message,"AT$SF=%02x%02x%02x%02x%04x%04x",header_byte,power_supply_voltage_Arduino,t_byte,h_byte,0,0);

    Serial.println(Sigfox_message);
    Serial.flush() ;

    Serial.println("AT$P=2");   // SLeep mode
    Serial.flush() ;

  } else {   
    // Decrementaiton counter WDT
    counter_power_down_wake_up  =  counter_power_down_wake_up-1 ;
  }

  // Save Power by writing all Digital IO to LOW EXCLUDING SPECIAL PORT
  for(int i=2; i<19; i++) {
    if (i != RESET_SIGFOX_MODULE) {
      pinMode(i,OUTPUT);
      digitalWrite(i,LOW);
    }
  }

  // Brown-out Detector DISABLE , − this must be called right before the __asm__ sleep instruction
  MCUCR = bit (BODS) | bit (BODSE)  ; // | bit(PUD) ;  // Do not forget to diseable ALL IO Pull up by >Turn PUD bit in the MCUCR // TBCHECK ???
  MCUCR = bit (BODS);
  // MCUCR |=(3<<5); // Set both BODS and BODSE  at the same time
  // MCUCR = (MCUCR & ~(1 <<5)) | (1<<6); // then set the BODS bit and clear the BODSE at the same time
    
  //sleep_cpu (); // sleep within 3 clock cycles of above
  __asm__ __volatile__("sleep");
}

//needed for the digital input interrupt
//void digitalInterrupt() {
// }

// watchdog interrupt
//DON'T FORGET THIS!  Needed for the watch dog timer.  This is called after a watch dog timer timeout - this is the interrupt function called after waking up
ISR(WDT_vect) {

  if (DEBUG_MODE) {      
    Serial.println("ISR_WDT VECTOR");
    Serial.flush() ;
  }
}
