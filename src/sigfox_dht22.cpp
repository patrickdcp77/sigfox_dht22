#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN 11
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
#define DHT_POWER_SUPPLY_PIN 10

#include <avr/interrupt.h>

#define MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT A0
#define MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN A1
#define RESET_SIGFOX_MODULE 2

const unsigned int MAX_COUNTER_POWER_DOWN_WAKE_UP = 116;
unsigned int counter_power_down_wake_up;

byte header_byte = B00000000;

boolean DEBUG_MODE = 0;

void setup() {
  pinMode(RESET_SIGFOX_MODULE, OUTPUT);
  digitalWrite(RESET_SIGFOX_MODULE, LOW);
  delay(100);
  digitalWrite(RESET_SIGFOX_MODULE, HIGH);
  Serial.begin(9600);

  Serial.println("programme master module sigfox code reference 20220211 precision balances optimisee");

  if (DEBUG_MODE) {
    Serial.println("SETUP DONE");
  }

  Serial.println("AT$SF=12345678");

  counter_power_down_wake_up = MAX_COUNTER_POWER_DOWN_WAKE_UP;

  Serial.println("AT");
  Serial.flush();
  delay(100);

  Serial.println("AT$SF=SETUP DONE");
  Serial.println("AT$P=2");
  Serial.flush();

  for (int i = 2; i < 19; i++) {
    if (i != RESET_SIGFOX_MODULE) {
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
    }
  }

  digitalWrite(RESET_SIGFOX_MODULE, HIGH);

  WDTCSR = B00011000;
  WDTCSR = B00100001;
  WDTCSR = (1 << 6);

  analogReference(INTERNAL);
  ADCSRA &= B01111111;

  cli();
  SMCR |= (1 << 2);
  SMCR |= 1;
  sei();
}

void loop() {
  if (DEBUG_MODE) {
    Serial.begin(9600);
    Serial.print("LOOP CNT = ");
    Serial.println(counter_power_down_wake_up);
    Serial.flush();
  }

  if (counter_power_down_wake_up == 0) {
    counter_power_down_wake_up = MAX_COUNTER_POWER_DOWN_WAKE_UP;

    Serial.begin(9600);
    pinMode(RESET_SIGFOX_MODULE, OUTPUT);
    digitalWrite(RESET_SIGFOX_MODULE, LOW);
    delay(100);
    digitalWrite(RESET_SIGFOX_MODULE, HIGH);

    if (DEBUG_MODE) {
      Serial.println("CODE TO BE EXCUTED");
    }

    pinMode(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN, OUTPUT);
    digitalWrite(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN, HIGH);
    pinMode(MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT, INPUT);

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    digitalWrite(DHT_POWER_SUPPLY_PIN, LOW);

    uint8_t h_byte = h;
    int8_t t_byte = (t + 35) * 2;

    if (DEBUG_MODE) {
      Serial.print(" *C SIGFOX #03: ");
      Serial.print(t_byte, HEX);
      Serial.print(" Byte   ");

      Serial.print("Humidity : ");
      Serial.print(h);
      Serial.print(" %\t SIGFOX #04: ");
      Serial.print(h_byte, HEX);
      Serial.println(" Byte   ");
    }

    ADCSRA |= B10000000;
    
    uint8_t power_supply_voltage_Arduino = (analogRead(MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT) >> 2);

    digitalWrite(MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN, LOW);
    ADCSRA &= B01111111;

    if (DEBUG_MODE) {
      Serial.print("ADMUX: ");
      Serial.print(ADMUX, BIN);

      Serial.print("  ARDUINO VOLTAGE : ");
      Serial.print(power_supply_voltage_Arduino, BIN);

      Serial.print("  SIGFOX #02: ");
      Serial.print(power_supply_voltage_Arduino, HEX);

      Serial.println();
    }

    Serial.println("AT");
    Serial.flush();
    delay(100);
    char Sigfox_message[34] = {0};

    sprintf(Sigfox_message, "AT$SF=%02x%02x%02x%02x%04x%04x", header_byte, power_supply_voltage_Arduino, t_byte, h_byte, 0, 0);

    Serial.println(Sigfox_message);
    Serial.flush();

    Serial.println("AT$P=2");
    Serial.flush();
  } else {
    counter_power_down_wake_up--;
  }

  for (int i = 2; i < 19; i++) {
    if (i != RESET_SIGFOX_MODULE) {
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
    }
  }

  MCUCR = bit(BODS) | bit(BODSE);
  MCUCR = bit(BODS);

  __asm__ __volatile__("sleep");
}

ISR(WDT_vect) {
  if (DEBUG_MODE) {
    Serial.println("ISR_WDT VECTOR");
    Serial.flush();
  }
}
