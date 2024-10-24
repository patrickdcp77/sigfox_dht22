#include <Arduino.h> // Inclut la bibliothèque principale d'Arduino pour utiliser ses fonctions et macros
#include <avr/sleep.h> // Inclut la bibliothèque AVR pour les fonctions de gestion du mode veille
#include <avr/power.h> // Inclut la bibliothèque AVR pour les fonctions de gestion de l'alimentation
#include <Adafruit_Sensor.h> // Inclut la bibliothèque Adafruit Sensor pour les capteurs
#include <DHT.h> // Inclut la bibliothèque DHT pour les capteurs de température et d'humidité DHT
#include <DHT_U.h> // Inclut la bibliothèque DHT Unified pour une interface unifiée avec les capteurs DHT

#define DHTPIN 11 // Définit la broche 11 comme la broche de connexion du capteur DHT
#define DHTTYPE DHT22 // Définit le type de capteur DHT comme étant le DHT22

DHT dht(DHTPIN, DHTTYPE); // Crée une instance de l'objet DHT avec la broche et le type spécifiés

#define DHT_POWER_SUPPLY_PIN 10 // Définit la broche 10 comme la broche d'alimentation du capteur DHT

#include <avr/interrupt.h> // Inclut la bibliothèque AVR pour les fonctions de gestion des interruptions

#define MICROPROCESOR_VOLTAGE_CAN_MEASUREMENT A0 // Définit la broche A0 pour la mesure de la tension du microprocesseur
#define MICROPROCESOR_VOLTAGE_CAN_PULL_UP_PIN A1 // Définit la broche A1 pour le pull-up de la mesure de la tension du microprocesseur
#define RESET_SIGFOX_MODULE 2 // Définit la broche 2 pour le reset du module Sigfox

const unsigned int MAX_COUNTER_POWER_DOWN_WAKE_UP = 116;
unsigned int counter_power_down_wake_up;

byte header_byte = B00000000;

boolean DEBUG_MODE = 0;

void setup() {
  analogReference(INTERNAL); // Définit la référence de tension analogique sur la référence interne
  ADCSRA &= B01111111; // Désactive l'ADC pour économiser de l'énergie

  cli(); // Désactive les interruptions globales
  SMCR |= (1 << 2); // Configure le mode veille pour le mode "Power-down"
  SMCR |= 1; // Active le mode veille
  sei(); // Réactive les interruptions globales
}

void loop() {
  if (DEBUG_MODE) {
    Serial.begin(9600); // Initialise la communication série à 9600 bauds
    Serial.print("LOOP CNT = "); // Affiche "LOOP CNT = " sur le moniteur série
    Serial.println(counter_power_down_wake_up); // Affiche la valeur du compteur de réveil sur le moniteur série
    Serial.flush(); // Attend que toutes les données soient envoyées sur le port série
  }

  if (counter_power_down_wake_up == 0) { // Vérifie si le compteur de réveil est à 0
    counter_power_down_wake_up = MAX_COUNTER_POWER_DOWN_WAKE_UP; // Réinitialise le compteur de réveil à la valeur maximale

    Serial.begin(9600); // Initialise la communication série à 9600 bauds
    pinMode(RESET_SIGFOX_MODULE, OUTPUT); // Configure la broche de reset du module Sigfox comme une sortie
    digitalWrite(RESET_SIGFOX_MODULE, LOW); // Met la broche de reset du module Sigfox à LOW pour le réinitialiser
    delay(100); // Attend 100 millisecondes
    digitalWrite(RESET_SIGFOX_MODULE, HIGH); // Met la broche de reset du module Sigfox à HIGH pour terminer la réinitialisation

    if (DEBUG_MODE) {
      Serial.println("CODE TO BE EXECUTED"); // Si le mode DEBUG est activé, affiche "CODE TO BE EXECUTED" sur le moniteur série
    }

    // Ajoutez ici le code à exécuter lorsque le compteur de réveil est réinitialisé
  }

  counter_power_down_wake_up--; // Décrémente le compteur de réveil

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Configure le mode veille pour le mode "Power-down"
  sleep_enable(); // Active le mode veille
  sleep_cpu(); // Met le microcontrôleur en mode veille

  // Le microcontrôleur se réveille ici après une interruption
  sleep_disable(); // Désactive le mode veille après le réveil
}

ISR(WDT_vect) {
  if (DEBUG_MODE) {
    Serial.println("ISR_WDT VECTOR"); // Si le mode DEBUG est activé, affiche "ISR_WDT VECTOR" sur le moniteur série
    Serial.flush(); // Attend que toutes les données soient envoyées sur le port série
  }
}
