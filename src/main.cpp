#include <Arduino.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

// déclaration des ports des boutons
int BP0 = 0;
int BP1 = 2;
int BP2 = 12;

// declaration du potentiometre
int pot = 33;

// declaration du PWM
int PWM = 27;
int phase = 26;
int ON_PWM = 25;

// caracteristique de la PWM
int freq = 25000;
int canal0 = 0;
int resolution = 11;

int Val_BP0;
int Val_BP1;
int Val_BP2;
int Val_pot;

// Paramétrage du rétroéclairage
const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

void setup()
{
  // Initialise la liaison avec le terminal
  Serial.begin(115200);

  // Configuration des ports pour les boutons
  pinMode(BP0, INPUT_PULLUP);
  pinMode(BP1, INPUT_PULLUP);
  pinMode(BP2, INPUT_PULLUP);

  pinMode(PWM, OUTPUT);
  pinMode(phase, OUTPUT);
  pinMode(ON_PWM, OUTPUT);

  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);
  lcd.setRGB(colorR, colorG, colorB);

  // config PWM
  ledcSetup(canal0, freq, resolution);

  // liaison du canal du PWM avec les broches de l'ESP32
  ledcAttachPin(PWM, canal0);
}

void loop()
{
  // Lecture de la valeur des boutons
  Val_BP0 = digitalRead(BP0);
  Val_BP1 = digitalRead(BP1);
  Val_BP2 = digitalRead(BP2);

  Val_pot = analogRead(pot);

  // Message dans le terminal
  Serial.printf("bp0=%d  bp1=%d\n bp2=%d", Val_BP0, Val_BP1, Val_BP2);

  // Affichage sur l'écran LCD
  lcd.setCursor(0, 0);
  lcd.printf("potar = %d", Val_pot);
  delay(300);

  // ecriture rapport cyclique 
  //digitalWrite(phase, BP1);
  //digitalWrite(ON_PWM, 0);
  ledcWrite(canal0, 1024);
}
