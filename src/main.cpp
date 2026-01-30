#include <Arduino.h>
#include "rgb_lcd.h"
#include <ESP32Encoder.h>

void vTaskPeriodic(void *pvParameters);

rgb_lcd lcd;

// déclaration des ports des boutons
int BP0 = 0;
int BP1 = 2;
int BP2 = 12;

// declaration du potentiometre
int pot = 33;

// Déclaration du capteur IR
int IR0 = 36; // broche 36
int Val_IR0;

// Déclaration du codeur
ESP32Encoder encoder;
long encoder_count = 0;
long step_count = 0;

// Déclaration de la machine a état
int etat = 1;

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

// Seuil de détection du capteur IR (à ajuster selon votre capteur)
int IR_Seuil = 2000;

// Paramétrage du rétroéclairage
int colorR = 255;
int colorG = 0;
int colorB = 0;

int csg_vitesse;

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

  // Utilise attachFullQuad pour lire tous les changements
  encoder.attachFullQuad(19, 23);
  encoder.setCount(0);

  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);
  lcd.setRGB(colorR, colorG, colorB);

  // config PWM
  ledcSetup(canal0, freq, resolution);

  // liaison du canal du PWM avec les broches de l'ESP32
  ledcAttachPin(PWM, canal0);

  Serial.begin(115200);
  Serial.printf("Initialisations\n");

  // Création de la tâche périodique
  xTaskCreate(vTaskPeriodic, "vTaskPeriodic", 10000, NULL, 2, NULL);
}

void loop()
{
  // Lecture de la valeur des boutons
  Val_BP0 = digitalRead(BP0);
  Val_BP1 = digitalRead(BP1);
  Val_BP2 = digitalRead(BP2);

  Val_pot = analogRead(pot);

  // Lecture de la valeur du capteur IR
  Val_IR0 = analogRead(IR0);

  // Lecture de la valeur du codeur
  encoder_count = encoder.getCount();

  // Message dans le terminal
  // Serial.printf("bp0=%d  bp1=%d  bp2=%d  IR0=%d  encoder=%ld  etat=%d\n", Val_BP0, Val_BP1, Val_BP2, Val_IR0, encoder_count, etat);

  // Affichage sur l'écran LCD
  lcd.setCursor(0, 0);
  lcd.printf("IR0=%d", Val_IR0);
  lcd.setCursor(0, 1);
  lcd.printf("enc=%ld  etat=%d", encoder_count, etat);


  switch (etat)
  {
  case 1:
    digitalWrite(phase, 0);
    digitalWrite(ON_PWM, 1);
    ledcWrite(canal0, 600);
    if (Val_IR0 > IR_Seuil)
    {
      etat = 2;
    }
    break;

  case 2:
    encoder.setCount(0);
    encoder_count = 0;
    step_count = 0;
    digitalWrite(ON_PWM, 0);
    ledcWrite(canal0, 0);
    if (Val_BP0 == 0)
    {
      step_count = 100;
      etat = 3;
    }
    else if (Val_BP1 == 0)
    {
      step_count = -100;
      etat = 5;
    }
    break;

  case 3:
    digitalWrite(phase, 0);
    digitalWrite(ON_PWM, 1);
    ledcWrite(canal0, 600);

    if (encoder.getCount() > step_count)
    {
      etat = 4;
    }
    break;

  case 4:
    digitalWrite(ON_PWM, 0);
    ledcWrite(canal0, 0);
    step_count += 100;
    delay(1000);
    etat = 3;

    if (step_count > 840)
    {
      etat = 2;
    }
    break;

  case 5:
    digitalWrite(phase, 1);
    digitalWrite(ON_PWM, 1);
    ledcWrite(canal0, 600);
    if (encoder.getCount() < step_count)
    {
      etat = 6;
    }
    break;

  case 6:
    digitalWrite(ON_PWM, 0);
    ledcWrite(canal0, 0);

    delay(1000);
    step_count -= 100;
    etat = 5;

    if (step_count < -840)
    {
      etat = 2;
    }
    break;
  }
}

void vTaskPeriodic(void *pvParameters)
{
  int encoder_memo=0;
  int image_vitesse;
  long encoder_count;
  int erreur;
  int commande;
  TickType_t xLastWakeTime;
  // Lecture du nombre de ticks quand la tâche commence
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    Serial.printf("A répéter\n");
    encoder_count = encoder.getCount(); 
    image_vitesse = encoder_count - encoder_memo;
    encoder_memo = encoder_count;

    erreur = csg_vitesse - image_vitesse;

    // Asservissement proportionnel
    commande = 550 + (erreur * 5);   // 5 est le gain proportionnel à ajuster
    commande = constrain(commande, 0, 2047);  // Limite entre 0 et 2047 (résolution 11 bits)
    ledcWrite(canal0, commande);

    Serial.printf("image_vitesse=%d  erreur=%d  commande=%d\n", image_vitesse, erreur, commande);

    // Endort la tâche pendant le temps restant par rapport au réveil,      
    // ici 100ms, donc la tâche s'effectue ici toutes les 100ms.
    // xLastWakeTime sera mis à jour avec le nombre de ticks au prochain
    // réveil de la tâche.
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
  }
}