#include <Arduino.h>
#include "rgb_lcd.h"
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

void vTaskPeriodic(void *pvParameters);

Adafruit_TCS34725 tcs;

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

//servomoteur
int val_servo;
Servo myservo;

//accelerometre
Adafruit_MPU6050 mpu;

// asservissement
int csg_vitesse;
float integ = 0;

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

  // Config Capteur couleur
  Wire.begin();

  // liaison du canal du PWM avec les broches de l'ESP32
  ledcAttachPin(PWM, canal0);

  Serial.begin(115200);
  Serial.printf("Initialisations\n");

  // Initialisation du capteur de couleur TCS34725
  if (tcs.begin())
  {
    Serial.println("TCS34725 trouvé");
  }
  else
  {
    Serial.println("TCS34725 non trouvé - vérifier les connexions");
  }

  //initialisation servomoteur
  ESP32PWM::allocateTimer(2);
	myservo.setPeriodHertz(50);
  myservo.attach(13);

  //initialisation accélérometre
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!"); 
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

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

  // capteur de couleur
  uint16_t r, g, b, c, colorTemp;

  tcs.getRawData(&r, &g, &b, &c);

  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  //Serial.printf("%d, %d, %d, %d\n", IR_Seuil, Val_IR0, etat, Val_pot);

  // accelerometre 
    sensors_event_t a, gyro, temp;
    mpu.getEvent( &a, &gyro, &temp);
    float angle = atan2(a.acceleration.y, -a.acceleration.x)*180/M_PI;

    Serial.println(angle);                                        

  switch (etat)
  {
  case 1: // initialisation 
    digitalWrite(ON_PWM, 1); 

    myservo.write(1456); // servomoteur 90°
    csg_vitesse = 2;
    if (Val_IR0 > IR_Seuil) // bras position initial
    {
      etat = 2;
    }
    break;

  case 2:
    encoder.setCount(0); // arret du bras 
    encoder_count = 0;
    step_count = 0;
    digitalWrite(ON_PWM, 0);
    ledcWrite(canal0, 0);
    csg_vitesse = 0;
    integ = 0;
    if (Val_BP0 == 0) // appui BP bleu
    {
      step_count = 100;
      etat = 3;
    }
    else if (Val_BP1 == 0) // appui BP jaune
    {
      step_count = -100;
      etat = 5;
    }
    break;

  case 3: // avance le bras d'un cran (sens des aiguilles d'une montre) jusque ce que le encoder soit superieur au step_count
    digitalWrite(ON_PWM, 1);
    csg_vitesse = 2;
    if (encoder.getCount() > step_count)
    {
      etat = 4;
    }
    break;

  case 4: // arrete le bras pendant 1s puis incremente step_count
    digitalWrite(ON_PWM, 0);
    ledcWrite(canal0, 0);
    step_count += 100;
    csg_vitesse = 0;
    integ = 0;
    delay(1000);
    etat = 3;

    if (step_count > 540)
    {
      etat = 7;
    }
    break;

  case 5: //avance le bras d'un cran (sens inverse des aiguilles d'une montre) jusque ce que le encoder soit inferieur au step_count
    digitalWrite(ON_PWM, 1);
    csg_vitesse = -2;
    if (encoder.getCount() < step_count)
    {
      etat = 6;
    }
    break;

  case 6: // arrete le bras pendant 1s puis incremente step_count
    digitalWrite(ON_PWM, 0);
    ledcWrite(canal0, 0);
    csg_vitesse = 0;
    integ = 0;

    delay(1000);
    step_count -= 100;
    etat = 5;

    if (step_count < -840)
    {
      etat = 2;
    }
    break;

  case 7: // detection de la couleur de la balle  
    if (g==1 & b==1) // balle jaune
    {
      etat = 8;
    }
    else if (r==1 & b==2) //balle blanche
    {
      etat = 10;
    }
    break;

  case 8: // positionne la balle jaune sur le servomoteur 
    digitalWrite(ON_PWM, 1);
    csg_vitesse = 2;
    if (encoder.getCount() > 640)
    {
      etat = 9;
    }
    break;

  case 9: // arret du bras + ejection de la balle par le servomoteur 
    digitalWrite(ON_PWM, 0);
    ledcWrite(canal0, 0);
    csg_vitesse = 0;
    integ = 0;

    delay(3000);
    myservo.write(1140);
    delay(3000);
    etat = 1;
    
    break;

  case 10: // positionne la balle blanche sur la detection de balle 
    digitalWrite(ON_PWM, 1);
    csg_vitesse = 1;
    if (encoder.getCount() > 730)
    {
      etat = 11;
    }
    break;

  case 11: // arret du bras 
    digitalWrite(ON_PWM, 0);
    ledcWrite(canal0, 0);
    step_count += 100;
    csg_vitesse = 0;
    integ = 0;
    delay(1000);
    etat = 10;

    if (step_count > 730) 
    {
      etat = 12;
    }
    break;

  case 12: // vérification de la présence de la balle (au cas où ou on la retire manuellement) 
    if (Val_IR0 < 100)
    {
      etat = 1;
    }
    else 
    {
      etat = 12;
    }

  }
}  

void vTaskPeriodic(void *pvParameters)
{
  const float Kp = 30.0f; // gain proportionnel (ajuster)
  const float Ki = 8.0f;  // gain integral (ajuster)

  int encoder_memo = 0;
  int image_vitesse;
  long encoder_count;
  float erreur;
  float commande;
  float prev_erreur = 0.0f;

  TickType_t xLastWakeTime;
  // Lecture du nombre de ticks quand la tâche commence
  xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    encoder_count = encoder.getCount();
    image_vitesse = encoder_count - encoder_memo;
    encoder_memo = encoder_count;

    erreur = (float)csg_vitesse - (float)image_vitesse;

    integ = (integ + erreur);

    // Correcteur
    float commande_f = Kp * erreur + Ki * integ;

    int commande = (int)roundf(commande_f);
    commande = constrain(commande, -2047, 2047); // résolution 11 bits

    if (commande > 0)
    {
      digitalWrite(phase, 0);
      ledcWrite(canal0, commande);
    }
    else
    {
      digitalWrite(phase, 1);
      ledcWrite(canal0, -commande);
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
  }
}