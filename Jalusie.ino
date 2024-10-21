#include <LiquidCrystal.h>
#include <Servo.h>

int lightPin = 0;
int lightIntesity = 0;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
Servo myservo;
int pos = 0;
int buttonMode = 13;
int buttonSet = 5;
String mode = "manual";
int old_angle = 180;
int new_angle = 0;
int step = 1;
int potPin = A1;  // Der Pin, an dem das Potentiometer angeschlossen ist
int potValue = 0; // Variable zum Speichern des Potentiometerwerts
int servoAngle = 0; // Variable zum Speichern des Servo-Winkels
int targetIntesity = 50;

void setup() {
  myservo.attach(6);
  lcd.begin(16, 2);
  pinMode(buttonMode, INPUT_PULLUP);
  pinMode(buttonSet, INPUT_PULLUP);
  Serial.begin(9600);
  while (!Serial);
}

void loop() {
  // MANUAL MODUS
  //Wann Mode ist manual wir bekomen das wert von potenzimeter und bewege den servo als mit dem potenziometer eingerichtet
  if (mode == "manual") {
    potValue = analogRead(potPin);  // Lese den analogen Wert (zwischen 0 und 1023)

    // Den gelesenen Wert auf der seriellen Konsole ausgeben
    Serial.print("Potentiometerwert: ");
    Serial.println(potValue);
    
    lcd.setCursor(0, 0);
    lcd.print("Modus: Manual");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    servoAngle = map(potValue, 0, 1023, 0, 90);
    myservo.write(servoAngle);  // Drehe den Servo auf den entsprechenden Winkel
    delay(15);                  // Kurze Pause für die Servo-Bewegung

  // Optional: den Potentiometerwert und den Servo-Winkel anzeigen
  Serial.print("Potentiometer Wert: ");
  Serial.print(potValue);
  Serial.print(" -> Servo Winkel: ");
  Serial.println(servoAngle);
  }

  //MODUS AUTO
  if ((digitalRead(buttonMode) == LOW && mode == "manual") || (mode == "auto")) {
    mode = "auto";
    potValue = analogRead(potPin);
    lightIntesity = analogRead(lightPin);
    targetIntesity = map(potValue, 0, 1023, 0, 90);
    lcd.setCursor(0, 0);
    lcd.print("Modus: Auto  ");
    lcd.setCursor(0, 1);
    lcd.print("    ");
    lcd.setCursor(0, 1);
    lcd.print("Ziel:");
    lcd.setCursor(6, 1);
    lcd.print("   ");
    lcd.setCursor(6, 1);
    lcd.print(targetIntesity);

    //calculate angle of motor
    new_angle = berechneWinkel(targetIntesity, lightIntesity);
    double difference = fabs(old_angle - new_angle);
    Serial.println("alte Winkel ist:");
    Serial.println(old_angle);
    Serial.println("neue Winkel ist:");
    Serial.println(new_angle);
    Serial.println("Unterschied ist:");
    Serial.println(difference);
    step = richtige_richtung(new_angle, old_angle);
    //wann den licht heller 
    if ((difference > 10) && (step == 1)) {
      Serial.println("Das Licht ist heller");
      delay(2000);
      for (pos = old_angle; pos <= new_angle; pos += step) {
        Serial.println("step ist:");
        Serial.println(step);
        Serial.println("position ist:");
        Serial.println(pos);
        myservo.write(pos);
        delay(15);
      }
      old_angle = new_angle;
    }
    //wann das Licht dünkler ist 
    if ((difference > 10) && (step == -1)) {
      Serial.println("Das Licht ist dünkler");
      delay(2000);
      for (pos = old_angle; pos >= new_angle; pos += step) {
        Serial.println("step ist:");
        Serial.println(step);
        Serial.println("position ist:");
        Serial.println(pos);
        myservo.write(pos);
        delay(15);
      }
      old_angle = new_angle;
    }

    delay(2000);
  }
  //VON AUTO ZUM MANUAL
  if (digitalRead(buttonMode) == LOW && mode == "auto") {
    lcd.setCursor(0, 0);
    lcd.print("Modus: Manual");
    lcd.setCursor(0, 1);
    lcd.print("                     ");
    mode = "manual";
    delay(2000);
  } 
}

int berechneWinkel(int targetIntesity, int lightIntesity ) {
  if (lightIntesity <= 0) return 0;  // Vollständiges Öffnen bei extrem niedrigem Außenlicht
        Serial.println("Target Intesity:");
        Serial.println(targetIntesity);
        Serial.println("Light Intesity:");
        Serial.println(lightIntesity);  
  // Berechne den Servo-Winkel basierend auf der gewünschten und aktuellen Lichtintensität
  int theta = 90 - (targetIntesity * 90) / lightIntesity;
  
  // Begrenze den Winkel auf [0, 90]
  if (theta > 90) theta = 90;
  if (theta < 0) theta = 0;
  
  return theta;
}
// in welche richtung drehen?
int richtige_richtung(double new_angle, double old_angle) {
  if (new_angle > old_angle) {
    return 1;
  } else {
    return -1;
  }
}
