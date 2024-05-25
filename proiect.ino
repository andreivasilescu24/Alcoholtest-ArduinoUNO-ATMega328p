#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define AlcoholSensorAnalog A0
#define AlcoholSensorDigital 7
#define Buzzer 8
#define GreenLed 3
#define RedLed 4
#define LCD_I2C_Address 0x27
#define Button 2

LiquidCrystal_I2C lcd(LCD_I2C_Address, 16, 2);

volatile bool measureAlcohol = false;
volatile bool startMeasurement = false;
volatile bool stopMeasurement = false;

void startPrint() {
  lcd.setCursor(0, 0);
  lcd.setCursor(2, 0);
  lcd.print("ALCOHOLTEST");
  delay(4000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press button to");
  lcd.setCursor(0, 1);
  lcd.print("start!");
}

void setup() {
  // Baud rate
  Serial.begin(9600);
  Serial.println("Alcohol sensor warming up!");

  // Output pins
  DDRD |= (1 << DDD3);  // GreenLed
  DDRD |= (1 << DDD4);  // RedLed
  DDRB |= (1 << DDB0);  // Buzzer

  // Input pins
  DDRD &= ~(1 << DDD7); // AlcoholSensor
  DDRD &= ~(1 << DDD2); // Button

  // Pull-up resistor on button
  PORTD |= (1 << PORTD2);

  // LCD initalization
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Warming up...");
  delay(10000);
  Serial.println("Alcohol sensor warmed up!");

  lcd.clear();
  lcd.setCursor(0, 0);
  startPrint();

  // Interrupt on button
  attachInterrupt(digitalPinToInterrupt(Button), buttonISR, FALLING);
}

void buttonISR() {
  measureAlcohol = !measureAlcohol;
  if (measureAlcohol) {
    startMeasurement = true;
  } else {
    stopMeasurement = true;
  }
}

void loop() {
  if (startMeasurement) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Starting");
    lcd.setCursor(0, 1);
    lcd.print("measurement...");
    delay(2000);
    lcd.clear();
    delay(100);
    startMeasurement = false;
  } else if (stopMeasurement) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Stopping");
    lcd.setCursor(0, 1);
    lcd.print("measurement...");
    PORTD &= ~(1 << PORTD4); // Turn off RedLed
    PORTD &= ~(1 << PORTD3); // Turn off GreenLed
    delay(10000);
    lcd.clear();
    startPrint();
    stopMeasurement = false;
  }

  if (measureAlcohol) {
    bool digitalAlcohol = PIND & (1 << PIND7); // Read digital value
    float alcoholValue = analogRead(AlcoholSensorAnalog);

    Serial.print("Analog value: ");
    Serial.print(alcoholValue);
    Serial.print("\t");
    Serial.print("Digital value: ");
    Serial.println(digitalAlcohol);

    lcd.clear();
    lcd.setCursor(0, 0);
    float ppm = alcoholValue * (5.0 / 1023.0) * 50;
    lcd.print(ppm);
    lcd.print(" ppm");

    if (digitalAlcohol) {
      lcd.setCursor(0, 1);
      lcd.print("No alcohol");
      PORTD |= (1 << PORTD3);  // Turn on GreenLed
      PORTD &= ~(1 << PORTD4); // Turn off RedLed
      PORTB |= (1 << PORTB0);  // Turn off Buzzer
    } else {
      lcd.setCursor(0, 1);
      lcd.print("Alcohol detected");
      PORTB &= ~(1 << PORTB0); // Turn on Buzzer
      PORTD &= ~(1 << PORTD3); // Turn off GreenLed
      PORTD |= (1 << PORTD4);  // Turn on RedLed
      delay(100);
      PORTB |= (1 << PORTB0);  // Turn off Buzzer
      PORTD |= (1 << PORTD3);  // Turn on GreenLed
      PORTD &= ~(1 << PORTD4); // Turn off RedLed
    }
  }

  delay(100);
}
