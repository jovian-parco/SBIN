// Define the pins
const int buzzerPin = 4; // Buzzer is connected to pin 4
const int sensorPin = A0; // Water sensor is connected to analog pin A0

// Define the threshold value
const int threshold = 500; // Change this value according to your sensor

void setup() {
  // Initialize the serial monitor
  Serial.begin(9600);
  
  // Set the buzzer pin as output
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  // Read the sensor value
  int sensorValue = analogRead(sensorPin);

  Serial.println(sensorValue);
  
  // Check if the sensor value is above the threshold
  if (sensorValue < threshold) {
    // Turn on the buzzer
    digitalWrite(buzzerPin, LOW);
    
  }
  else {
    // Turn off the buzzer
    digitalWrite(buzzerPin, LOW);
  }
  
  // Wait for a short time
  delay(100);
}