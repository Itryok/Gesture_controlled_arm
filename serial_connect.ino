#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
  Serial.begin(9600); // Must match the baud rate in Python code
  servo1.attach(8);   // Shoulder
  servo2.attach(9);  // Elbow
  servo3.attach(10);  // Wrist
}

void loop() {
  if (Serial.available() > 0) {
    String anglesString = Serial.readStringUntil('\n'); // Read the angles sent by Python
    anglesString.trim(); // Remove any leading/trailing whitespace
    int index = 0;
    String angleStr;
    int angles[4];

    // Parse the angles
    for (int i = 0; i < 3; i++) {
      int commaIndex = anglesString.indexOf(',', index);
      if (commaIndex == -1) {
        angleStr = anglesString.substring(index);
      } else {
        angleStr = anglesString.substring(index, commaIndex);
      }
      angles[i] = angleStr.toInt();
      index = commaIndex + 1;
    }

    // Set servo angles
    servo1.write(angles[0]);
    servo2.write(angles[1]);
    servo3.write(angles[2]);
    
    // Print received angles
    Serial.print("Received angles: ");
    Serial.print(angles[0]);
    Serial.print(", ");
    Serial.print(angles[1]);
    Serial.print(", ");
    Serial.print(angles[2]);
    Serial.print(", ");
    Serial.println(angles[3]);
    delay(15);
  }
}