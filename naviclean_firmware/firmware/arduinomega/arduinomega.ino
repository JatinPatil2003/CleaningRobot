double num = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the built-in LED pin as output
  Serial.begin(115200);  // Start the serial communication at 115200 baud rate
  delay(2000);  // Give a small delay for serial initialization
}

void loop() {
  // Check if there's any data available in the serial buffer
  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n');  // Read until newline character

    int commaIndex = receivedData.indexOf(',');  // Find the position of the comma

    // Ensure commaIndex is valid before proceeding
    if (commaIndex != -1) {
      String left = receivedData.substring(0, commaIndex);  // Extract left part
      String right = receivedData.substring(commaIndex + 1);  // Extract right part

      float var1 = left.toFloat();  // Convert left part to float
      float var2 = right.toFloat();  // Convert right part to float

      // Check if either of the variables is zero and control the LED accordingly
      if (var1 == 0.0 || var2 == 0.0) {
        digitalWrite(LED_BUILTIN, LOW);  // Turn on LED if either value is zero
      } else {
        digitalWrite(LED_BUILTIN, HIGH);  // Turn off LED if both values are non-zero
      }
    }
//    String dataToSend = String(data) + "," + String(data);  // Concatenate two data values with a comma
//    Serial.println(dataToSend);  // Send the data over serial
//    data++;  // Increment the data
//  
//    // Prevent overflow
//    if (data > 10000) {
//      data = 0;  // Reset the data if it exceeds a certain limit
//    }
  }

  // Create a data string to send back, no base formatting needed
  String dataToSend = String(num) + "," + String(num);  // Concatenate two data values with a comma
  Serial.println(dataToSend);  // Send the data over serial
  num += 0.01;  // Increment the data

  // Prevent overflow
  if (num > 10000) {
    num = 0;  // Reset the data if it exceeds a certain limit
  }
  delay(100);
}
