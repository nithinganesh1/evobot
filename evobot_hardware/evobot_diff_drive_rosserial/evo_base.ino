const int DIR_A = 4;
const int PWM_A = 5;

const int DIR_B = 7;
const int PWM_B = 6;

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);

  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_A, OUTPUT);

  pinMode(DIR_B, OUTPUT);
  pinMode(PWM_B, OUTPUT);

  inputString.reserve(20);
}

void loop() {
  // Read serial input
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    if (inputString == "S") {
      analogWrite(PWM_A, 0);
      analogWrite(PWM_B, 0);
    }
    else if (inputString.startsWith("L") && inputString.indexOf("R") > 0) {
      int rIndex = inputString.indexOf("R");
      float leftVal = inputString.substring(1, rIndex).toFloat();
      float rightVal = inputString.substring(rIndex + 1).toFloat();

      setMotor(PWM_A, DIR_A, leftVal);
      setMotor(PWM_B, DIR_B, rightVal);
    }

    inputString = "";
    stringComplete = false;
  }
}

void setMotor(int pwmPin, int dirPin, float value) {
  // Deadband for small values
  if (value > -0.05 && value < 0.05) {
    analogWrite(pwmPin, 0);
    return;
  }

  int pwmOut = (int)(fabs(value) * 255.0);

  if (value < 0) {
    // Reverse
    digitalWrite(dirPin, LOW);
  } else {
    // Forward
    digitalWrite(dirPin, HIGH);
  }

  analogWrite(pwmPin, pwmOut);
}
