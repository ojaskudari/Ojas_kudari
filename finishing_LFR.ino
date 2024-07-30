int ir1 = 12;  // BOT-LEFT
int ir2 = 11;  // BOT-MIDDLE
int ir3 = 10;  // BOT-RIGHT
int m11 = 4;
int m12 = 5;
int m21 = 6;
int m22 = 7;
int m1e = 3;
int m2e = 9;
int echo = 8;
int trig = 13;
int order_A = 0;
int order_B = 0;
int order_C = 0;
int redPin = 1;
int bluePin = 3;
int greenPin = 2;
int buzzerPin=14;

int nodeCount = 0;
int currentNode = 0;

void setup() {
  // Initialize the IR sensor pins as input
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(m11, OUTPUT);
  pinMode(m12, OUTPUT);
  pinMode(m21, OUTPUT);
  pinMode(m22, OUTPUT);
  pinMode(m1e, OUTPUT);
  pinMode(m2e, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  // Read the state of the IR sensors
  if (digitalRead(ir1) && digitalRead(ir2) && digitalRead(ir3)) {
    nodeCount++;
    currentNode = nodeCount;
   
    switch(currentNode) {
      case 0:
        stop();
        forward();
        break;

      case 1:
        stop();
        left();
        forward();
        break;

      case 2:
        stop();
        right();
        forward();
        break;
      
      case 3:
      case 4:
      case 5:
      case 7:
      case 8:
      case 9:
      case 11:
      case 12:
      case 13:
      case 14:
      case 15:
      case 16:
      case 17:
      case 18:
      case 19:
      case 20:
      case 21:
      case 22:
      case 23:
      case 24:
      case 25:
        stop();
        blast_US();
        //calculate_distance();
        classify_order();
        forward();
        break;

      case 6:
        stop();
        right();
        forward();
        break;

      case 10:
        stop();
        right();
        forward();
        break;

      default:
        stop();
        forward();
        stop();
        break;
    }
  }
}

void classify_order() {
  float distance = blast_US();
  if (distance > 0 && distance <= 3) {
    order_A++;
    digitalWrite(redPin, HIGH);
    delay(500); // Assuming some delay is needed
    digitalWrite(redPin, LOW);
  } else if (distance > 3 && distance <= 6) {
    order_B++;
    digitalWrite(greenPin, HIGH);
    delay(500); // Assuming some delay is needed
    digitalWrite(greenPin, LOW);
  } else if (distance > 6) {
    order_C++;
    digitalWrite(bluePin, HIGH);
    delay(500); // Assuming some delay is needed
    digitalWrite(bluePin, LOW);
  } else {
    digitalWrite(buzzerPin, HIGH); // Assuming a buzzer pin is declared somewhere
    delay(500);
    digitalWrite(buzzerPin, LOW);
  }
}

long blast_US() {
  long duration, fro;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  fro = (long)duration / 2 / 29.1;
  Serial.print("Distance:");
  Serial.println(fro);
  return fro;
}

void stop() {
  digitalWrite(m11, LOW);
  digitalWrite(m12, LOW);
  digitalWrite(m21, LOW);
  digitalWrite(m22, LOW);
}

void forward() {
  analogWrite(m1e, 140);
  analogWrite(m2e, 140);
  digitalWrite(m11, HIGH);
  digitalWrite(m12, LOW);
  digitalWrite(m21, HIGH);
  digitalWrite(m22, LOW);
}

void right() {
  analogWrite(m1e, 25);
  analogWrite(m2e, 170);
  digitalWrite(m11, HIGH);
  digitalWrite(m12, LOW);
  digitalWrite(m21, HIGH);
  digitalWrite(m22, LOW);
}

void left() {
  analogWrite(m2e, 25);
  analogWrite(m1e, 170);
  digitalWrite(m11, HIGH);
  digitalWrite(m12, LOW);
  digitalWrite(m21, HIGH);
  digitalWrite(m22, LOW);
}

void convey_order(int pin, int num) {
  for (int i = 0; i < num; i++) {
    digitalWrite(pin, HIGH);
    digitalWrite(buzzerPin, HIGH); // Assuming a buzzer pin is declared somewhere
    delay(500);
    digitalWrite(pin, LOW);
    digitalWrite(buzzerPin, LOW);
    delay(500);
  }
}
