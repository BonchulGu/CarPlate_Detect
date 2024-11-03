#define TRIG_PIN 4     // 초음파 센서 트리거 핀
#define ECHO_PIN 9     // 초음파 센서 에코 핀

// 왼쪽 모터 제어 핀
#define LPWM 6         // 왼쪽 모터 PWM 핀 (역방향)
#define RPWM 5         // 왼쪽 모터 PWM 핀 (정방향)
#define L_EN 7         // 왼쪽 모터 활성화 핀
#define R_EN 8         // 왼쪽 모터 활성화 핀

// 오른쪽 모터 제어 핀
#define RPWM_Right 10  // 오른쪽 모터 PWM 핀 (정방향)
#define LPWM_Right 11  // 오른쪽 모터 PWM 핀 (역방향)
#define L_EN_Right 12  // 오른쪽 모터 활성화 핀
#define R_EN_Right 13  // 오른쪽 모터 활성화 핀

// 속도 변수
int normalSpeed = 255;  // 최대 속도
int slowSpeed = 150;    // 느린 속도 (서서히 정지)

void setup() {
  Serial.begin(9600);

  // 왼쪽 모터 핀 설정
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  digitalWrite(L_EN, LOW);
  digitalWrite(R_EN, LOW);

  // 오른쪽 모터 핀 설정
  pinMode(LPWM_Right, OUTPUT);
  pinMode(RPWM_Right, OUTPUT);
  pinMode(L_EN_Right, OUTPUT);
  pinMode(R_EN_Right, OUTPUT);
  digitalWrite(L_EN_Right, LOW);
  digitalWrite(R_EN_Right, LOW);

  // 초음파 센서 핀 설정
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // 초음파 센서 거리 측정 및 전송
  float distance = getDistance();
  Serial.println(distance);
  delay(500); // 0.5초마다 거리 데이터 전송

  // 시리얼로부터 명령 수신
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'F') {
      forward(normalSpeed);  // 정방향 최대 속도
    }
    else if (command == 'B') {
      backward(normalSpeed); // 역방향 최대 속도
    }
    else if (command == 'L') {
      backward(slowSpeed);    // 서서히 정지 (감속)
    }
    else if (command == 'S') {
      stopMotor();           // 모터 정지
    }
  }
}

// 초음파 센서로 거리 측정 함수
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2; // cm 단위 거리 계산
  return distance;
}

// 모터 정방향 제어 함수
void backward(int speed) {
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN_Right, HIGH);
  digitalWrite(R_EN_Right, HIGH);

  // 왼쪽 모터 정방향 회전
  analogWrite(RPWM, speed);  
  analogWrite(LPWM, 0);

  // 오른쪽 모터 정방향 회전 (반대 방향)
  analogWrite(LPWM_Right, speed);  
  analogWrite(RPWM_Right, 0);
}

// 모터 역방향 제어 함수
void forward(int speed) {
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN_Right, HIGH);
  digitalWrite(R_EN_Right, HIGH);

  // 왼쪽 모터 역방향 회전
  analogWrite(LPWM, speed);  
  analogWrite(RPWM, 0);

  // 오른쪽 모터 역방향 회전 (반대 방향)
  analogWrite(RPWM_Right, speed);  
  analogWrite(LPWM_Right, 0);
}

// 모터 정지 함수
void stopMotor() {
  digitalWrite(L_EN, LOW);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN_Right, LOW);
  digitalWrite(R_EN_Right, LOW);

  // 모터 속도 제어 핀을 0으로 설정하여 정지
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  analogWrite(RPWM_Right, 0);
  analogWrite(LPWM_Right, 0);
}
