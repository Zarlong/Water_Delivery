#define WIFI_NAME "MyRobot"
#define WIFI_PASSWORD "7771111777"

#define DEVICE_NAME "ZAR"

#define SERVER_IP "192.168.5.115"
#define SERVER_PORT 1234

#define ENABLE_AVOCADO_ESP_WIFI_MONITOR
#define ENABLE_AVOCADO_ESP_WIFI_BOOT

#include <AVOCADO_esp.h>

AVOCADO_esp esp;

float eold = 0;

int c = 0;
int mils = 0;


#define FRAME_HEADER 0XAA       // определяем заголовок пакета
#define DATA_HEADER 0xAD        // определяем значение байта начала данных
#define SIZE_HI_BYTE 1          // определяем место в пакете старшего байта размера данных без checksum
#define SIZE_LO_BYTE 2          // определяем место в пакете младшего байта размера данных без checksum
#define DATA_START_BYTE 5       // определяем место в пакете байта начала полезных данных
#define DISTANCE_DATA_SYZE 7    // определяем место в пакете размера данных расстояния
#define START_ANGLE_HI_BYTE 11  // определяем место старшего байта стартового угла пакета
#define START_ANGLE_LO_BYTE 12  // определяем место младшего байта стартового угла пакета
#define FIRST_DIST_HI_BYTE 14   // определяем место старшего байта первой пробы расстояния в пакете
#define FIRST_DIST_LO_BYTE 15   // определяем место младшего байта первой пробы расстояния в пакете
// конец datasheet

float lidar_data[360];  // массив для хранения расстояний

uint8_t raw_data[256];    // буфер для хранения "сырых" данных
bool data_ready = false;  // флаг готовности данных
long lidar_delay = 1000;  // интервал вывода в серийный порт
long current_millis = 0;  // текущие millis() для сравнения

#define PWML 25
#define CH1L 32
#define CH2L 33

#define PWMR 14
#define CH1R 26
#define CH2R 27

bool milflag = 0;

#define PWMACT 21
#define ACT1 23
#define ACT2 22

int mode = 0;
bool f = false;
// int c;

void setup() {

  Serial.begin(115200);      // инициируем работу с нулевым серийным портом для вывода в серийный монитор
  Serial2.begin(230400);     // инициируем работу с первым серийным портом для чтения данных лидар
  memset(raw_data, 0, 256);  // обнуляем буфер

  pinMode(PWML, OUTPUT);
  pinMode(PWMR, OUTPUT);

  pinMode(CH1L, OUTPUT);
  pinMode(CH2L, OUTPUT);
  pinMode(CH1R, OUTPUT);
  pinMode(CH2R, OUTPUT);

  pinMode(ACT1, OUTPUT);
  pinMode(ACT2, OUTPUT);

  pinMode(PWMACT, OUTPUT);
  // pinMode(CH2R, OUTPUT);

  esp.setup();
}

void loop() {
  esp.update();
  digitalWrite(CH1L, LOW);
  digitalWrite(CH2L, HIGH);

  digitalWrite(CH1R, LOW);
  digitalWrite(CH2R, HIGH);

  if (esp.available()) {
    String input_data = esp.read();
    if (input_data == "0") {
      mode = 0;
      analogWrite(PWMACT, 0);
    }
    if (input_data == "1") {
      mode = 1;
      analogWrite(PWMACT, 0);
      c = millis();
    }
    if (input_data == "2") {
      mode = 2;
      analogWrite(PWMACT, 0);
    }
    if (input_data == "3") {
      mode = 3;
    }
    if (input_data == "4") {
      mode = 4;
    }
  }
  if (mode == 4) {
    digitalWrite(ACT1, LOW);
    digitalWrite(ACT2, HIGH);

    // digitalWrite(CH1R, HIGH);
    // digitalWrite(CH2R, LOW);

    analogWrite(PWMACT, 200);
    esp.print("ACT");
    esp.update();
  }
  if (mode == 3) {
    digitalWrite(ACT1, HIGH);
    digitalWrite(ACT2, LOW);

    // digitalWrite(CH1R, HIGH);
    // digitalWrite(CH2R, LOW);

    analogWrite(PWMACT, 200);
    esp.print("ACT");
    esp.update();
  }

  if (mode == 2) {
    digitalWrite(CH1L, HIGH);
    digitalWrite(CH2L, LOW);

    digitalWrite(CH1R, HIGH);
    digitalWrite(CH2R, LOW);

    analogWrite(PWML, 70);
    analogWrite(PWMR, 70);
  }

  for (int i = 170; i < 190; ++i) {
    if (lidar_data[i] < 400 and lidar_data[i] != 0) {
      // esp.print(String(lidar_data[i]) + " " + String(i));
      f = true;
      mils = millis();
      // esp.print("1");
      // c = millis();
      // k = i;
      while (lidar_data[i] < 400) {
        analogWrite(PWML, 0);
        analogWrite(PWMR, 0);
        esp.print(String(lidar_data[i]) + " " + String(i));
        esp.update();

        while (Serial2.available()) {

          // читаем стартовый байт пакета
          if (Serial2.read() == (raw_data[0] = FRAME_HEADER)) {
            for (int i = 1; i != 256; i++) {

              // заполняем буфер
              raw_data[i] = Serial2.read();
            }
          }
          esp.update();
        }

        // если байт указывающий на данные совпадает с заголовком пакета
        if (raw_data[DATA_START_BYTE] == DATA_HEADER) {

          // берем длинну полезных данных (2-й и 3-й байт сырых данных)
          uint16_t raw_data_length = (raw_data[SIZE_HI_BYTE] << 8) + raw_data[SIZE_LO_BYTE];

          // берем контрольную сумму из сырых данных
          uint16_t checksum = (raw_data[raw_data_length] << 8) + raw_data[raw_data_length + 1];
          delay(5);

          // вызываем функцию подсчета контрольной суммы
          if (checksum == checksum_cmp(raw_data, raw_data_length)) {  // если сумма прошла

            // подсчет стартового угла данного пакета
            float start_angle = ((raw_data[START_ANGLE_HI_BYTE] << 8)
                                 + raw_data[START_ANGLE_LO_BYTE])
                                * 0.01;

            // подсчет кол-ва проб расстояния: кол-во проб = (размер данных расстояния - 5) /  3
            uint8_t read_count = (raw_data[DISTANCE_DATA_SYZE] - 5) / 3;
            for (int n = 0; n < read_count; n++) {

              // подсчет угла каждой пробы:
              // угол = стартовый угол + 22,5° * (номер пробы - 1)/ кол-во проб
              float angle = start_angle + 22.5 * (n - 1) / read_count;

              // итератор для массива lidar_data
              int i = int(angle);

              // подсчет расстояния
              float distance = ((raw_data[FIRST_DIST_HI_BYTE + n * 3] << 8)
                                + raw_data[FIRST_DIST_LO_BYTE + n * 3])
                               * 0.25;

              // запись расстояния в массив
              lidar_data[i] = distance;
            }
          }
          data_ready = true;
        }
      }
      mils = millis() - mils;
    } else {
      // if (millis() > c + 500) {
      f = false;
      // esp.print("0");
      // }
    }
    // esp.update();
  }
  if (f == true) {
    if (mode == 1) {
      // int c = millis();
      // while (millis() < c + 2000) {
      analogWrite(PWML, 0);
      analogWrite(PWMR, 0);
      // esp.update();
      // }
    }
  } else {
    if (mils + millis() < c + 8000 + mils) {
      go();
      milflag = 1;
    } else {
      if (milflag == 1) {
        analogWrite(PWML, 0);
        analogWrite(PWMR, 0);
        milflag = 2;
      }
    }
    // esp.update();
  }
  // if (milflag == 2) {
  //   digitalWrite(CH1L, HIGH);
  //   digitalWrite(CH2L, LOW);

  //   digitalWrite(CH1R, LOW);
  //   digitalWrite(CH2R, HIGH);

  //   analogWrite(PWML, 70);
  //   analogWrite(PWMR, 70);
    
  //   delay(2000);

  //   digitalWrite(CH1L, HIGH);
  //   digitalWrite(CH2L, LOW);

  //   digitalWrite(CH1R, HIGH);
  //   digitalWrite(CH2R, LOW);

  //   analogWrite(PWML, 70);
  //   analogWrite(PWMR, 70);
    
  //   delay(3000);
    
  //   analogWrite(PWML, 0);
  //   analogWrite(PWMR, 0);
  // }

  if (mode == 0) {
    analogWrite(PWML, 0);
    analogWrite(PWMR, 0);
    // esp.update();
  }
  // for (int i = 268; i < 272; ++i) {
  //   if (lidar_data[i] != 0) {
  //     float p = 70 - lidar_data[i];

  //     float PID = p * 0.01;

  //     esp.print(String(PID) + " " + String(p) + " " + String(lidar_data[i]));
  //   }
  // }

  // если прошёл интервал ожидания и данные готовы


  // пока есть данные для чтения из серйиного порта №1
  while (Serial2.available()) {

    // читаем стартовый байт пакета
    if (Serial2.read() == (raw_data[0] = FRAME_HEADER)) {
      for (int i = 1; i != 256; i++) {

        // заполняем буфер
        raw_data[i] = Serial2.read();
      }
    }
    esp.update();
  }

  // если байт указывающий на данные совпадает с заголовком пакета
  if (raw_data[DATA_START_BYTE] == DATA_HEADER) {

    // берем длинну полезных данных (2-й и 3-й байт сырых данных)
    uint16_t raw_data_length = (raw_data[SIZE_HI_BYTE] << 8) + raw_data[SIZE_LO_BYTE];

    // берем контрольную сумму из сырых данных
    uint16_t checksum = (raw_data[raw_data_length] << 8) + raw_data[raw_data_length + 1];
    delay(5);

    // вызываем функцию подсчета контрольной суммы
    if (checksum == checksum_cmp(raw_data, raw_data_length)) {  // если сумма прошла

      // подсчет стартового угла данного пакета
      float start_angle = ((raw_data[START_ANGLE_HI_BYTE] << 8)
                           + raw_data[START_ANGLE_LO_BYTE])
                          * 0.01;

      // подсчет кол-ва проб расстояния: кол-во проб = (размер данных расстояния - 5) /  3
      uint8_t read_count = (raw_data[DISTANCE_DATA_SYZE] - 5) / 3;
      for (int n = 0; n < read_count; n++) {

        // подсчет угла каждой пробы:
        // угол = стартовый угол + 22,5° * (номер пробы - 1)/ кол-во проб
        float angle = start_angle + 22.5 * (n - 1) / read_count;

        // итератор для массива lidar_data
        int i = int(angle);

        // подсчет расстояния
        float distance = ((raw_data[FIRST_DIST_HI_BYTE + n * 3] << 8)
                          + raw_data[FIRST_DIST_LO_BYTE + n * 3])
                         * 0.25;

        // запись расстояния в массив
        lidar_data[i] = distance;
      }
    }
    data_ready = true;
  }
}

// функция вычисления контрольной суммы.
uint16_t checksum_cmp(uint8_t *raw_data, uint16_t raw_data_length) {
  uint16_t _checksum = 0;
  while (raw_data_length--) {
    // складываем все байты пакета
    _checksum += *raw_data++;
  }
  return _checksum;
}