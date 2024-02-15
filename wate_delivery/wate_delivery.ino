#define MOTOR_MOSFET_PIN      19    // определяем вывод для управления ключом мотора
// из datasheet:
#define FRAME_HEADER          0XAA  // определяем заголовок пакета
#define DATA_HEADER           0xAD  // определяем значение байта начала данных
#define SIZE_HI_BYTE          1     // определяем место в пакете старшего байта размера данных без checksum
#define SIZE_LO_BYTE          2     // определяем место в пакете младшего байта размера данных без checksum
#define DATA_START_BYTE       5     // определяем место в пакете байта начала полезных данных
#define DISTANCE_DATA_SYZE    7     // определяем место в пакете размера данных расстояния
#define START_ANGLE_HI_BYTE   11    // определяем место старшего байта стартового угла пакета
#define START_ANGLE_LO_BYTE   12    // определяем место младшего байта стартового угла пакета
#define FIRST_DIST_HI_BYTE    14    // определяем место старшего байта первой пробы расстояния в пакете
#define FIRST_DIST_LO_BYTE    15    // определяем место младшего байта первой пробы расстояния в пакете
// конец datasheet

float lidar_data[360];              // массив для хранения расстояний

uint8_t raw_data[256];              // буфер для хранения "сырых" данных
bool data_ready = false;            // флаг готовности данных
long lidar_delay = 1000;            // интервал вывода в серийный порт
long current_millis = 0;            // текущие millis() для сравнения

void setup() {

    Serial.begin(115200);           // инициируем работу с нулевым серийным портом для вывода в серийный монитор
    Serial1.begin(230400);          // инициируем работу с первым серийным портом для чтения данных лидара
    pinMode(
            MOTOR_MOSFET_PIN,
            OUTPUT
            );                      // устанавливаем вывод ключа мотора в режим выход
    digitalWrite(
            MOTOR_MOSFET_PIN,
            HIGH);                  // устанавливаем вывод ключа мотора в режим логической единицы
    memset(raw_data, 0, 256);       // обнуляем буфер
}

void loop() {

    // если прошёл интервал ожидания и данные готовы
    if(millis() - current_millis > lidar_delay && data_ready) {
        for(int i = 0; i < 360; i++) {

            // выводим текст в серийный монитор
            Serial.print("Angle: ");

            // добавляем пробел (для красоты вывода)
            // если угол меньше 10
            if(i < 10){
                Serial.print(" ");
            }
            // выводим индекс массива
            Serial.print(i);

            // выводим данные в серийный монитор
            Serial.print("\tDistance: ");
            Serial.println(lidar_data[i]);

            // записываем текущие millis
            current_millis = millis();

            // обнуляем флаг готовности данных
            data_ready = false;
        }
    }

    // пока есть данные для чтения из серйиного порта №1
    while(Serial1.available()) {

        // читаем стартовый байт пакета
        if (Serial1.read() == (raw_data[0] = FRAME_HEADER)){
            for (int i = 1; i!= 256; i++){

                // заполняем буфер
                raw_data[i] = Serial1.read();
            }
        }
    }

    // если байт указывающий на данные совпадает с заголовком пакета
    if(raw_data[DATA_START_BYTE] == DATA_HEADER) {

        // берем длинну полезных данных (2-й и 3-й байт сырых данных)
        uint16_t raw_data_length = (raw_data[SIZE_HI_BYTE] << 8) + raw_data[SIZE_LO_BYTE];

        // берем контрольную сумму из сырых данных
        uint16_t checksum = (raw_data[raw_data_length] << 8) + raw_data[raw_data_length+1];
        delay(10);

        // вызываем функцию подсчета контрольной суммы
        if (checksum == checksum_cmp(raw_data, raw_data_length)) { // если сумма прошла

            // подсчет стартового угла данного пакета
            float start_angle = ((raw_data[START_ANGLE_HI_BYTE] << 8)
                                    + raw_data[START_ANGLE_LO_BYTE])*0.01;

            // подсчет кол-ва проб расстояния: кол-во проб = (размер данных расстояния - 5) /  3
            uint8_t read_count = (raw_data[DISTANCE_DATA_SYZE] - 5) /  3;
            for (int n = 0; n < read_count; n++){

                // подсчет угла каждой пробы:
                // угол = стартовый угол + 22,5° * (номер пробы - 1)/ кол-во проб
                float angle = start_angle + 22.5 * (n - 1) / read_count;

                // итератор для массива lidar_data
                int i = int(angle);

                // подсчет расстояния
                float distance = ((raw_data[FIRST_DIST_HI_BYTE+n*3] << 8)
                                    + raw_data[FIRST_DIST_LO_BYTE+n*3]) * 0.25;

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