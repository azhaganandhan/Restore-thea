const String ver = "3.1.23";


//================================= config start

String dev_N = "7"; //set dev nom

//the choice of module type
#define COMM_MODULE 3 // 1-GPRS, 2-WiFi, 3-LoRa
//if GPRS is enabled
#define GPRS_OPER 1  //1-BSNL, 2-VODAFONE
bool flgSchedule = false;  //allows schedule for GSM cards, включает расписание для GSM плат
uint8_t setHour = 15;  //setting the time when statistics are sent, установка времени отправки статистики за день по расписанию
uint8_t setMin = 1;


//if LoRa is enabled
uint8_t masCH[] = {3,4};

bool sendEnergyInfo = false; //true - allows the sending of data of voltage and current, etc., тру - включает отправку электрических данных


bool SerialDebug = true;  //allows debugging to the serial port, включение отладки в серийный порт
bool SaveLog = false; //allows log entries to the SD card, вкючение записи лога на сдКарту


const uint16_t delayBetSend = 20; //delay ms between sending packets from send.txt, задержка между отправкой пакетов из send.txt



const float kofVps = 2.62;  //correction factor for measuring the Board's power supply voltage, поправочный коффициент для измерения напряжения питания платы

const  uint8_t K2_V1 = 16; //correction factor for phase 1 voltage
const  uint8_t K2_V2 = 19; //correction factor for phase 2 voltage
const  uint8_t K2_V3 = 14; //correction factor for phase 3 voltage
 
//================================= config end
