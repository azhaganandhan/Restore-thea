#include "Arduino.h"
#include "config.h"
#include "uTimerLib.h"
#include <stdint.h>
#include <ctype.h>
#include <watchdog.h>
#include <SPI.h>
#include <SD.h>
#include <DS3231.h>
#include <RH_RF95.h>

#define RFM95_CS  52    // "E"
#define RFM95_RST 18   // "D"
#define RFM95_INT 46   // "B"


#define RF95_FREQ 865.5
//#define RF95_FREQ 434.0
uint8_t    CHNL = 2;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

const uint8_t pinPorts[30] = {23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53, 42, 40, 38, 36, 34, 32, 30, 28, 26, 24, A4, A3, A2, A1};


#if GPRS_OPER == 1
char apn[]  = "internet";
#endif

#if GPRS_OPER == 2
char apn[]  = "www";
#endif

const char user[] = "";
const char pass[] = "";
// MQTT details
uint16_t qty_SendLine = 1;
const char* broker = "electricity.auroville.org.in";


//COMM_MODULE 1-GPRS, 2-WiFi, 3-LoRa
#if COMM_MODULE == 1
const  uint16_t qntStrFile = 30; //how many lines to generate files for sending, по сколько строк формировать файлы для отправки
uint32_t sizeFileVol = 300;   //максимальное кол-во байт в файле
#endif
#if COMM_MODULE == 2
const  uint16_t qntStrFile = 600; //how many lines to generate files for sending, по сколько строк формировать файлы для отправки
uint32_t sizeFileVol = 300;   //максимальное кол-во байт в файле
#endif
#if COMM_MODULE == 3
const  uint16_t qntStrFile = 600; //how many lines to generate files for sending, по сколько строк формировать файлы для отправки
uint32_t sizeFileVol = 70;   //максимальное кол-во байт в файле
#endif



const  uint8_t kK_f1V = 255.0;
const  uint8_t kK_f2V = 255.0;
const  uint8_t kK_f3V = 255.0;
const  uint8_t kK_f1I = 95.0;
const  uint8_t kK_f2I = 95.0;
const  uint8_t kK_f3I = 95.0;



double startDate = 1544011201; //дата запуска системы
DS3231  rtc(SDA, SCL);
Time t;

File sendFile;
File statSendFile;
File configFile;


#define SerialAT Serial1

#if COMM_MODULE == 1
#define TINY_GSM_MODEM_SIM800
#include <PubSubClient.h>
#include <TinyGsmClient.h>
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
#endif



#include "EmonLib.h"                   // Include Emon Library
EnergyMonitor emon1;                   // Create an instance
EnergyMonitor emon2;                   // Create an instance
EnergyMonitor emon3;                   // Create an instance
const uint8_t pin_f1V = A7;
const uint8_t pin_f2V = A6;
const uint8_t pin_f3V = A5;
const uint8_t pin_f1I = A10;
const uint8_t pin_f2I = A9;
const uint8_t pin_f3I = A8;
float f1rP       = 0;        //extract Real Power into variable
float f1V        = 0;             //extract Vrms into Variable
float f1I        = 0;             //extract Irms into Variable
float f2rP       = 0;        //extract Real Power into variable
float f2V        = 0;             //extract Vrms into Variable
float f2I        = 0;             //extract Irms into Variable
float f3rP       = 0;        //extract Real Power into variable
float f3V        = 0;             //extract Vrms into Variable
float f3I        = 0;             //extract Irms into Variable




const int pinRadioGND = 44;
const int pinRele = 2;

const int pinWDT = 22;
bool flgWDT = false;



uint16_t val_L1V = 0;                  //
uint16_t val_L2V = 0;                  //
uint16_t val_L3V = 0;                  //
double val_L1A = 0;                  //
double val_L2A = 0;                  //
double val_L3A = 0;                  //
uint8_t val_L1PF = 0;                  //
uint8_t val_L2PF = 0;                  //
uint8_t val_L3PF = 0;                  //

bool       flg_VAPF_startMesh = false;
uint16_t   tmr_VAPF_startMesh = 0;


int      tmrMSec = 0;
uint8_t  tmrSec = 0;
uint8_t  tmrMin = 0;
volatile int16_t tmrReadPorts = 0;

uint8_t curHour = 0;
uint8_t curMin = 0;
uint8_t curSec = 0;
uint8_t lastHour = 0;
uint8_t lastMin = 0;
uint8_t lastSec = 0;

String oldTS = "";
String newTS = "";
bool flgNoSend = false;


bool stringComplete = false;
char inChar ;
String payloadString = "";

bool sendedTimeOut = false;
bool sendedProcess = false;
bool sendedResult = false;


volatile int16_t pulseP0, pulseP1, pulseP2, pulseP3, pulseP4, pulseP5, pulseP6, pulseP7, pulseP8, pulseP9, pulseP10,
         pulseP11, pulseP12, pulseP13, pulseP14, pulseP15, pulseP16, pulseP17, pulseP18, pulseP19, pulseP20, pulseP21, pulseP22,
         pulseP23, pulseP24, pulseP25, pulseP26, pulseP27, pulseP28, pulseP29;

         
uint16_t attemptSendCnt = 0;
uint16_t completedSendCnt = 0;



volatile bool prevStateP0 = true;
volatile bool prevStateP1 = true;
volatile bool prevStateP2 = true;
volatile bool prevStateP3 = true;
volatile bool prevStateP4 = true;
volatile bool prevStateP5 = true;
volatile bool prevStateP6 = true;
volatile bool prevStateP7 = true;
volatile bool prevStateP8 = true;
volatile bool prevStateP9 = true;
volatile bool prevStateP10 = true;
volatile bool prevStateP11 = true;
volatile bool prevStateP12 = true;
volatile bool prevStateP13 = true;
volatile bool prevStateP14 = true;
volatile bool prevStateP15 = true;
volatile bool prevStateP16 = true;
volatile bool prevStateP17 = true;
volatile bool prevStateP18 = true;
volatile bool prevStateP19 = true;
volatile bool prevStateP20 = true;
volatile bool prevStateP21 = true;
volatile bool prevStateP22 = true;
volatile bool prevStateP23 = true;
volatile bool prevStateP24 = true;
volatile bool prevStateP25 = true;
volatile bool prevStateP26 = true;
volatile bool prevStateP27 = true;
volatile bool prevStateP28 = true;
volatile bool prevStateP29 = true;



volatile bool stateP0 = true;
volatile bool stateP1 = true;
volatile bool stateP2 = true;
volatile bool stateP3 = true;
volatile bool stateP4 = true;
volatile bool stateP5 = true;
volatile bool stateP6 = true;
volatile bool stateP7 = true;
volatile bool stateP8 = true;
volatile bool stateP9 = true;
volatile bool stateP10 = true;
volatile bool stateP11 = true;
volatile bool stateP12 = true;
volatile bool stateP13 = true;
volatile bool stateP14 = true;
volatile bool stateP15 = true;
volatile bool stateP16 = true;
volatile bool stateP17 = true;
volatile bool stateP18 = true;
volatile bool stateP19 = true;
volatile bool stateP20 = true;
volatile bool stateP21 = true;
volatile bool stateP22 = true;
volatile bool stateP23 = true;
volatile bool stateP24 = true;
volatile bool stateP25 = true;
volatile bool stateP26 = true;
volatile bool stateP27 = true;
volatile bool stateP28 = true;
volatile bool stateP29 = true;


String serLogStr = "";

uint32_t  arrayStrArhval = 1;
String fNameSendFile = "sending/send1.txt";
int16_t fNomSendFile = 1;
uint32_t tsToSend = 0;
uint32_t cntTotalStr = 0;
String fullNameSendFile = "";
String dirSendFile = "sending/";
int32_t  arrayStr1val = 0;
bool chkLostFiles = true;


bool flgSendPortsPocket = false;
bool flgLoraLineOK = false;
uint16_t cntLoraLineOK = 0;
bool flgSavePulsePorts = false;
bool flgSendPocket = false;
bool flgUpdateTime = false;
bool sessionDay = true;
uint8_t reconnectGPRScnt = 2;
bool flgSendSostDev = true;
bool flg_nid_reconnectGPRS = false;
uint8_t tmpDel = 3;



void timer_1ms() {

  if ( tmrReadPorts >= 2) {
    stateP0 = digitalRead(pinPorts[0]);
    stateP1 = digitalRead(pinPorts[1]);
    stateP2 = digitalRead(pinPorts[2]);
    stateP3 = digitalRead(pinPorts[3]);
    stateP4 = digitalRead(pinPorts[4]);
    stateP5 = digitalRead(pinPorts[5]);
    stateP6 = digitalRead(pinPorts[6]);
    stateP7 = digitalRead(pinPorts[7]);
    stateP8 = digitalRead(pinPorts[8]);
    stateP9 = digitalRead(pinPorts[9]);
    stateP10 = digitalRead(pinPorts[10]);
    stateP11 = digitalRead(pinPorts[11]);
    stateP12 = digitalRead(pinPorts[12]);
    stateP13 = digitalRead(pinPorts[13]);
    stateP14 = digitalRead(pinPorts[14]);
    stateP15 = digitalRead(pinPorts[15]);
    stateP16 = digitalRead(pinPorts[16]);
    stateP17 = digitalRead(pinPorts[17]);
    stateP18 = digitalRead(pinPorts[18]);
    stateP19 = digitalRead(pinPorts[19]);
    stateP20 = digitalRead(pinPorts[20]);
    stateP21 = digitalRead(pinPorts[21]);
    stateP22 = digitalRead(pinPorts[22]);
    stateP23 = digitalRead(pinPorts[23]);
    stateP24 = digitalRead(pinPorts[24]);
    stateP25 = digitalRead(pinPorts[25]);
    stateP26 = digitalRead(pinPorts[26]);
    stateP27 = digitalRead(pinPorts[27]);
    stateP28 = digitalRead(pinPorts[28]);
    stateP29 = digitalRead(pinPorts[29]);


    if ((stateP0 == false) && (prevStateP0 == true)) {
      pulseP0++;
      flgWDT = true;
    }
    if ((stateP1 == false) && (prevStateP1 == true)) {
      pulseP1++;
      flgWDT = true;
    }
    if ((stateP2 == false) && (prevStateP2 == true)) {
      pulseP2++;
      flgWDT = true;
    }
    if ((stateP3 == false) && (prevStateP3 == true)) {
      pulseP3++;
      flgWDT = true;
    }
    if ((stateP4 == false) && (prevStateP4 == true)) {
      pulseP4++;
      flgWDT = true;
    }
    if ((stateP5 == false) && (prevStateP5 == true)) {
      pulseP5++;
      flgWDT = true;
    }
    if ((stateP6 == false) && (prevStateP6 == true)) {
      pulseP6++;
      flgWDT = true;
    }
    if ((stateP7 == false) && (prevStateP7 == true)) {
      pulseP7++;
      flgWDT = true;
    }
    if ((stateP8 == false) && (prevStateP8 == true)) {
      pulseP8++;
      flgWDT = true;
    }
    if ((stateP9 == false) && (prevStateP9 == true)) {
      pulseP9++;
      flgWDT = true;
    }
    if ((stateP10 == false) && (prevStateP10 == true)) {
      pulseP10++;
      flgWDT = true;
    }
    if ((stateP11 == false) && (prevStateP11 == true)) {
      pulseP11++;
      flgWDT = true;
    }
    if ((stateP12 == false) && (prevStateP12 == true)) {
      pulseP12++;
      flgWDT = true;
    }
    if ((stateP13 == false) && (prevStateP13 == true)) {
      pulseP13++;
      flgWDT = true;
    }
    if ((stateP14 == false) && (prevStateP14 == true)) {
      pulseP14++;
      flgWDT = true;
    }
    if ((stateP15 == false) && (prevStateP15 == true)) {
      pulseP15++;
      flgWDT = true;
    }
    if ((stateP16 == false) && (prevStateP16 == true)) {
      pulseP16++;
      flgWDT = true;
    }
    if ((stateP17 == false) && (prevStateP17 == true)) {
      pulseP17++;
      flgWDT = true;
    }
    if ((stateP18 == false) && (prevStateP18 == true)) {
      pulseP18++;
      flgWDT = true;
    }
    if ((stateP19 == false) && (prevStateP19 == true)) {
      pulseP19++;
      flgWDT = true;
    }
    if ((stateP20 == false) && (prevStateP20 == true)) {
      pulseP20++;
      flgWDT = true;
    }
    if ((stateP21 == false) && (prevStateP21 == true)) {
      pulseP21++;
      flgWDT = true;
    }
    if ((stateP22 == false) && (prevStateP22 == true)) {
      pulseP22++;
      flgWDT = true;
    }
    if ((stateP23 == false) && (prevStateP23 == true)) {
      pulseP23++;
      flgWDT = true;
    }
    if ((stateP24 == false) && (prevStateP24 == true)) {
      pulseP24++;
      flgWDT = true;
    }
    if ((stateP25 == false) && (prevStateP25 == true)) {
      pulseP25++;
      flgWDT = true;
    }
    if ((stateP26 == false) && (prevStateP26 == true)) {
      pulseP26++;
      flgWDT = true;
    }
    if ((stateP27 == false) && (prevStateP27 == true)) {
      pulseP27++;
      flgWDT = true;
    }
    if ((stateP28 == false) && (prevStateP28 == true)) {
      pulseP28++;
      flgWDT = true;
    }
    if ((stateP29 == false) && (prevStateP29 == true)) {
      pulseP29++;
      flgWDT = true;
    }


    prevStateP0 = stateP0;
    prevStateP1 = stateP1;
    prevStateP2 = stateP2;
    prevStateP3 = stateP3;
    prevStateP4 = stateP4;
    prevStateP5 = stateP5;
    prevStateP6 = stateP6;
    prevStateP7 = stateP7;
    prevStateP8 = stateP8;
    prevStateP9 = stateP9;
    prevStateP10 = stateP10;
    prevStateP11 = stateP11;
    prevStateP12 = stateP12;
    prevStateP13 = stateP13;
    prevStateP14 = stateP14;
    prevStateP15 = stateP15;
    prevStateP16 = stateP16;
    prevStateP17 = stateP17;
    prevStateP18 = stateP18;
    prevStateP19 = stateP19;
    prevStateP20 = stateP20;
    prevStateP21 = stateP21;
    prevStateP22 = stateP22;
    prevStateP23 = stateP23;
    prevStateP24 = stateP24;
    prevStateP25 = stateP25;
    prevStateP26 = stateP26;
    prevStateP27 = stateP27;
    prevStateP28 = stateP28;
    prevStateP29 = stateP29;

    tmrReadPorts = 0;
    //WDT_Restart (WDT);
  } else {
    tmrReadPorts++;
  }


  // if ( tmr_VAPF_startMesh >= 2000) {
  //    tmr_VAPF_startMesh = 0;
  //    flg_VAPF_startMesh = true;
  //  } else {
  //    tmr_VAPF_startMesh++;
  //  }


    if (stringComplete) {
      flgLoraLineOK = false;
      cntLoraLineOK = 0;
      }


   
 

        


  if ( tmrMSec >= 1000) {
    tmrSec++;
    tmrMSec = 0;
    WDT_Restart (WDT);


   if ( cntLoraLineOK >= tmpDel) {
      flgLoraLineOK = true;
      cntLoraLineOK = 0;      
       } else{
        cntLoraLineOK++;
        }


    if (!flgWDT) {
      digitalWrite(pinWDT, false);
    }


    flgUpdateTime = true;
    //WDT_Restart (WDT);
    if (sendedProcess) {
      sendedTimeOut = true;
    }
    if ((curHour != 0) || (curMin != 0) || (curSec != 0)) {
      if ((curSec < 30) && (curMin != lastMin)) {

        lastHour = curHour;
        lastMin = curMin;
        lastSec = curSec;
        flgSavePulsePorts = true;
      }
    }
  } else {
    tmrMSec++;
  }



  if ( tmrSec >= 60) { ///  60
    tmrMin++;
    tmrSec = 0;
    //  sendSostDev();
//    lastHour = curHour;
//    lastMin = curMin;
//    lastSec = curSec;
   flgSendPortsPocket = true;
   // flgSavePulsePorts = true;   

  }





  if ( tmrMin >= 60) {//60
    tmrMin = 0;
    flgSendSostDev = true;
    chkLostFiles = true;
   // flgSendPortsPocket = true;
    
  }

}

void watchdogSetup(void) { }


void serialEvent1() {

  while (Serial1.available()) {
    inChar = (char)Serial1.read();

    if (inChar == '@') {
      stringComplete = true;  //
      // Serial.println(payloadString);
      //  break;
    } else
      payloadString += inChar;
    if (inChar == '%') {
      payloadString = "";
    }
  }
}




float getPwrV() {

  float u = analogRead(A11);
  u = (u * 3.3) / 4096;
  u = u + kofVps;
  return u;
}





void sendSostDev() {
  String strToSend = "";
  uint8_t termo = rtc.getTemp();
  float pwrV = getPwrV();
  String balans = "";
  int pwrSignal = 0;
  String ussd_phone_num = "";



#if COMM_MODULE == 1
  pwrSignal = modem.getSignalQuality();
  pwrSignal = (pwrSignal * 100) / 31; //-> %

  if (GPRS_OPER == 1) {
    // ussd_phone_num = modem.sendUSSD("*888#");
  }

  if (GPRS_OPER == 2) {
    // ussd_phone_num = modem.sendUSSD("*1#");
  }

  balans = getBalans();
  strToSend = dev_N + ";"  + "1" + ","  + String(pwrV) + ","  + String(termo) + ","  + String(pwrSignal) + "," + String(balans) ;
  serialLOGnoLN("sendMQTT >>>>: "); serialLOG(strToSend);

  if (!sendGPRS("4", strToSend)) {
    mqtt.disconnect();
    serialLOGnoLN("StateDev sendGPRS Error ");
  }

#endif
  //#if COMM_MODULE == 2
  //
  //strToSend = dev_N + ";"  + String(COMM_MODULE) + ","  + String(pwrV) + ","  + String(termo) + ","  + String("0");
  // serialLOGnoLN("sendMQTT >>>>: "); serialLOG(strToSend);
  //
  //      if (!sendWIFI("4", strToSend)) {
  //        serialLOG("sendWIFI Error ");
  //      }
  //#endif
  //
  //
}






void chkLostFilesLoop () {
  String tmpFnameFull = "";
  String tmpFname1 = "sending/send";;
  String tmpFname2 = ".txt";;
  String tmpStr2 = "";
  int16_t iz = 1001;

  for (iz; iz > 1; iz--) {

    WDT_Restart (WDT);

    tmpFnameFull = tmpFname1 + String(iz) + tmpFname2;
   // serialLOG(tmpFnameFull);
  
    if (SD.exists(tmpFnameFull)) {  //проверяем существует ли файл
        fNomSendFile = iz;
        fNameSendFile = tmpFnameFull;
        serialLOGnoLN(tmpFnameFull);
        serialLOG("    stat  >>>>>>>>>>>>>>>>>>>>>>>>>>>>ok.");  


      
//      SD.remove("config/statSend.txt");
//      statSendFile = SD.open("config/statSend.txt", FILE_WRITE);
//      if (statSendFile) {
//        tmpStr2 = String(iz) + ";0;0";
//        statSendFile.print(tmpStr2);
//        statSendFile.close();
//              
//      } else {
//        serialLOG("error opening statSend.txt");        
//      }      
      break;
    }




  } //for (int16_t iz=100; i>0;) {

  chkLostFiles = false;

}





bool sendPortsPocketNow(String strToSend2) {
  //we collect a string to send to the server and send it, собираем строку для отправки на сервер и отправляем

  // return false;
  // strToSend = dev_N +  "," + getTS() +  ";" + makeShtampPorts();
  //strToSend = strToSend.substring(0, strToSend.length()-1);  //удаляем последний разделитель строки
  // strToSend = "15,190;1,66,34,52,,24,45,321,523,4523,235,234:2,34,67,3,65,,34,66,,:3,67,3,,7,35,";

  //  if (!sendWIFI("1", strToSend2)) {
  //    serialLOG("415sendWIFI Error ");
  //    return false;
  //  } else {
  //    resetPulsePorts();
  //    return true;
  //  }


  //>>>>>>>>>>>>>

  WDT_Restart (WDT);

#if COMM_MODULE == 1
  if (!sendGPRS("1", strToSend2)) {
    mqtt.disconnect();
    serialLOG("~~~sendGPRS Error ");
    attemptSendCnt++;
    return false;
  } else {
    resetPulsePorts();
    serialLOG("~~~sendGPRS OK ");
    attemptSendCnt++;
    completedSendCnt++;
    return true;
  }
#endif
#if COMM_MODULE == 2
  if (!sendWIFI("1", strToSend2)) {
    serialLOG("~~~415sendWIFI Error ");
    attemptSendCnt++;
    return false;
  } else {
    resetPulsePorts();
    attemptSendCnt++;
    completedSendCnt++;
    return true;
  }
#endif

#if COMM_MODULE == 3

    String tmpTS = "0";
    uint32_t calcDir = 0;
   
       while (String(tmpTS) == "0") {
      tmpTS = getTS();
    }

  if (!sendLORA("1", strToSend2)) {
    nextCHNL();
    serialLOG("NO SENDED");
    attemptSendCnt++;
    return false;
  } else {
    serialLOG("SENDED");
    resetPulsePorts();
    attemptSendCnt++;
    completedSendCnt++;

    if (attemptSendCnt>-1) {
      calcDir = calcDirectory("/SENDING/");
      strToSend2 = dev_N +  "," + tmpTS +  ";" + String(attemptSendCnt) + "," + String(completedSendCnt)+ "," + String(calcDir);
    if (!sendLORA("varuna/7", strToSend2)) {
      serialLOG("err SENDED stat"); 
      } else {
      attemptSendCnt=0;
      completedSendCnt=0;
      serialLOG("SENDED stat"); 
      }
    }
    return true;
  }



#endif



  //>>>>>>>>>>>>>
  String sTmp = "412sendMQTT Protocol1(3): " + strToSend2;
  serialLOG(sTmp);


}




bool sendPortsPocketNow2() {
  //we collect a string to send to the server and send it, собираем строку для отправки на сервер и отправляем
  String strToSend = "";
  strToSend = dev_N + ";" + String(round(f1V)) + "," + String(round(f1I * 100)) + "," + String(round(f1rP * 100)) + "," + String(round(f2V)) + "," + String(round(f2I * 100)) + "," + String(round(f2rP * 100))
              + "," + String(round(f3V)) + "," + String(round(f3I * 100)) + "," + String(round(f3rP * 100));
  String sTmp = "402sendMQTT Protocol2: " + strToSend;



  //  if (!sendWIFI("2", strToSend)) {
  //    serialLOG("405sendWIFI Error ");
  //  }


  //>>>>>>>>>>>>>

  WDT_Restart (WDT);

#if COMM_MODULE == 1
  if (!sendGPRS("2", strToSend)) {
    mqtt.disconnect();
    serialLOGnoLN("sendGPRS Error ");
    return false;
  } else {
    return true;
  }
#endif
#if COMM_MODULE == 2
  if (!sendWIFI("2", strToSend)) {
    serialLOG("415sendWIFI Error ");
    return false;
  } else {
    return true;
  }
#endif
#if COMM_MODULE == 3

  if (!sendLORA("2", strToSend)) {
    nextCHNL();
    serialLOG("NO SENDED");
    return false;
  } else {
    serialLOG("SENDED");
    return true;
  }
#endif
  //>>>>>>>>>>>>>




  delay(100);
  // return false;
  // strToSend = dev_N +  "," + getTS() +  ";" + makeShtampPorts();
  //strToSend = strToSend.substring(0, strToSend.length()-1);  //удаляем последний разделитель строки
  // strToSend = "15,190;1,66,34,52,,24,45,321,523,4523,235,234:2,34,67,3,65,,34,66,,:3,67,3,,7,35,";

  serialLOG(sTmp);

}






void readConfigFile() {

  //read the configuration file, считал конфигурационный файл
  String tmpStr3 = "";

  configFile = SD.open("config/config.txt");
  if (configFile) {
    serialLOG("config.txt:");
    while (configFile.available()) {
      char sym;
      sym = (char)configFile.read();
      tmpStr3 += sym;
    }
    serialLOG(tmpStr3);

    char buffer[150];
    tmpStr3.toCharArray(buffer, sizeof(tmpStr3) + 1);
    char *p = buffer;
    char *str;
    int i = 0;
    String  tmpStr4 = "";

    //parsing the envelope and highlighting the message, разбор конверта и выделение сообщения
    while ((str = strtok_r(p, ";", &p)) != NULL) // установить разделитель
    {
      if (i == 0) {
        dev_N = String(str);
      }
      //  if (i==1) {tmpStr4=String(str);  serialLOG(str); startDate =  tmpStr4.toDouble();}
      if (i == 2) {
        tmpStr4 = String(str);
        qty_SendLine =  tmpStr4.toInt();
      }
      if (i == 3) {
        tmpStr4 = String(str);
        setHour =  tmpStr4.toInt();
      }
      if (i == 4) {
        tmpStr4 = String(str);
        setMin =  tmpStr4.toInt();
      }
      if (i == 5) {
        tmpStr4 = String(str);
        tmpStr4.toCharArray(apn, sizeof(tmpStr4) + 1);
      }
      i++;
    }
    serialLOG(String(dev_N));
    serialLOG(String(startDate));
    serialLOG(String(qty_SendLine));
    serialLOG(String(setHour));
    serialLOG(String(setMin));
    serialLOG(String(apn));

    configFile.close();
  } else {
    // if the file didn't open, print an error:
    serialLOG("error opening config.txt");
  }


}



#if COMM_MODULE == 1
boolean sendGPRS(String tpic, String msg) {
  serialLOGnoLN("=== sendGPRS(String tpic, String msg) ===        ");
  serialLOG(msg);
  bool gprsStat = modem.isGprsConnected();
  if (gprsStat) {
    if (mqttConnect( tpic, msg)) {
      serialLOG("=== MQTT CONNECTED ===");
      reconnectGPRScnt = 0;
      return true;
    } else {
      serialLOG("=== MQTT NOT CONNECTED ===");
      reconnectGPRScnt++;
      //if (reconnectGPRScnt > 1) {
      //      if (reconnectGPRS()) {
      //        reconnectGPRScnt = 0;
      //        serialLOG("reconnectGPRS OK");
      //        return false;
      //      } else  {
      //        reconnectGPRScnt++;
      //        serialLOG("reconnectGPRS Err");
      //        return false;
      //      }

      flg_nid_reconnectGPRS = true;

      return false;


    }
    return false;
    serialLOG("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~     sendGPRS Err");
    //  }
    //delay(10000);
    mqtt.loop();
  } else {
    flg_nid_reconnectGPRS = true;
    return false;
  }
}


String getBalans() {
  serialLOG("=== getBalans() ===");
  String ussd_balance = "";

#if GPRS_OPER == 1
  ussd_balance = modem.sendUSSD("*124*1#");

  if (String(ussd_balance) != "") {

    char charBuf[100];
    ussd_balance.toCharArray(charBuf, 100);

    char *p2 = charBuf;
    char *str;
    int i = 0;
    String tmpStrMas1[15] = "";

    ussd_balance = "";

    while ((str = strtok_r(p2, "\r\n", &p2)) != NULL) // установить разделитель
    {

      tmpStrMas1[i] = String(str);
      serialLOG(tmpStrMas1[i]);

      i++;
    }

    i = 0;
    for (int a = 0; a != 7; a++)  {
      char charBuf9[100];
      tmpStrMas1[a].toCharArray(charBuf9, 100);
      char *p9 = charBuf9;
      while ((str = strtok_r(p9, ":", &p9)) != NULL) // установить разделитель
      {
        if ((a == 0) || (a == 1) || (a == 3) || (a == 5)) {
          if (i == 1) {
            ussd_balance = ussd_balance + String(str);
            serialLOG( String(str));
          }
          i++;
        }
      }
      i = 0;
    }


  }  else {
    ussd_balance = "";
  }
WDT_Restart (WDT);
  String ussd_number = modem.sendUSSD("*888#");

  if (String(ussd_number) != "") {


    char charBuf3[100];
    ussd_number.toCharArray(charBuf3, 100);

    char *p3 = charBuf3;
    char *str;
    int i = 0;

    while ((str = strtok_r(p3, ":", &p3)) != NULL) // установить разделитель
    {
      if (i == 1) {
        ussd_number =  String(str);
      }
      i++;
    }


  } else {
    ussd_number = "";
  }

  ussd_balance = ussd_balance + "," + ussd_number;


#endif

#if GPRS_OPER == 2

WDT_Restart (WDT);
  ussd_balance = modem.sendUSSD("111*2*1#");

  if (String(ussd_balance) != "") {

    char charBuf[100];
    ussd_balance.toCharArray(charBuf, 100);

    char *p2 = charBuf;
    char *str;
    int i = 0;
    String tmpStrMas1[15] = "";

    serialLOG( ussd_balance);

    ussd_balance = "";

    while ((str = strtok_r(p2, " ", &p2)) != NULL) // установить разделитель
    {
      if ((i == 2) || (i == 5) || (i == 12)) {

        ussd_balance = ussd_balance + " " + String(str);
        serialLOG( String(str));

        i++;
      }
    }
    i = 0;

  }  else {
    ussd_balance = "";
  }


WDT_Restart (WDT);
  String ussd_number = modem.sendUSSD("*1#");

  if (String(ussd_number) != "") {


    char charBuf3[100];
    ussd_number.toCharArray(charBuf3, 100);

    char *p3 = charBuf3;
    char *str;
    int i = 0;

    while ((str = strtok_r(p3, " ", &p3)) != NULL) // установить разделитель
    {
      if (i == 7) {
        ussd_number =  String(str);
      }
      i++;
    }


  } else {
    ussd_number = "";
  }

  ussd_balance = ussd_balance + "," + ussd_number;



#endif





  return ussd_balance;

}




void mqttCallback(char* topic, byte* payload, unsigned int len) { //для ком.модуля 1
  serialLOG("=== mqttCallback(char* topic, ===");
  //  String tmpString = "";
  //  serialLOGnoLN("Message arrived [");
  //  tmpString = String(topic);
  //  serialLOGnoLN(tmpString);
  //  serialLOGnoLN("]: ");
  //  tmpString = String(payload);
  //  serialLOG(tmpString);
  //  serialLOG();

  // Only proceed if incoming message's topic matches
  if (String(topic) == String("4")) {
    serialLOGnoLN("Message 3 [");
    //    ledStatus = !ledStatus;
    //    digitalWrite(LED_PIN, ledStatus);
    //    mqtt.publish(topicLedStatus, ledStatus ? "1" : "0");
  }
}




boolean mqttConnect(String tpic, String msg) {
  serialLOG("=== mqttConnect(String tpic, String msg) ===");
  serialLOGnoLN("Connecting to ");
  serialLOGnoLN(broker);

  WDT_Restart (WDT);

  // Connect to MQTT Broker
  boolean status = mqtt.connect("a256aa", "mqttwarn", "Sozercatel1256");
  // boolean status = mqtt.connect("a256", "53", "66-252-7.ap-south-1.com");


  if (status == false) {
    serialLOG(" fail");
    return false;
  }
  serialLOG(" OK");
  char charBuf1[1] ;
  char charBuf2[250] ;
  // String  tpic = "1";
  // String  msg = "15,990;1,3,2,,24,45,,234:2,34,6,34,66,,:3,67,3,,88";
  tpic.toCharArray(charBuf1, tpic.length() + 1);
  msg.toCharArray(charBuf2, msg.length() + 1);
  mqtt.publish(charBuf1, charBuf2);
  //  mqtt.publish("1", "15,990;1,3,2,,24,45,,234:2,34,6,34,66,,:3,67,3,,35");
  mqtt.subscribe("4");
  return mqtt.connected();
}


boolean reconnectGPRS() {

  if (!flgSchedule) {
    serialLOG("=== reconnectGPRS() ===");
    //////////if (sessionDay == true) {
    serialLOGnoLN("Waiting for network...");
    WDT_Restart (WDT);
    if (!modem.waitForNetwork()) {
      serialLOG(" fail");
      return false;
    } else {
      serialLOG("network OK");
      serialLOGnoLN("Connecting to ");
      serialLOG(apn);
      WDT_Restart (WDT);
      if (!modem.gprsConnect(apn, user, pass)) {
        serialLOG("----------------   gprs fail      ????????????????????????????????????????????????????????????????????????????????");
        return false;
      } else {
        serialLOG("+++++++++++++++++    gprs OK         >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
        // boolean status = mqtt.connect("a256", "mqttwarn", "Sozercatel1256");
        flg_nid_reconnectGPRS = false;
        return true; //  mqtt.connected();
      }
    }

  } else {

    if ((flgSendPocket) && (sessionDay)) {
      serialLOG("=== reconnectGPRS() ===");
      //////////if (sessionDay == true) {
      serialLOGnoLN("Waiting for network...");
      if (!modem.waitForNetwork()) {
        serialLOG(" fail");
        return false;
      } else {
        serialLOG("network OK");
        serialLOGnoLN("Connecting to ");
        serialLOG(apn);
        if (!modem.gprsConnect(apn, user, pass)) {
          serialLOG("----------------   gprs fail      ????????????????????????????????????????????????????????????????????????????????");
          return false;
        } else {
          serialLOG("+++++++++++++++++    gprs OK         >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
          // boolean status = mqtt.connect("a256", "mqttwarn", "Sozercatel1256");
          flg_nid_reconnectGPRS = false;
          return true; //  mqtt.connected();
        }
      }
      //////////} else {return true;}
    }

  }





}
#endif






boolean sendWIFI(String tpic, String msg) {
  String  tmpStr5 = tpic + "%";  tmpStr5 = tmpStr5 + msg; tmpStr5 = tmpStr5 + "@";
  SerialAT.print(tmpStr5);
  sendedProcess = true;
  WDT_Restart (WDT);

  while (sendedProcess) {
    delay(1);
    serialLOGnoLN(".");

    while (Serial1.available()) {
      inChar = (char)Serial1.read();
    }

    if (char(inChar) == '1') {
      sendedTimeOut = false;
      sendedResult = true;
      inChar = '0';
    }

    if (sendedTimeOut) {
      sendedTimeOut = false;
      sendedProcess = false;
      sendedResult = false;
      serialLOG("618sendedTimeOut=1");
      return false;
    }
    if (sendedResult) {
      sendedTimeOut = false;
      sendedProcess = false;
      sendedResult = false;
      serialLOG("625sendedResult OK");
      return true;
    }
  }
}







bool sendPortsPocket() {

  String arrayStrPock[qty_SendLine];

  String strToSend = "";
  String tmpStr2 = "";
  String arrayStr[3] = "";
  int32_t a = 0;
  String tmpTS = "0";

  String tmpFname1 = "sending/send";;
  String tmpFname2 = ".txt";;

  WDT_Restart (WDT);

  fNameSendFile = tmpFname1 + String(fNomSendFile) + tmpFname2;  

  sendFile = SD.open(fNameSendFile);
  if (sendFile) {
     serialLOGnoLN("opening: ");   serialLOG(fNameSendFile);
    while (sendFile.available()) {
      char sym2;
      sym2 = (char)sendFile.read();
      // serialLOG(sym);
      tmpStr2 += sym2;
      // serialLOGnoLN("tmpStr2 : ");serialLOG(tmpStr2);
      if (sym2 == ':') {
        a++;
        if (a > arrayStr1val) {
          strToSend = strToSend + tmpStr2;
         // arrayStr1val = a;
          break;
        } else {
          tmpStr2 = "";
        }       
      }
    }
    sendFile.close();
  } else {
    // if the file didn't open, print an error:  
    serialLOGnoLN("error opening: ");   serialLOG(fNameSendFile);
  }





if (String(strToSend) != "") {
  serialLOGnoLN("731sendMQTT : "); serialLOG(strToSend);

  #if COMM_MODULE == 1
      if (!sendGPRS("1", strToSend)) {
        mqtt.disconnect();
        serialLOGnoLN("sendGPRS Error ");
    //    flgNoSend = true;
        return false;
      }
#endif
#if COMM_MODULE == 2
      if (!sendWIFI("1", strToSend)) {
        serialLOG("740sendWIFI Error ");
//        flgNoSend = true;
        return false;
      }
#endif
#if COMM_MODULE == 3

      if (!sendLORA("1", strToSend)) {
        nextCHNL();
        serialLOG("NO SENDED");
       // flgNoSend = true;
          return false;
      }
#endif
  else {
   serialLOG("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~    sendPortsPocket OK"); 
   
   arrayStr1val = a;     
    
    return true;
    }

} else {
  //записываем  в случае если обнаруженно что строка не содержит данных для отправки
  serialLOG("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~    sendPortsPocket NULL");
  if (SD.exists(fNameSendFile)) {
      SD.remove(fNameSendFile);
      arrayStr1val = 0; //курсор отправки на ноль
      cntTotalStr = 0; //курсор записи на ноль
       flgSendPortsPocket = false;

//#if COMM_MODULE == 3
       flgLoraLineOK = false;  
//#endif
       
     
            
       if (fNomSendFile>1) {
        fNomSendFile--;
       }
       serialLOGnoLN("del: ");  serialLOG(fNameSendFile);
      // return false;
       if (fNomSendFile == 1) {
        flgSendPortsPocket = false;
        //return false;
        } else {
          flgSendPortsPocket = true;
          //return true;
          }   
       
    }



 // if (fNomSendFile == 1) {flgSendPortsPocket = false;} else {flgSendPortsPocket = true;}       
  return false;
   
}



} //sendPortsPocket






void saveArhiv(String tmpStr2) {
  // t = rtc.getTime();
  // String statPorts = makeShtampPorts();
  // String tmpStr2 = "";
  // tmpStr2 = getTS() + ";"; tmpStr2 = tmpStr2 + statPorts;  tmpStr2 = tmpStr2 + ":";//
  /////////////////////test/////////tmpStr2 = String(getTS().toInt()-random(2000,5000)) + ";0"; // ts;nom_line


  //if it is midnight delete it if there is and create a new file with this day, если наступила полночь то удаляем если есть и создаем новый файл с этим днем
  uint8_t tmpHour = lastHour;
  uint8_t tmpMin = lastMin;
  String tmpTS = "0";

  String tmpDay = "";
  tmpDay =  String(t.date);
  String tmpMonth = "";
  tmpMonth = String(t.mon);
  String tmpData = "archive/";
  tmpData = tmpData + tmpMonth;
  tmpData = tmpData + "-";
  tmpData = tmpData + tmpDay;
  tmpData = tmpData + ".txt";

  WDT_Restart (WDT);

  if ((tmpHour == 0) && (tmpMin == 0)) {
    if (SD.exists(tmpData)) {
      SD.remove(tmpData);
      serialLOGnoLN(tmpData);
      serialLOG(" 835>>>>>>>>>>>DEL>>>>>>>>>>>>done.");
    }
  }

  //creating a new file, создаем новый файл
  statSendFile = SD.open(tmpData, FILE_WRITE);
  if (statSendFile) {
    statSendFile.print(tmpStr2);
    statSendFile.close();
    serialLOGnoLN(tmpData);
    serialLOG(" 845>>>>>>save Arhiv file>>>>>>>>>>>>>>>>>>>>>>>>>>>>done.");
  } else {
    serialLOGnoLN(tmpData);
    serialLOG(" 846error opening Arhiv file");
  }


}



void serialLOG(String serLog) {

  if (SerialDebug) {
    // serialLOGnoLN("859serLog  ");
    Serial.println(serLog);
  }

  if (SaveLog) {
    serLogStr = serLogStr + serLog + '\r' + '\n';
    if (serLogStr.length() > 230) {
      serLog = serLogStr;
      serLogStr = "";
      //t = rtc.getTime();
      String tmpStr2 = "";

      //если наступила полночь то удаляем если есть и создаем новый файл с этим днем
      uint8_t tmpHour = lastHour;
      uint8_t tmpMin = lastMin;
      uint8_t tmpSec = lastSec;

      tmpStr2 = String(tmpHour) + ":" + String(tmpMin) + ":" + String(tmpSec) + "    " + serLog;//

      String tmpDay =  String(t.date);
      String tmpMonth = String(t.mon);
      String tmpData = "log/";
      tmpData = tmpData + tmpMonth;
      tmpData = tmpData + "-";
      tmpData = tmpData + tmpDay;
      tmpData = tmpData + ".txt";

      if ((tmpHour == 0) && (tmpMin == 0)) {
        if (SD.exists(tmpData)) {
          SD.remove(tmpData);
          serialLOGnoLN(tmpData);
          serialLOG("1433>>>>>>>>>>> serialLOG tmpData DEL >>>>>>>>done.");
        }
      }

      //creating a new file, создаем новый файл
      statSendFile = SD.open(tmpData, FILE_WRITE);
      if (statSendFile) {
        statSendFile.println(tmpStr2);
        statSendFile.close();
        //serialLOGnoLN(tmpData);
        serialLOG("900>>>>>>save Log file>>>>>>>>>>>>>>>>>>>>>>>>>>>>done.");
      } else {
        serialLOGnoLN(tmpData);
        serialLOG("903 error opening Log file");
      }
    }
  }
}




void serialLOGnoLN(String serLog) {

  if (SerialDebug) {
    //  serialLOGnoLN("915serLog  ");
    Serial.print(serLog);
  }

  if (SaveLog) {


    serLogStr = serLogStr + serLog;
    if (serLogStr.length() > 230) {
      serLog = serLogStr;
      serLogStr = "";
      //  t = rtc.getTime();
      String tmpStr2 = "";

      //если наступила полночь то удаляем если есть и создаем новый файл с этим днем
      uint8_t tmpHour = lastHour;
      uint8_t tmpMin = lastMin;
      uint8_t tmpSec = lastSec;

      tmpStr2 = String(tmpHour) + ":" + String(tmpMin) + ":" + String(tmpSec) + "    " + serLog;//

      String tmpDay =  String(t.date);
      String tmpMonth = String(t.mon);
      String tmpData = "log/";
      tmpData = tmpData + tmpMonth;
      tmpData = tmpData + "-";
      tmpData = tmpData + tmpDay;
      tmpData = tmpData + ".txt";

      if ((tmpHour == 0) && (tmpMin == 0)) {
        if (SD.exists(tmpData)) {
          SD.remove(tmpData);
          serialLOGnoLN(tmpData);
          serialLOG(">>>>>>>>>>>DEL>>>>>>>>>>>>done.");
        }
      }

      //создаем новый файл
      statSendFile = SD.open(tmpData, FILE_WRITE);
      if (statSendFile) {
        statSendFile.println(tmpStr2);
        statSendFile.close();
        //serialLOGnoLN(tmpData);
        //serialLOG(">>>>>>save Log file>>>>>>>>>>>>>>>>>>>>>>>>>>>>done.");
      } else {
        serialLOGnoLN(tmpData);
        serialLOG(" error opening Log file");
      }
    }
  }
}




String getTS() {
  WDT_Restart (WDT);
  uint32_t intTmp = (rtc.getUnixTime(rtc.getTime()) - startDate) / 60 ;
  //  intTmp = intTmp - startDate ;
  //  intTmp = intTmp/60;
  serialLOGnoLN("974UNIXTime minute = "); serialLOG(String(intTmp));
  String tsTmp = String(intTmp);
  return tsTmp;
}




void saveStatePorts(String tmpStr) {

  //test-----------------------------------------------------------------------------------
//if (String(oldTS) != String(newTS)) {
  String tmpTS = "0";
  String tmpStr2 = "";
  String arrayStr[3] = "";
  uint32_t a = 0;
  uint32_t sizeFile = 100000;
  WDT_Restart (WDT);
//  statSendFile = SD.open("config/statSend.txt");
//  if (statSendFile) {
//    serialLOG("647statSend.txt:");
//    while (statSendFile.available()) {
//      char sym;
//      sym = (char)statSendFile.read();
//      // serialLOG(sym);
//      if (sym == ';') {
//        a++;
//      } else {
//        arrayStr[a] += sym;
//      }
//      // serialLOGnoLN(a); serialLOGnoLN(" : "); serialLOG(String(arrayStr[a]));
//    }
//    if ((arrayStr[0] != "") && (arrayStr[1] != "")) {
//      tsToSend =  arrayStr[2].toInt();
//      cntTotalStr =  arrayStr[1].toInt();
//      fNomSendFile = arrayStr[0].toInt();
      if (cntTotalStr > qntStrFile) {
        fNomSendFile++;
        cntTotalStr =  0;
        arrayStr1val = 0;
        tsToSend = 0;
      }
      fNameSendFile = "sending/send";
      fNameSendFile = fNameSendFile + String(fNomSendFile);
      fNameSendFile = fNameSendFile + ".txt";
    while (String(tmpTS) == "0") {
        tmpTS = getTS();
      }
      tsToSend = tmpTS.toInt();
      
    serialLOGnoLN("nom File = ");
    serialLOG(String(fNomSendFile));
    serialLOGnoLN("arrayStr1val = ");    serialLOG(String(arrayStr1val));
    serialLOGnoLN("cntTotalStr = ");    serialLOG(String(cntTotalStr));
    serialLOGnoLN("last ts to send = ");
    serialLOG(String(tsToSend));
      cntTotalStr++;
      
   // }
//    if ((arrayStr[0] == "") || (arrayStr[1] == "")) {

//      if (SD.exists("config/statSend.txt")) {
//        SD.remove("config/statSend.txt");
//        tsToSend = 0;
//        cntTotalStr =  0;
//        fNomSendFile = 1;
//        //SD.remove(fNameSendFile);
//        serialLOG("668>>>>>>statSend.txt>>>>>send.txt>>>>>>>>>>>DEL>>>>>>>>>>>>done.");
//      }
//    }

//    statSendFile.close();
//  } else {
//    // if the file didn't open, print an error:
//    serialLOG("675error opening statSend.txt");
//  }

  //test end----------------------------------------------------------------------------------------




  sendFile = SD.open(fNameSendFile, FILE_WRITE);
  if (sendFile) {
    //  tmpStr = tmpStr + ":";
    sendFile.print(tmpStr);
    sizeFile = sendFile.size();
    sendFile.close();
    if (sizeFile>sizeFileVol) {fNomSendFile++;}
    serialLOG(">>>986saveStatePorts()>>>>>>>>>done.");
  } else {
    serialLOG("990error opening send.txt");
  }
  resetPulsePorts();
  oldTS = newTS;

  //000000000000000000000 test
//  tmpStr2 = String(fNomSendFile) + ";" + String(cntTotalStr + 1) + ";" + String(tsToSend); // ts;nom_line
//  if (SD.exists("config/statSend.txt")) {
//    SD.remove("config/statSend.txt");
//    serialLOG("749>>>>>>statSend.txt>>>>>>>>>>>DEL>>>>>>>>>>>>done.");
//  }
//  statSendFile = SD.open("config/statSend.txt", FILE_WRITE);
//  if (statSendFile) {
//    statSendFile.print(tmpStr2);
//    statSendFile.close();
//    serialLOG("755>>>>>>new  statSend.txt>>>>>>>>>>>>>>>>>>>>>>>>>>>>done.");
//  } else {
//    serialLOG("757error opening statSend.txt");
//  }

  //000000000000000000000 test end
//}



}




String makeShtampPorts() {
  String tmpStr = "";
  bool flgEndStr = false;

  if (pulseP29 > 0) {
    tmpStr = String(pulseP29);
    flgEndStr = true;
  }
  if (pulseP28 > 0) {
    tmpStr = String(pulseP28) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP27 > 0) {
    tmpStr = String(pulseP27) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP26 > 0) {
    tmpStr = String(pulseP26) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP25 > 0) {
    tmpStr = String(pulseP25) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP24 > 0) {
    tmpStr = String(pulseP24) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP23 > 0) {
    tmpStr = String(pulseP23) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }

  if (pulseP22 > 0) {
    tmpStr = String(pulseP22) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP21 > 0) {
    tmpStr = String(pulseP21) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP20 > 0) {
    tmpStr = String(pulseP20) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP19 > 0) {
    tmpStr = String(pulseP19) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP18 > 0) {
    tmpStr = String(pulseP18) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP17 > 0) {
    tmpStr = String(pulseP17) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP16 > 0) {
    tmpStr = String(pulseP16) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP15 > 0) {
    tmpStr = String(pulseP15) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP14 > 0) {
    tmpStr = String(pulseP14) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP13 > 0) {
    tmpStr = String(pulseP13) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP12 > 0) {
    tmpStr = String(pulseP12) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP11 > 0) {
    tmpStr = String(pulseP11) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP10 > 0) {
    tmpStr = String(pulseP10) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP9 > 0) {
    tmpStr = String(pulseP9) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP8 > 0) {
    tmpStr = String(pulseP8) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP7 > 0) {
    tmpStr = String(pulseP7) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP6 > 0) {
    tmpStr = String(pulseP6) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP5 > 0) {
    tmpStr = String(pulseP5) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP4 > 0) {
    tmpStr = String(pulseP4) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP3 > 0) {
    tmpStr = String(pulseP3) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP2 > 0) {
    tmpStr = String(pulseP2) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP1 > 0) {
    tmpStr = String(pulseP1) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }
  if (pulseP0 > 0) {
    tmpStr = String(pulseP0) + "," + tmpStr;
    flgEndStr = true;
  } else {
    if (flgEndStr) {
      tmpStr = "," + tmpStr;
    }
  }


  return tmpStr;
}

void resetPulsePorts() {
  pulseP0 = 0; pulseP1 = 0; pulseP2 = 0; pulseP3 = 0; pulseP4 = 0; pulseP5 = 0; pulseP6 = 0; pulseP7 = 0;
  pulseP8 = 0; pulseP9 = 0; pulseP10 = 0; pulseP11 = 0; pulseP12 = 0; pulseP13 = 0; pulseP14 = 0; pulseP15 = 0;
  pulseP16 = 0; pulseP17 = 0; pulseP18 = 0; pulseP19 = 0; pulseP20 = 0; pulseP21 = 0; pulseP22 = 0; pulseP23 = 0;
  pulseP24 = 0; pulseP25 = 0; pulseP26 = 0; pulseP27 = 0; pulseP28 = 0; pulseP29 = 0;
}





void setup() {

  //  pulseP0 = 9; //test
  TimerLib.setInterval_us(timer_1ms, 1000);

  pinMode(pinPorts[0], INPUT); //INPUT_PULLUP
  pinMode(pinPorts[1], INPUT);
  pinMode(pinPorts[2], INPUT);
  pinMode(pinPorts[3], INPUT);
  pinMode(pinPorts[4], INPUT);
  pinMode(pinPorts[5], INPUT);
  pinMode(pinPorts[6], INPUT);
  pinMode(pinPorts[7], INPUT);
  pinMode(pinPorts[8], INPUT);
  pinMode(pinPorts[9], INPUT);
  pinMode(pinPorts[10], INPUT);
  pinMode(pinPorts[11], INPUT);
  pinMode(pinPorts[12], INPUT);
  pinMode(pinPorts[13], INPUT);
  pinMode(pinPorts[14], INPUT);
  pinMode(pinPorts[15], INPUT);
  pinMode(pinPorts[16], INPUT);
  pinMode(pinPorts[17], INPUT);
  pinMode(pinPorts[18], INPUT);
  pinMode(pinPorts[19], INPUT);
  pinMode(pinPorts[20], INPUT);
  pinMode(pinPorts[21], INPUT);
  pinMode(pinPorts[22], INPUT);
  pinMode(pinPorts[23], INPUT);
  pinMode(pinPorts[24], INPUT);
  pinMode(pinPorts[25], INPUT);
  pinMode(pinPorts[26], INPUT);
  pinMode(pinPorts[27], INPUT);
  pinMode(pinPorts[28], INPUT);
  pinMode(pinPorts[29], INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(pinRadioGND, OUTPUT);
  pinMode(pinWDT, OUTPUT);
  pinMode(pinRele, OUTPUT);
  digitalWrite(pinRadioGND, true);
  digitalWrite(pinWDT, true);
  digitalWrite(pinRele, true);

  analogReadResolution(12);
  pinMode(pin_f1V, INPUT);
  pinMode(pin_f2V, INPUT);
  pinMode(pin_f3V, INPUT);
  pinMode(pin_f1I, INPUT);
  pinMode(pin_f2I, INPUT);
  pinMode(pin_f3I, INPUT);

  rtc.begin();
  delay(5);
  WDT_Restart (WDT);

  Serial.begin(115200);
  SerialAT.begin(115200);
  // delay(100);

  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(50);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(150);                       // wait for a second
  }

  emon1.voltage(pin_f1V, kK_f1V, 1.7);  // Voltage: input pin, calibration, phase_shift
  emon2.voltage(pin_f2V, kK_f2V, 1.7);  // Voltage: input pin, calibration, phase_shift
  emon3.voltage(pin_f3V, kK_f3V, 1.7);  // Voltage: input pin, calibration, phase_shift
  emon1.current(pin_f1I, kK_f1I);             // Current: input pin, calibration.
  emon2.current(pin_f2I, kK_f2I);             // Current: input pin, calibration.
  emon3.current(pin_f3I, kK_f3I);             // Current: input pin, calibration.

  if (!SD.begin(4)) {
    serialLOG(" ");
    serialLOG(" ");
    serialLOG(" ");
    serialLOG("SD card initialization failed!");
    return;
  } else {
    serialLOG(" ");
    serialLOG(" ");
    serialLOG(" ");
    serialLOG("SD card initialization done.");
  }
  serialLOGnoLN("ver: ");  serialLOG(ver);
  serialLOGnoLN("dev_N: ");  serialLOG(dev_N);


  resetPulsePorts();


#if COMM_MODULE == 1
  // MQTT Broker setup
  mqtt.setServer(broker, 1885);
  mqtt.setCallback(mqttCallback);
#endif

  //  rtc.setDOW(FRIDAY);     // Set Day-of-Week to SUNDAY
  //    rtc.setTime(9, 18, 0);     // Set the time to 12:00:00 (24hr format)
  //    rtc.setDate(5, 4, 2019); // Set the date to January 1st, 2014

  serialLOG(rtc.getTimeStr());
  serialLOG(rtc.getDateStr());
  serialLOG(rtc.getDOWStr());

  t = rtc.getTime();
  curHour = t.hour;
  curMin = t.min;
  curSec = t.sec;

  /////////////////////////////////////////////////readConfigFile();

#if COMM_MODULE == 3

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    serialLOG("LoRa radio init failed");
    serialLOG("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  serialLOG("LoRa radio init OK!");
    if (!rf95.setFrequency(RF95_FREQ)) {
    serialLOG("setFrequency failed");
    while (1);
  }
  serialLOGnoLN("Set Freq to: "); serialLOG(String(RF95_FREQ));
  rf95.setSpreadingFactor(7);
  rf95.setSignalBandwidth(500000);
  rf95.setCodingRate4(5);
  rf95.setTxPower(23, false);
    nextCHNL();
  #endif

  WDT_Restart (WDT);



}




void loop() {





  //>>>>>>>>>>>>>>>>>>>>>>>>>>>TEST

  //serialLOGnoLN(prevstate_L3Hz); serialLOGnoLN("   "); serialLOG(state_L3Hz);


  if (chkLostFiles) {

    chkLostFilesLoop();

  } // end      if (chkLostFiles) {


  //>>>>>>>>>>>>>>>>>>>>>>>>>>>TEST



WDT_Restart (WDT);

  if (stringComplete) {

    char buffer[150];
    //  payloadString.toCharArray(buffer, sizeof(payloadString) + 1);
    payloadString.toCharArray(buffer, payloadString.length() + 1);
    char *p = buffer;
    char *str;
    int i = 0;
    String  tmpStr6 = "";
    int devNcomper = 0;
    int nomZaprosa = 0;
    int param1 = 0;
    int param2 = 0;
    int param3 = 0;
    int param4 = 0;
    int param5 = 0;
    int param6 = 0;
    String tmpTS = "0";

    //    <=inTopic - 0,0,21,2,2019,10,5,30   в ответ сервер публикает:    ном.устройства(0-для всех), номерзапроса, дату, время

    //parsing the envelope and highlighting the message, разбор конверта и выделение сообщения
    while ((str = strtok_r(p, ",", &p)) != NULL) // установить разделитель
    {
      if (i == 0) {
        tmpStr6 = String(str);
        devNcomper = tmpStr6.toInt();
      }
      if (i == 1) {
        tmpStr6 = String(str);
        nomZaprosa =  tmpStr6.toInt();
      }

      //------------------------    nomZaprosa == 0
      if (nomZaprosa == 0) {
        if (i == 2) {
          tmpStr6 = String(str);
          param1 =  tmpStr6.toInt();
        }
        if (i == 3) {
          tmpStr6 = String(str);
          param2 =  tmpStr6.toInt();
        }
        if (i == 4) {
          tmpStr6 = String(str);
          param3 =  tmpStr6.toInt();
        }
        if (i == 5) {
          tmpStr6 = String(str);
          param4 =  tmpStr6.toInt();
        }
        if (i == 6) {
          tmpStr6 = String(str);
          param5 =  tmpStr6.toInt();
        }
        if (i == 7) {
          tmpStr6 = String(str);
          param6 =  tmpStr6.toInt();
        }

      }
      //------------------------
      //<=inTopic  15,1,1,29 - запрос диапазона данных из архива (ном.устройства идинтиф.запроса, месяц, день)
      ////<=inTopic 15,1,1,29-request a range of data from the archive (nom.device identif.request, month, day)
      //------------------------    nomZaprosa == 1
      if (nomZaprosa == 1) {
        if (i == 2) {
          tmpStr6 = String(str);
          param1 =  tmpStr6.toInt();
        }
        if (i == 3) {
          tmpStr6 = String(str);
          param2 =  tmpStr6.toInt();
        }

        //here is the mechanism for copying data from an archive to a file send.txt, здесь механизм копирования данных из архива в файл send.txt
      }
      //------------------------


      i++;
    }

    uint8_t tmpSec = random(1, 59);

    if (nomZaprosa == 0) {
      rtc.setTime(param4, param5, tmpSec);     // Set the time to 12:00:00 (24hr format)
      rtc.setDate(param1, param2, param3);   // Set the date to January 1st, 2014
    }

    serialLOG(String(devNcomper));
    serialLOG(String(nomZaprosa));
    serialLOG(String(param1));
    serialLOG(String(param2));
    serialLOG(String(param3));
    serialLOG(String(param4));
    serialLOG(String(param5));
    serialLOG(String(param6));




    serialLOG(payloadString);
    payloadString = "";

    stringComplete = false;
  }




  //if (!sendArhivPocket("xxx.txt")) {
  //
  //  Serial.println("_____________________________________________________+++++++++++++++++++++++++++++++++++++++++++ end arhive file");
  //  }
  //



  if (flgUpdateTime) {

    t = rtc.getTime();
    curHour = t.hour;
    curMin = t.min;
    curSec = t.sec;


    if (sessionDay == false) {
      digitalWrite(pinRadioGND, false);
    } else {
      digitalWrite(pinRadioGND, true);
    }

    serialLOGnoLN("  P0: "); serialLOGnoLN(String(pulseP0));
    serialLOGnoLN("  P1: "); serialLOGnoLN(String(pulseP1));
    serialLOGnoLN("  P2: "); serialLOGnoLN(String(pulseP2));
    serialLOGnoLN("  P3: "); serialLOGnoLN(String(pulseP3));
    serialLOGnoLN("  P4: "); serialLOGnoLN(String(pulseP4));
    serialLOGnoLN("  P5: "); serialLOGnoLN(String(pulseP5));
    serialLOGnoLN("  P6: "); serialLOGnoLN(String(pulseP6));
    serialLOGnoLN("  P7: "); serialLOGnoLN(String(pulseP7));
    serialLOGnoLN("  P8: "); serialLOGnoLN(String(pulseP8));
    serialLOGnoLN("  P9: "); serialLOGnoLN(String(pulseP9));
    serialLOGnoLN("  P10: "); serialLOGnoLN(String(pulseP10));
    serialLOGnoLN("  P11: "); serialLOGnoLN(String(pulseP11));
    serialLOGnoLN("  P12: "); serialLOGnoLN(String(pulseP12));
    serialLOGnoLN("  P13: "); serialLOGnoLN(String(pulseP13));
    serialLOGnoLN("  P14: "); serialLOGnoLN(String(pulseP14));
    serialLOGnoLN("  P15: "); serialLOG(String(pulseP15));
    serialLOGnoLN("  P16: "); serialLOGnoLN(String(pulseP16));
    serialLOGnoLN("  P17: "); serialLOGnoLN(String(pulseP17));
    serialLOGnoLN("  P18: "); serialLOGnoLN(String(pulseP18));
    serialLOGnoLN("  P19: "); serialLOGnoLN(String(pulseP19));
    serialLOGnoLN("  P20: "); serialLOGnoLN(String(pulseP20));
    serialLOGnoLN("  P21: "); serialLOGnoLN(String(pulseP21));
    serialLOGnoLN("  P22: "); serialLOGnoLN(String(pulseP22));
    serialLOGnoLN("  P23: "); serialLOGnoLN(String(pulseP23));
    serialLOGnoLN("  P24: "); serialLOGnoLN(String(pulseP24));
    serialLOGnoLN("  P25: "); serialLOGnoLN(String(pulseP25));
    serialLOGnoLN("  P26: "); serialLOGnoLN(String(pulseP26));
    serialLOGnoLN("  P27: "); serialLOGnoLN(String(pulseP27));
    serialLOGnoLN("  P28: "); serialLOGnoLN(String(pulseP28));
    serialLOGnoLN("  P29: "); serialLOG(String(pulseP29));


    serialLOGnoLN(String(curHour));  serialLOGnoLN(":");  serialLOG(String(curMin));

    WDT_Restart (WDT);

    if (flgSchedule == true) {
      if ((lastHour == setHour) && (lastMin >= setMin))  {
        flgSendPocket = true;
      }
      if (lastHour < setHour) {
        flgSendPocket = false;
        sessionDay = true;
      }
      if ((flgSendPocket) && (sessionDay)) {
        serialLOG("session on");
      } else {
        serialLOG("session off");
      }
    } else {
      flgSendPocket = true;
      sessionDay = true;
    }


    f1rP       = 0;
    f1V        = 0;
    f1I        = 0;

    f2rP       = 0;
    f2V        = 0;
    f2I        = 0;

    f3rP       = 0;
    f3V        = 0;
    f3I        = 0;


    for (int i = 0; i < 3; i++) {
      emon1.calcVI(20, 2000);        // Calculate all. No.of half wavelengths (crossings), time-out
      //   emon1.serialprint();           // Print out all variables (realpower, apparent power, Vrms, Irms, power factor)
      f1rP       += emon1.powerFactor;        //extract Real Power into variable
      f1V        += emon1.Vrms;             //extract Vrms into Variable
      f1I        += emon1.Irms;             //extract Irms into Variable
      emon2.calcVI(20, 2000);        // Calculate all. No.of half wavelengths (crossings), time-out
      //    emon2.serialprint();           // Print out all variables (realpower, apparent power, Vrms, Irms, power factor)
      f2rP       += emon2.powerFactor;        //extract Real Power into variable
      f2V        += emon2.Vrms;             //extract Vrms into Variable
      f2I        += emon2.Irms;             //extract Irms into Variable
      emon3.calcVI(20, 2000);        // Calculate all. No.of half wavelengths (crossings), time-out
      //   emon3.serialprint();           // Print out all variables (realpower, apparent power, Vrms, Irms, power factor)
      f3rP       += emon3.powerFactor;        //extract Real Power into variable
      f3V        += emon3.Vrms;             //extract Vrms into Variable
      f3I        += emon3.Irms;             //extract Irms into Variable

      if (i == 2) {
        f1rP       = f1rP / 3;
        f1V        = f1V / 3;
        f1I        = f1I / 3;

        f2rP       = f2rP / 3;
        f2V        = f2V / 3;
        f2I        = f2I / 3;

        f3rP       = f3rP / 3;
        f3V        = f3V / 3;
        f3I        = f3I / 3;
      }
      delay(20);
    }


    if (f1I < 0.5) {
      f1I = 0;
      f1rP = 0;
    }
    if (f1V < 20) {
      f1V = 0;
      f1I = 0;
      f1rP = 0;
    } else {
      f1V = f1V + K2_V1;
    }

    if (f2I < 0.5) {
      f2I = 0;
      f2rP = 0;
    }
    if (f2V < 20) {
      f2V = 0;
      f2I = 0;
      f2rP = 0;
    } else {
      f2V = f2V + K2_V2;
    }

    if (f3I < 0.5) {
      f3I = 0;
      f3rP = 0;
    }
    if (f3V < 20) {
      f3V = 0;
      f3I = 0;
      f3rP = 0;
    } else {
      f3V = f3V + K2_V3;
    }



    serialLOGnoLN(String(f1rP));  serialLOGnoLN("   ");  serialLOGnoLN(String(f1V));  serialLOGnoLN("   ");  serialLOG(String(f1I));
    serialLOGnoLN(String(f2rP));  serialLOGnoLN("   ");  serialLOGnoLN(String(f2V));  serialLOGnoLN("   ");  serialLOG(String(f2I));
    serialLOGnoLN(String(f3rP));  serialLOGnoLN("   ");  serialLOGnoLN(String(f3V));  serialLOGnoLN("   ");  serialLOG(String(f3I));

    WDT_Restart (WDT);

    flgUpdateTime = false;

  }

#if COMM_MODULE == 1
  if (flgSendPocket) {
    if (sessionDay) {
      //    if (flgSchedule) { sendPortsPocket();}
    } else {
      if (flgSchedule) {
        modem.gprsDisconnect();
      }
    }
  }
#endif


uint16_t cntSndPkt = 0;

  if (flgSavePulsePorts) {
    String tmpTS = "0";

    while (String(tmpTS) == "0") {
      tmpTS = getTS();
    }
    
    String dateShtamp =  dev_N +  "," + tmpTS +  ";" + makeShtampPorts() + ":";
    serialLOG(dateShtamp);
 //   saveArhiv(dateShtamp);

#if COMM_MODULE == 1
 if (!sendPortsPocketNow(dateShtamp)) {    
      saveStatePorts(dateShtamp);
    }  else {
      attemptSendCnt++;     
      while (sendPortsPocket()) {
        attemptSendCnt++;
        completedSendCntt++;
        delay(delayBetSend);
        serialLOGnoLN(".");
      }
    }
#endif
#if COMM_MODULE == 2
 if (!sendPortsPocketNow(dateShtamp)) {    
      saveStatePorts(dateShtamp);
    }  else {
      attemptSendCnt++;      
      while (sendPortsPocket()) {
        attemptSendCnt++;
        completedSendCnt++;
        delay(delayBetSend);
        serialLOGnoLN(".");
      }
    }
#endif
#if COMM_MODULE == 3

if (flgLoraLineOK) {
 if (!sendPortsPocketNow(dateShtamp)) {    
      saveStatePorts(dateShtamp);
    }  else {     
      while (sendPortsPocket()) {
        attemptSendCnt++;
        completedSendCnt++;
        delay(delayBetSend);
        serialLOGnoLN(".");
      } 
     }
} else {
  saveStatePorts(dateShtamp);
  }



    
#endif



    if (sendEnergyInfo) {
      if (!sendPortsPocketNow2()) {
        serialLOG("Err send protocol 2");
      }
    }
    flgSavePulsePorts = false;
    // sendPortsPocket();  getTS(); ///ТОЛЬКО ДЛЯ ТЕСТА>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

  }

  flgWDT = false;

  if (flgSendSostDev) {
    sendSostDev();
    flgSendSostDev = false;
  }

#if COMM_MODULE == 1
  if (flg_nid_reconnectGPRS) {
    if (reconnectGPRS()) {
      serialLOG("reconnectGPRS OK");
    } else  {
      serialLOG("reconnectGPRS Err");
    }
  }
#endif



#if COMM_MODULE == 3

WDT_Restart (WDT);

 uint8_t buf2[RH_RF95_MAX_MESSAGE_LEN];

    uint8_t len = sizeof(buf2);


    // Should be a reply message for us now
    if (rf95.recv(buf2, &len))
    {

 if ((char)buf2[0] == '%') {
  for (int iii=0; iii<len; iii++) {
    inChar = (char)buf2[iii];

    if (inChar == '@') {
      stringComplete = true;  //
      // Serial.println(payloadString);
      //  break;
    } else
      payloadString += inChar;
    if (inChar == '%') {
      payloadString = "";
    }
  }
      serialLOGnoLN("Got reply: ");
      serialLOG((char*)buf2);
      serialLOGnoLN("RSSI: ");
      serialLOG(String(rf95.lastRssi(), DEC));

    

      serialLOGnoLN("len: ");
      serialLOG(String(len, DEC));
      serialLOGnoLN("payloadString: ");
      serialLOG(payloadString);

    } 
  }


//if ((flgSendPortsPocket)&&(flgLoraLineOK)) {
//while (sendPortsPocket()) {
//        delay(delayBetSend);
//        serialLOGnoLN(".");
//      }      
//}

#endif









} //end loop




//
//
//bool sendArhivPocket(String nameArhivFile) {
//  // return false;
//  //read the timestamp and number of the first line to send, считал таймштамп и номер первой строки для отправки
//
//  String strToSend = "";
//  String tmpStr2 = "";
//  String arrayStr[2] = "";
//  int a = 0;
//
//
//  //считал заданое колво строк из send.txt
//
//  String arrayStrPock[1];
//  a = 0;
//  uint16_t  b = 0;
//
//  arrayStrArhval = 1;
//
//  sendFile = SD.open("archive/2-24.txt");
//  if (sendFile) {
//    serialLOG("archive.txt:");
//    while (sendFile.available()) {
//      char sym2;
//      sym2 = (char)sendFile.read();
//      // serialLOG(sym);
//      tmpStr2 += sym2;
//      serialLOGnoLN("tmpStr2 : "); serialLOG(tmpStr2);
//      if (sym2 == NULL) {
//        strToSend =  "";
//        break;
//      }
//      if (sym2 == ':') {
//        if (a >= arrayStrArhval)  {
//          // arrayStrPock[b] = tmpStr2;
//          serialLOGnoLN(String(b)); serialLOGnoLN(" : "); serialLOG(String(arrayStrPock[b]));
//          // b++;
//          strToSend =  tmpStr2;
//        }
//        tmpStr2 = "";
//        if (a >= (arrayStrArhval + 1 )) {
//          break;
//        }
//        a++;
//      }
//    }
//
//    statSendFile.close();
//    arrayStrArhval++;
//  } else {
//    // if the file didn't open, print an error:
//    serialLOG("error opening x.txt");
//  }
//  //собираем строку для отправки на сервер и отправляем
//  if (strToSend != "" ) { //если новая стока есть то..
//    uint16_t ii = 0;
//    uint16_t cnt_Fsym = 0;
//    while (strToSend.length() > ii) {
//      if (char(strToSend[ii]) != ':') {
//        cnt_Fsym++;
//      }
//      ii++;
//    }
//    if (cnt_Fsym != 0) {
//
//      serialLOGnoLN("strToSend: ??????????>>>>>>>>>>>>>"); serialLOG(strToSend);
//      strToSend = dev_N + ","  + strToSend;
//      strToSend = strToSend.substring(0, strToSend.length() - 1); //удаляем последний разделитель строки
//      // strToSend = "15,190;1,66,34,52,,24,45,321,523,4523,235,234:2,34,67,3,65,,34,66,,:3,67,3,,7,35,";
//      serialLOGnoLN("sendMQTT >>>>: "); serialLOG(strToSend);
//#if COMM_MODULE == 1
//      if (!sendGPRS("1", strToSend)) {
//        mqtt.disconnect();
//        serialLOGnoLN("sendGPRS Error ");
//      }
//#endif
//#if COMM_MODULE == 2
//      if (!sendWIFI("1", strToSend)) {
//        serialLOG("sendWIFI Error ");
//      }
//#endif
//#if COMM_MODULE == 3
//
//      if (!sendLORA("1", strToSend)) {
//
//        CHNL++;
//        if (CHNL > 3) {
//          CHNL = 1;
//        }
//
//
//switch (CHNL) {
//    case 1:
//        rf95.setSpreadingFactor(12);
//        rf95.setSignalBandwidth(500000);
//        rf95.setCodingRate4(5);
//        if (!rf95.setFrequency(866.0)) {
//                Serial.println("setFrequency failed");
//                while (1);
//              }
//        Serial.print("CHNL: ");      Serial.println(CHNL);
//      break;
//    case 2:
//        rf95.setSpreadingFactor(7);
//        rf95.setSignalBandwidth(500000);
//        rf95.setCodingRate4(5);
//        if (!rf95.setFrequency(865.5)) {
//                Serial.println("setFrequency failed");
//                while (1);
//              }
//        Serial.print("CHNL: ");      Serial.println(CHNL);
//      break;
//    case 3:
//        rf95.setSpreadingFactor(11);
//        rf95.setSignalBandwidth(500000);
//        rf95.setCodingRate4(5);
//        if (!rf95.setFrequency(866.5)) {
//                Serial.println("setFrequency failed");
//                while (1);
//              }
//        Serial.print("CHNL: ");      Serial.println(CHNL);
//        break;
//  }
//
//        delay(5);
//
//        Serial.println("NO SENDED");
//
//        return false;
//      } else {
//        Serial.println("SENDED");
//        resetPulsePorts();
//        return true;
//      }
//#endif
//      else {
//        delay(100);
//      }
//    } 
//
//    return true;
//  } else {
//    return false;
//  }
//}
//


boolean sendLORA(String tpic, String msg) {
  String  tmpStr5 = tpic + "%";  tmpStr5 = tmpStr5 + msg; tmpStr5 = tmpStr5 + "@";
  String  tmpStr6 = "";
  String  chekMask = "";
  String tmpStr2 = "";

WDT_Restart (WDT);


  for (int ib = 0; ib < tmpStr5.length() + 1; ib++) {
    tmpStr6 += tmpStr5[ib];
    if (ib == 18) {
      tmpStr6 += " ";
    }

  }

  for (int ic = 0; ic < msg.length() + 1; ic++) {
    chekMask += msg[ic];
    if ((char)msg[ic] == ';') {
      chekMask = chekMask.substring(0, chekMask.length() - 1);
      break;
    }

  }


  char radiopacket[150];
  tmpStr6.toCharArray(radiopacket, tmpStr6.length() + 1);

  serialLOGnoLN("Sending "); serialLOG(radiopacket);

  rf95.send((uint8_t *)radiopacket, sizeof(radiopacket));

  serialLOG("Waiting for packet to complete...");
  delay(3);
  rf95.waitPacketSent();




  serialLOG("Waiting for reply...");
  if (rf95.waitAvailableTimeout(1500))
  {


    // Now wait for a reply
    uint8_t buf2[RH_RF95_MAX_MESSAGE_LEN];
    // uint8_t buf2[9];
    uint8_t len = sizeof(buf2);


    // Should be a reply message for us now
    if (rf95.recv(buf2, &len))
    {
      serialLOGnoLN("Got reply: ");
      serialLOG((char*)buf2);
      serialLOGnoLN("RSSI: ");
      serialLOG(String(rf95.lastRssi(), DEC));

      tmpStr2 = (char*)buf2 ;

      serialLOGnoLN("len: ");
      serialLOG(String(len, DEC));
      serialLOGnoLN("Got reply str: ");
      serialLOG(tmpStr2);
      serialLOGnoLN("chekMask: ");
      serialLOG(chekMask);

    }

    if (((String)chekMask == (String)tmpStr2)&&(String(chekMask[0]) != String("%"))) {
      serialLOG("sendLORA pkt sended ");
      return true;
    }
    //    }
    else
    {
      serialLOG("Receive failed");
      return false;
    }
  }
  else
  {

    //flgSavePulsePorts = true;

    
    serialLOG("No reply, is there a listener around?");
    return false;
  }
}



void nextCHNL() {

WDT_Restart (WDT);
      
//    uint8_t CHNLtmp = random(1,5);
//    if (CHNLtmp != CHNL) {
//      CHNL = CHNLtmp;
//      } else {
//        CHNL = random(1,5);
//        }

 



 for (int i = 0; i < 5; i++) {
  bool flgBreak = false;
    CHNL++;  
    if (CHNL > 5) {
      CHNL = 1;
    }
    
    for(int ii = 0; ii < sizeof(masCH); ii++){
      if (CHNL == masCH[ii]) {
    flgBreak = true;
      break;
    }
      }
  if (flgBreak) {break;}
} 

//FOR TEST
//CHNL = 1;

switch (CHNL) {
    case 1:
        rf95.setSpreadingFactor(7);
        rf95.setSignalBandwidth(500000);
        rf95.setCodingRate4(5);
        if (!rf95.setFrequency(866.0)) {
                serialLOG("setFrequency failed");
                while (1);
              }
        serialLOGnoLN("CHNL: ");      serialLOG(String(CHNL));
      break;
    case 2:
        rf95.setSpreadingFactor(7);
        rf95.setSignalBandwidth(500000);
        rf95.setCodingRate4(5);
        if (!rf95.setFrequency(865.5)) {
                serialLOG("setFrequency failed");
                while (1);
              }
        serialLOGnoLN("CHNL: ");      serialLOG(String(CHNL));
      break;
    case 3:
        rf95.setSpreadingFactor(11);
        rf95.setSignalBandwidth(500000);
        rf95.setCodingRate4(5);
        if (!rf95.setFrequency(866.5)) {
                serialLOG("setFrequency failed");
                while (1);
              }
        serialLOGnoLN("CHNL: ");      serialLOG(String(CHNL));
        break;
     case 4:
        rf95.setSpreadingFactor(11);
        rf95.setSignalBandwidth(125000);
        rf95.setCodingRate4(5);
        if (!rf95.setFrequency(865.07)) {
                serialLOG("setFrequency failed");
                while (1);
              }
        serialLOGnoLN("CHNL: ");      serialLOG(String(CHNL));
        break;   
     case 5:
        rf95.setSpreadingFactor(11);
        rf95.setSignalBandwidth(125000);
        rf95.setCodingRate4(5);
        if (!rf95.setFrequency(866.93)) {
                serialLOG("setFrequency failed");
                while (1);
              }
        serialLOGnoLN("CHNL: ");      serialLOG(String(CHNL));
        break;   
  }

    delay(5);
  
  }



  uint32_t calcDirectory(String dirS) {
  uint32_t sizeFileAll = 0;
   File dir = SD.open(dirS);
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    
   // Serial.print(entry.name());
    uint32_t sizeFile = entry.size();
    sizeFileAll = sizeFileAll + sizeFile;   
    entry.close();
  }

  return sizeFileAll;
}
