#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "radSens1v2.h"

ClimateGuard_RadSens1v2 radSens(RS_DEFAULT_I2C_ADDRESS);

#define VBAT_CORRECTION       1.004     // Edit this for calibrating your battery voltage

uint32_t sequence = 0;

uint32_t prevPulses_0 = 0;
//uint32_t prevPulses_1 = 0;
//uint32_t prevPulses_2 = 0;
//uint32_t prevPulses_3 = 0;

/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

/* OTAA para*/
uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint32_t devAddr =  ( uint32_t )0x00000000;

#if defined( REGION_EU868 )
/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6] = { 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
#else
uint16_t userChannelsMask[6] = { 0xFF00,0x0000,0x0000,0x0000,0x0000,0x0000 };
#endif

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 60000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

/* Prepares the payload of the frame */
bool prepareTxFrame(uint8_t port)
{
  //RadSense Readings
  uint32_t  radstatic, raddynamic, pulses, cpm;
  raddynamic   = radSens.getRadIntensyDyanmic(); 
  radstatic    = radSens.getRadIntensyStatic();
  pulses       = radSens.getNumberOfPulses();

cpm = (pulses - prevPulses_0);
prevPulses_0 = pulses;

  /*
  if (sequence == 0) {
    prevPulses_0 = pulses;
    cpm = (pulses - prevPulses_1);
    sequence = 1;
  }

  else if (sequence == 1) {
    prevPulses_1 = pulses;
    cpm = (pulses - prevPulses_2);
    sequence = 2;
  }

  else if (sequence == 2) {
    prevPulses_2 = pulses;
    cpm = (pulses - prevPulses_3);
    sequence = 3;
  }

  else if (sequence == 3) {
    prevPulses_3 = pulses;
    cpm = (pulses - prevPulses_0);
    sequence = 0;
  }
  */

  //Print Values
  Serial.print("Number of pulses: ");
  Serial.println(pulses);

  Serial.print("Counts Per Minute: ");
  Serial.println(cpm);

  Serial.print("Rad intensity dynamic: ");
  Serial.println(raddynamic);

  Serial.print("Rad intensity static: ");
  Serial.println(radstatic);

  //Battery Voltage
  uint16_t batteryVoltage = ((float_t)((float_t)((float_t)getBatteryVoltage() * VBAT_CORRECTION)  / 10) + .5);

  Serial.print("BatteryVoltage: ");
  Serial.println(batteryVoltage);

  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
    appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
    if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
    if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
    for example, if use REGION_CN470,
    the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */

  //Build Payload
  unsigned char *puc;
  appDataSize = 0;

  puc = (unsigned char *)(&cpm);
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  puc = (unsigned char *)(&radstatic);
  appData[appDataSize++] = puc[2];
  appData[appDataSize++] = puc[1];
  appData[appDataSize++] = puc[0];

  appData[appDataSize++] = (uint8_t)((batteryVoltage-200) & 0xFF);

  return true;
}

///////////////////////////////////////////////////
//Some utilities for going into low power mode
TimerEvent_t sleepTimer;
//Records whether our sleep/low power timer expired
bool sleepTimerExpired;

static void wakeUp()
{
  sleepTimerExpired=true;
}

static void lowPowerSleep(uint32_t sleeptime)
{
  sleepTimerExpired=false;
  TimerInit( &sleepTimer, &wakeUp );
  TimerSetValue( &sleepTimer, sleeptime );
  TimerStart( &sleepTimer );
  //Low power handler also gets interrupted by other timers
  //So wait until our timer had expired
  while (!sleepTimerExpired) lowPowerHandler();
  TimerStop( &sleepTimer );
}

///////////////////////////////////////////////////

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext,LOW);//set vext to high

    radSens.radSens_init();
    radSens.setSensitivity(105);
    radSens.setHVGeneratorState(true);

    uint8_t sensorChipId = radSens.getChipId(); /*Returns chip id, default value: 0x7D.*/

    Serial.print("Chip id: 0x");
    Serial.println(sensorChipId, HEX);

    uint8_t firmWareVer = radSens.getFirmwareVersion(); /*Returns firmware version.*/

    Serial.print("Firmware version: ");
    Serial.println(firmWareVer);
#if(AT_SUPPORT)
    enableAt();
#endif
    deviceState = DEVICE_STATE_INIT;
    LoRaWAN.ifskipjoin();
}

void loop()
{
    switch( deviceState )
    {
        case DEVICE_STATE_INIT:
        {
#if(LORAWAN_DEVEUI_AUTO)
            LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
            getDevParam();
#endif
            printDevParam();
            LoRaWAN.init(loraWanClass,loraWanRegion);
            deviceState = DEVICE_STATE_JOIN;
            break;
        }
        case DEVICE_STATE_JOIN:
        {
            LoRaWAN.join();
            break;
        }
        case DEVICE_STATE_SEND:
        {
            prepareTxFrame( appPort );
            LoRaWAN.send();
            deviceState = DEVICE_STATE_CYCLE;
            break;
        }
        case DEVICE_STATE_CYCLE:
        {
            // Schedule next packet transmission
            txDutyCycleTime = appTxDutyCycle;
            LoRaWAN.cycle(txDutyCycleTime);
            deviceState = DEVICE_STATE_SLEEP;
            lowPowerSleep(txDutyCycleTime);
            break;
        }
        case DEVICE_STATE_SLEEP:
        {
            LoRaWAN.sleep();
            break;
        }
        default:
        {
            deviceState = DEVICE_STATE_INIT;
            break;
        }
    }
}