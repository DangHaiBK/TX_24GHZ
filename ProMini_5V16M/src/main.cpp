#include <Arduino.h>

#include <Bounce2.h>

#include <SPI.h>
#include "RF24.h"

#include "config/config.h"
#include "config/radioLink.h"

/* Radio class instance */
RF24 radio(CE_PIN, CSN_PIN);

/* Instance a button AUX1, AUX2 object */
Bounce2::Button Aux1 = Bounce2::Button();
Bounce2::Button Aux2 = Bounce2::Button();

/* Constant values for minimum percent and maximum percent for each channel */
// const int maxPercentChannel = 100;
// const int minPercentChannel = -100;

/* State variables */
bool receivedRxToSend;

/* Timestamp variables */
unsigned long last_receive;
unsigned long toggle_period;
unsigned long last_send;

#ifdef BATT_CHECK_STATUS
float batt_voltage;
unsigned long last_check_batt;
#endif  /* BATT_CHECK_STATUS */

/* radiolink protocol variables */
radiolink_protocol_t radio_link_send;
radiolink_protocol_t radio_link_response;

/* channel data struct variables */
dataPacket curData;
dataPacket prevData;
//dataPacket packetData;


#ifdef SERIAL_DEBUG
pseudo_packet_t packet;
void pseudo_send_data()
#endif  /* SERIAL_DEBUG */

bool check_payload_receive(radiolink_protocol_t payload);

void setup() {

#ifdef BATT_CHECK_STATUS
  Serial.begin(9600);
#endif /* BATT_CHECK_STATUS */

#ifdef SERIAL_DEBUG
  Serial.begin(9600);
  packet.pitch = 10;
  packet.roll = 12;
  packet.yaw = 16;
#endif  /* SERIAL_DEBUG */

  // pinMode(AUX1_CHANNEL, INPUT);
  // pinMode(AUX2_CHANNEL, INPUT);

  Aux1.attach(AUX1_CHANNEL, INPUT);
  Aux2.attach(AUX2_CHANNEL, INPUT);

  /* Define debounce interval AUX Channels in milliseconds */
  Aux1.interval(5);
  Aux2.interval(5);

  /* Indicate state corresponding to physically pressing Channels */
  Aux1.setPressedState(LOW);
  Aux2.setPressedState(LOW);

  pinMode(LED_STATUS_TX, OUTPUT);
  pinMode(LED_POWER, OUTPUT);

  /* Setup radio NRF24 */
  if (!radio.begin()) {
#ifdef SERIAL_DEBUG
    Serial.println("Radio module is not connected");
#endif  /* SERIAL_DEBUG */
    digitalWrite(LED_STATUS_TX, LOW);
    for (;;) {
#ifdef SERIAL_DEBUG
      Serial.println("Waiting for connection ...");
#endif  /* SERIAL_DEBUG */
      if (radio.begin()) {
        break;
      }
      delay(1000);
    }
  }

#ifdef SERIAL_DEBUG
  Serial.println("Radio module is connected");
#endif  /* SERIAL_DEBUG */

  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(CHANNEL_NRF24);
  radio.setCRCLength(RF24_CRC_16);

  /* Delay for stabilization */
  delay(1000);

  /* Indicate leds status */
  digitalWrite(LED_STATUS_TX, HIGH);
  digitalWrite(LED_POWER, HIGH);
  
  radio.openWritingPipe(TX_ADDR);
  radio.openReadingPipe(1, TX_ADDR);
  radio.startListening();
  
  /* Delay for stabilization */
  //delay(1000);

#ifdef SERIAL_DEBUG
  Serial.println("Preparing for sending packet ...");
#endif  /* SERIAL_DEBUG */

  /* Assign timestamp */
  toggle_period = millis();
  last_receive = millis();
  last_send = millis();
  last_check_batt = millis();
}

void loop() {

#ifdef SERIAL_DEBUG
    /* Send data to receiver - pseudo */
    pseudo_send_data();
    delay(2000);
#endif /* SERIAL_DEBUG */

  /* Receiver status from receiver */
  if (radio.available()) {
    while(radio.available()) {
      toggle_period = millis();
      radio.read(&radio_link_response, sizeof(radio_link_response));
      if (check_payload_receive(radio_link_response)) {
        last_receive = millis();
        receivedRxToSend = true;
      }
    }
  }

  /* Send data to receiver */
  if (millis() - last_send >= FRAME_RATE) {
    last_send = millis();

    if (receivedRxToSend) {
      /* Switch to sending mode of RF24 */
      radio.stopListening();

      /* Analog value for Throttle channel */
      curData.throttle = analogRead(THROTTLE_CHANNEL);
      curData.throttle = constrain(curData.throttle, VALUE_ADC_MIN, VALUE_ADC_MAX);
      if (abs(curData.throttle - prevData.throttle) >= VALUE_DIFFERENCE) {
        radio_link_send.payload[0] = map(curData.throttle, VALUE_ADC_MIN, VALUE_ADC_MAX, VALUE_STICK_MIN, VALUE_STICK_MAX);
        //packetData.throttle = map(curData.throttle, VALUE_ADC_MIN, VALUE_ADC_MAX, VALUE_STICK_MIN, VALUE_STICK_MAX);
        prevData.throttle = curData.throttle;
      }
      radio_link_send.payload[0] = map(prevData.throttle, VALUE_ADC_MIN, VALUE_ADC_MAX, VALUE_STICK_MIN, VALUE_STICK_MAX);
      
      /* Analog value for Pitch channel */
      curData.pitch = analogRead(PITCH_CHANNEL);
      curData.pitch = constrain(curData.pitch, VALUE_ADC_MIN, VALUE_ADC_MAX);
      if (abs(curData.pitch - prevData.pitch) >= VALUE_DIFFERENCE) {
        radio_link_send.payload[2] = map(curData.pitch, VALUE_ADC_MIN, VALUE_ADC_MAX, VALUE_STICK_MIN, VALUE_STICK_MAX);
        //packetData.pitch = map(curData.pitch, VALUE_ADC_MIN, VALUE_ADC_MAX, VALUE_STICK_MIN, VALUE_STICK_MAX);
        prevData.pitch = curData.pitch;
      }
      radio_link_send.payload[2] = map(prevData.pitch, VALUE_ADC_MIN, VALUE_ADC_MAX, VALUE_STICK_MIN, VALUE_STICK_MAX);

      /* Analog value for Roll channel */
      curData.roll = analogRead(ROLL_CHANNEL);
      curData.roll = constrain(curData.roll, VALUE_ADC_MIN, VALUE_ADC_MAX);
      if (abs(curData.roll - prevData.roll) >= VALUE_DIFFERENCE) {
        radio_link_send.payload[3] = map(curData.roll, VALUE_ADC_MIN, VALUE_ADC_MAX, VALUE_STICK_MIN, VALUE_STICK_MAX);
        //packetData.roll = map(curData.roll, VALUE_ADC_MIN, VALUE_ADC_MAX, VALUE_STICK_MIN, VALUE_STICK_MAX);
        prevData.roll = curData.roll;
      }
      radio_link_send.payload[3] = map(prevData.roll, VALUE_ADC_MIN, VALUE_ADC_MAX, VALUE_STICK_MIN, VALUE_STICK_MAX);

      /* Analog value for Yaw channel */
      curData.yaw = analogRead(YAW_CHANNEL);
      curData.yaw = constrain(curData.yaw, VALUE_ADC_MIN, VALUE_ADC_MAX);
      if (abs(curData.yaw - prevData.yaw) >= VALUE_DIFFERENCE) {
        radio_link_send.payload[1] = map(curData.yaw, VALUE_ADC_MIN, VALUE_ADC_MAX, VALUE_STICK_MIN, VALUE_STICK_MAX);
        //packetData.yaw = map(curData.yaw, VALUE_ADC_MIN, VALUE_ADC_MAX, VALUE_STICK_MIN, VALUE_STICK_MAX);
        prevData.yaw = curData.yaw;
      }
      radio_link_send.payload[1] = map(prevData.yaw, VALUE_ADC_MIN, VALUE_ADC_MAX, VALUE_STICK_MIN, VALUE_STICK_MAX);

      /* Digital values */
      Aux1.update();
      Aux2.update();

      if (Aux1.pressed()) {
        radio_link_send.payload[4] = VALUE_STICK_MAX;
      }
      else {    //released()
        radio_link_send.payload[4] = VALUE_STICK_MIN;
      }

      if (Aux2.pressed()) {
        radio_link_send.payload[5] = VALUE_STICK_MAX;
      }
      else {    //released()
        radio_link_send.payload[5] = VALUE_STICK_MIN;
      }

      // curData.AUX1 = digitalRead(AUX1_CHANNEL);
      // curData.AUX2 = digitalRead(AUX2_CHANNEL);

      // radio_link_send.payload[4] = (curData.AUX1 == 0) ? VALUE_STICK_MAX : VALUE_STICK_MIN;
      // radio_link_send.payload[5] = (curData.AUX2 == 0) ? VALUE_STICK_MAX : VALUE_STICK_MIN;

#ifdef SERIAL_DEBUG
      Serial.println("Stick data: ");
      Serial.print("[THROTTLE]: ");
      Serial.print(curData.throttle);
      Serial.print(" [YAW]: ");
      Serial.print(curData.yaw);
      Serial.print(" [PITCH]: ");
      Serial.print(curData.pitch);
      Serial.print(" [ROLL]: ");
      Serial.print(curData.roll);

      Serial.println();
      Serial.print(" [AUX1]: ");
      Serial.print(radio_link_send.payload[4]);
      Serial.print(" [AUX2]: ");
      Serial.print(radio_link_send.payload[5]);
      Serial.println();
#endif /* SERIAL_DEBUG */
    
      /* Prepare packet and send through RF24 */
      radio_link_send.startByte = RADIOLINK_START_BYTE;
      radio_link_send.length = NUM_CHANNELS;
      radio_link_send.infoByte = RADIOLINK_INFO_STICK_POS;
      radio_link_send.endByte = RADIOLINK_END_BYTE;

      //radio.openWritingPipe(TX_ADDR);
      radio.write(&radio_link_send, sizeof(radio_link_send));

      /* Restart receiver mode to get packet from RX */
      radio.startListening();
    }
  }

  /* Led status when loss signal from receiver */
  if (millis() - last_receive >= SIGNAL_FEEDBACK_TIMEOUT) {
    receivedRxToSend = false;
    if (millis() - toggle_period >= LOSS_TOGGLE_PERIOD) {
      toggle_period = millis();
      digitalWrite(LED_STATUS_TX, !digitalRead(LED_STATUS_TX));
    }
  }
  else {
    digitalWrite(LED_STATUS_TX, HIGH);
  }

#ifdef BATT_CHECK_STATUS 
  /* Check battery voltage */
  if (millis() - last_check_batt >= BATT_CHECK_PERIOD) {
    last_check_batt = millis();
    batt_voltage = analogRead(BATT_VOLT_PIN) * (5.0 / 1023.0);

    if (batt_voltage >= BATT_VOLT_THRES) {
      digitalWrite(LED_POWER, HIGH);
    }
    else {
      digitalWrite(LED_POWER, !digitalRead(LED_POWER));
    }

    Serial.print("[BATT VOLTAGE]: ");
    Serial.print(batt_voltage);
    Serial.println();
  }
#endif /* BATT_CHECK_STATUS */

}

/* Function check payload received from Rx */
bool check_payload_receive(radiolink_protocol_t payload) {
  if ((payload.startByte == RADIOLINK_START_BYTE) && (payload.endByte == RADIOLINK_END_BYTE) && (payload.infoByte == RADIOLINK_STATUS_PACKET_OK)) {
    return true;
  }
  else {
    return false;
  }
}

/* Pseudo data to use Print function */
void pseudo_send_data() {
  static int value;
  value = random(255);

  // packet.pitch += 1;
  // packet.roll += 2;
  // packet.yaw += 3;

  radio.openWritingPipe(TX_ADDR);
  radio.write(&value, sizeof(value));
  //radio.write(&packet, sizeof(packet));
  Serial.println("Writing data: ");
  Serial.println(value);
}