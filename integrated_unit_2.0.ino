#include <VirtualWire.h>
extern volatile unsigned long timer0_millis;

/*---CHANGE UNIT_ID FOR DIFFERENT UNITS--*/
#define UNIT_ID 1
/*---CHANGE UNIT_ID FOR DIFFERENT UNITS--*/

///// test leg  //////
#define TEST_LEG 13
///// test leg  //////

#define VCC_SONAR_TX 2
#define TX_DATA 3
#define RX_DATA 4
#define VCC_RX 5
#define TRIG_PIN1 6
#define ECHO_PIN1 7
#define TRIG_PIN2 8
#define ECHO_PIN2 9
#define TRIG_PIN3 10
#define ECHO_PIN3 11
#define TRIG_PIN4 12
#define ECHO_PIN4 A0
#define PIR_DATA A2
#define TX_BPS 2000

#define SONAR_DELAY_TIME_MS 30
#define MAX_MICROS 29100
#define MIN_SONAR_DIST 20
#define MAX_SONAR_DIST 300

//we dont use #define here because when using long numbers it makes integer overflows
const int NUM_SONARS = 4;
const int NUM_UNITS = 2;
const int SAMPLE_NUM = 120; //a bit less than the maximum number of messages the arduino can save
const long UNIT_WAITING_TIME_SEC = 5;
const long MEASURING_TIME_SEC = 5;
const long MS_IN_SEC = 1000;
const long start_send_time = (MEASURING_TIME_SEC + UNIT_WAITING_TIME_SEC*UNIT_ID) * MS_IN_SEC;
const long end_send_time = start_send_time + UNIT_WAITING_TIME_SEC * MS_IN_SEC;
const long event_time = (MEASURING_TIME_SEC + UNIT_WAITING_TIME_SEC* NUM_UNITS) * MS_IN_SEC;


const int RESET_INTERVAL_SEC = 10;
float UPDATE_MARGIN = RESET_INTERVAL_SEC;
int delay_time = 0;

int distance[NUM_SONARS];//4 sonar distances array
const int ports[NUM_SONARS * 2] = {TRIG_PIN1, ECHO_PIN1, TRIG_PIN2, ECHO_PIN2, TRIG_PIN3, ECHO_PIN3, TRIG_PIN4, ECHO_PIN4};

unsigned char sonar_id_list[SAMPLE_NUM];
unsigned int distance_list[SAMPLE_NUM];
unsigned long time_list[SAMPLE_NUM];
int lists_index = 1; //count how many messages are saved in the lists %%starts from 1 so it would'nt intrupt the update list check%%
bool lists_sent = false; //check whether we sent the lists alreay in the sending time
unsigned long t_correction = 0;
void setup()
{
  Serial.begin(9600);	  // Debugging only

  // Initialise the 434MHz trasmitter
  vw_set_ptt_inverted(true);// Required for DR3100
  vw_setup(TX_BPS);	// Bits per sec
  vw_set_tx_pin(TX_DATA);
  vw_set_rx_pin(RX_DATA);
  vw_rx_start();

  // Initialize sonar pins
  for (int i = 0; i < NUM_SONARS; i++)
  {
    pinMode(ports[2 * i], OUTPUT);
    pinMode(ports[2 * i + 1], INPUT);
  }

  // set vcc for sonars and trasmitter using pin VCC_SONAR_TX
  pinMode(VCC_SONAR_TX, OUTPUT);
  pinMode(VCC_RX, OUTPUT);
  pinMode(TEST_LEG, OUTPUT);
  pinMode(PIR_DATA, INPUT);
  digitalWrite(VCC_RX, LOW);
  digitalWrite(VCC_SONAR_TX, LOW);


  sonar_id_list[0] = 30; //set a default value so th update will be valid
  distance_list[0] = 1000;
  time_list[0] = 0;

}


//dist in cm, sonar_id from 1 to 4
String format_msg(int sonar_id, int dist, unsigned long time_millis)  {
  String colon = ":";
  String dot = ".";

  String msg = colon;
  msg += String(sonar_id);
  msg += dot;
  msg += String(dist);
  msg += dot;
  msg += String(time_millis);
  msg += colon;

  return msg;
}

void four_pulseIn()
{
  long initial_t, final_t;
  boolean time_exceeded = false;

  //FIRST
  digitalWrite(TRIG_PIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN1, LOW);
  //SECOND
  digitalWrite(TRIG_PIN2, LOW);  //OH Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(TRIG_PIN2, HIGH); //OH
  delayMicroseconds(10); // Added this line
  digitalWrite(TRIG_PIN2, LOW);//OH */
  //THIRD
  digitalWrite(TRIG_PIN3, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN3, LOW);
  //FOURTH
  digitalWrite(TRIG_PIN4, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN4, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN4, LOW);

  //  durationThirdSonar = pulseIn(echoPinThirdSonar, HIGH,15000); //Uncomment if you want to use pulsein.
  delay(1); //It takes the sensor 1 millisecond or less to send the initial pulse. //TODO: CHECK HOW TO DELAY WITH LESS THAN A MILLIS.
  initial_t =  micros();
  //TODO: THINK ABOUT BETTER LOGICS THAN && && &&...
  while (digitalRead(ECHO_PIN1) == HIGH && digitalRead(ECHO_PIN2) == HIGH && digitalRead(ECHO_PIN3) == HIGH && digitalRead(ECHO_PIN4) == HIGH) {
    //RECOGNITION
    if (micros() - initial_t > MAX_MICROS) { //If it's further than 500 cm
      time_exceeded = true;
      break;
    }
  }
  final_t = micros();

  //SONAR ID RECOGNITION
  int sonar_id = -1;
  if (digitalRead(ECHO_PIN1) != HIGH)
    sonar_id = 0;
  else if (digitalRead(ECHO_PIN2) != HIGH)
    sonar_id = 1;
  else if (digitalRead(ECHO_PIN3) != HIGH)
    sonar_id = 2;
  else if (digitalRead(ECHO_PIN4) != HIGH)
    sonar_id = 3;

  //DISTANCE
  long duration = final_t - initial_t;
  int distance = -1;
  if (!time_exceeded)
    distance = (duration / 2) / 29.1;

  //SAVE TO LISTS ONLY CORRECT DIST
  if (distance < MAX_SONAR_DIST && distance > MIN_SONAR_DIST) //%%update only values in the defined range%%
  {
    lists_update(sonar_id, distance, millis());
  }

  //SEND FORMATTED MESSAGE
  //send_formatted_distance_message(sonar_id, distance);
}

//sonar_id from 0 to 3
void send_formatted_distance_message(int sonar_id, int distance, unsigned long time_millis)
{
  String msg_str = format_msg(sonar_id, distance, time_millis);
  const int len = msg_str.length() + 1;
  char msg[len];
  msg_str.toCharArray(msg, len);
  vw_send((uint8_t *)msg, strlen(msg));//send message
  vw_wait_tx(); // Wait until the whole message is gone
  Serial.println(msg_str);
}

void lists_update(unsigned char sonar_id, unsigned int distance, unsigned long t)
{
  //%% with logic that similiar values will be saved once%% TO DO: check if its okay
  if ( distance != -1 && lists_index <= SAMPLE_NUM )
    //&& (!( (abs(time_list[lists_index - 1] - t) < 500) && (sonar_id_list[lists_index - 1] = sonar_id) && (abs(distance_list[lists_index - 1] - distance) < 1000) )))
  {
    sonar_id_list[lists_index] = UNIT_ID * NUM_SONARS + sonar_id;
    distance_list[lists_index] = distance;
    time_list[lists_index] = t;

    lists_index++;
  };
}



void send_lists()
{

  for (int i = 1; i < lists_index; i++) //starts at 1 so we dont send the made up first data
    send_formatted_distance_message(sonar_id_list[i], distance_list[i], time_list[i]);
}

void reset_lists() {//starts at 1 so we dont send the made up first data
  for (int i = 1; i < SAMPLE_NUM; i++)
  {
    sonar_id_list[i] = 0;
    distance_list[i] = 0;
    time_list[i] = 0;
  }
}

void reset_millis() {
  //count_reset++;
  noInterrupts ();
  timer0_millis = 0;
  interrupts ();
  UPDATE_MARGIN = 1;
  digitalWrite(VCC_RX, LOW);
}

bool recieved_sync()
{
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  buf[0] = '\0';
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  if (vw_get_message(buf, &buflen))
  {
    if (buf[0] == '0')
    {
      return true;
    }
  }
  return false;
}
bool trust_pir = true;
int pir_valid_counter = 0; // because sometimes the pir data gets 1
void loop()
{
  if (millis()%1000<5)
  {
    send_formatted_distance_message(0,0,0);
  }
  //long start=millis();
  if (trust_pir)
  {
    if ( digitalRead(PIR_DATA) == 1)
    {
      pir_valid_counter++;
      // start mesuarung if pir read and stop trust pir and take time
      if (pir_valid_counter > 100)
      {
        Serial.println("pir read");
        digitalWrite(VCC_SONAR_TX, HIGH);
        trust_pir = false;
        Serial.println("pir false");
        t_correction = millis();
        pir_valid_counter=0;
      }
    }
    else
      //if the pir soesnt read, update your watch
    {
      if ((millis() % (RESET_INTERVAL_SEC * MS_IN_SEC) > long((RESET_INTERVAL_SEC - UPDATE_MARGIN)*MS_IN_SEC) || (millis() % (RESET_INTERVAL_SEC * MS_IN_SEC) < long(UPDATE_MARGIN * MS_IN_SEC))))
        // in the update margain
      {
        digitalWrite(VCC_RX, HIGH);
        if (recieved_sync())
        {
          Serial.println("something recieved");
          delay_time = 0;
          reset_millis();
          delay(50);
        }

        else if ((millis() % (RESET_INTERVAL_SEC * MS_IN_SEC) > 0.9 * UPDATE_MARGIN * MS_IN_SEC) && (millis()  > RESET_INTERVAL_SEC * MS_IN_SEC) )
        {
          // if time exceeded
          //0.9 so we get that it wouldn't pass that
          Serial.println("time exceeded");
          delay_time = millis() - RESET_INTERVAL_SEC * MS_IN_SEC;
          Serial.println(delay_time);
          reset_millis();
        }
      }
    }
  }
  else
    // i dont trust the pir (because i read from him) i start the event
  {
    
    if ( (millis() - t_correction) < MEASURING_TIME_SEC * MS_IN_SEC ) //sonar measuring time
    {
      Serial.println("measuring!");
      if ( (millis() + delay_time) % (SONAR_DELAY_TIME_MS * NUM_UNITS) > SONAR_DELAY_TIME_MS * UNIT_ID && (millis() + delay_time) % (SONAR_DELAY_TIME_MS * NUM_UNITS) < SONAR_DELAY_TIME_MS * (UNIT_ID + 1) )
        //send in your own time
      {
        four_pulseIn();//measure distance with one simultaneous pulse and also updates the lists
        //delay(SONAR_DELAY_TIME_MS);
      }
    }
    if ((millis() - t_correction) > start_send_time && (millis() - t_correction) < end_send_time )
    {
      //send time
      send_lists();
      reset_lists();
      lists_index = 1;
    }
    if  ((millis() - t_correction) > event_time)
    {
      //stop the event
      trust_pir = true;
      digitalWrite(VCC_SONAR_TX, LOW);
    }
  }
  //Serial.println("it took");
  //Serial.println(millis()-start);
}


