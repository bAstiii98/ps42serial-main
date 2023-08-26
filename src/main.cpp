#include <Arduino.h>
#include <PS4Controller.h>
#include <SoftwareSerial.h>
#include <RuipuBattery.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>

#define MAC "7c:38:ad:4c:3f:45"
#define DEADZONE 13
#define led 2
WebServer server(80);

extern void setupB();
extern void loopB();
extern void Send(int16_t uSteer, int16_t uSpeed);

SoftwareSerial BMSSerial(33, 32); // RX (green wire), TX (blue wire)
RuipuBattery BMS(&BMSSerial);

uint8_t soc;
float voltage;
float current;
float low;
float high;
uint8_t maxTemp;
uint8_t minTemp;

#define PIN        6 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 16 // Popular NeoPixel ring size
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


void handleWeb()
{
  server.sendHeader("Connection", "close");

  String buf;
  buf += "Bat status:";
  buf += "\nSoc:     " + String(soc) + " %";
  buf += "\nVoltage: " + String(voltage) + " V";
  buf += "\nCurrent: " + String(current) + " A";
  buf += "\nLow:     " + String(low) + " V";
  buf += "\nHigh:    " + String(high) + " V";
  buf += "\nmaxTemp: " + String(maxTemp) + " °C";
  buf += "\nminTemp: " + String(minTemp) + " °C";

  server.send(200, "text/plain", buf);
}

void setup()
{
  pixels.begin(); 
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  Serial.println("BMS");
  BMSSerial.begin(9600);

  WiFi.softAP("Fahrende Bombe (ACHTUNG)", "immernochbesseralspeter");

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // delay(1000);
  Serial.println("Hi!");

  Serial.println("PS4.begin");
  PS4.begin(MAC);
  Serial.println("Initialization ready!");

  Serial.println("Hovercomm");
  setupB();

  Serial.println("Ready!");

  server.on("/", handleWeb);

  server.begin();
}

bool connected;

time_t unlockTimeout = 0;
time_t infoTimeout = 0;

bool haveReadData = false;
bool DR =0;


void loop()
{
  DR = PS4.Triangle();
  if (DR >0)
  {
    pixels.setPixelColor(NUMPIXELS, pixels.Color(255, 255, 255));
    digitalWrite(led, HIGH);
  }else{

    pixels.setPixelColor(NUMPIXELS, pixels.Color(0, 0, 0));
    digitalWrite(led, LOW);
  }
  
  pixels.clear();

  server.handleClient();

  time_t now = millis();

  // Do BMS stuffs
  if (now > unlockTimeout)
  {
   // Serial.println("unlock");
    BMS.unlock();         // Send the unlock command
    haveReadData = false; // Reset the read flag

    unlockTimeout = now + 1000;
  }

  if (BMS.read() && !haveReadData)
  {
    haveReadData = true; // Set the read flag

    soc = BMS.soc();
    voltage = BMS.voltage();
    current = BMS.current();
    low = BMS.low();
    high = BMS.high();
    maxTemp = BMS.maxTemp();
    minTemp = BMS.minTemp();

    // Serial plotter friendly outputs
    Serial.print("State of Charge:");
    Serial.print(BMS.soc());
    Serial.print(", ");
    Serial.print("Voltage:");
    Serial.print(BMS.voltage());
    Serial.print(", ");
    Serial.print("Current:");
    Serial.print(BMS.current());
    Serial.print(", ");
    Serial.print("Lowest:");
    Serial.print(BMS.low());
    Serial.print(", ");
    Serial.print("Highest:");
    Serial.print(BMS.high());
    Serial.print(", ");
    Serial.print("Max Temp:");
    Serial.print(BMS.maxTemp());
    Serial.print(", ");
    Serial.print("Min Temp:");
    Serial.print(BMS.minTemp());
    Serial.print(", ");
    Serial.println();
  }

  bool curstate = PS4.isConnected();

  if (!curstate)
  {
    if (now > infoTimeout)
    {
      Serial.println("Controller not connected");
      infoTimeout = now + 1000;
    }

    connected = false;

    return;
  }

  if (!connected)
  {
    Serial.println("Connected!");
    connected = true;
  }

  int8_t lstickX = PS4.LStickX();
  int8_t lstickY = PS4.LStickY();

  int8_t rstickX = PS4.RStickX();
  int8_t rstickY = PS4.RStickY();

  // int8_t dir = rstickX;
  int8_t dir = lstickX;
  if (abs(dir) < DEADZONE)
  {
    dir = 0;
  }

  int8_t speed = lstickY;
  if (abs(speed) < DEADZONE)
  {
    speed = 0;
  }

  int dirI = map(dir, -127, 128, -1000, 1000);
  int maxspeed = 700;

  if (PS4.L2() >= 0.1)
  {
    maxspeed += float(1000 - maxspeed) * float(PS4.L2Value()) / 255;
  }

  if (PS4.R2() >= 0.1)
  {
    maxspeed += float(1000 - maxspeed) * float(PS4.R2Value()) / 255;
  }

  int speedI = -map(speed, -127, 128, -maxspeed, maxspeed);

  speedI = pow(float(speedI) / 1000, 3) * 1000;
  dirI = pow(float(dirI) / 1000, 3) * 1000;

  Send(dirI, speedI);

  delay(25);

  Serial.printf("L(%4d|%4d) R(%4d|%4d) -> speed %4d -> %6d dir %4d boost %d %4d\n",
                lstickX, lstickY,
                rstickX, rstickY,
                speed, speedI, dir,
                PS4.L2(), PS4.L2Value());
}

#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD 115200       // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND 100            // [ms] Sending time interval
#define SPEED_MAX_TEST 300       // [-] Maximum speed for testing
#define SPEED_STEP 20            // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

SoftwareSerial HoverSerial(21, 22); // RX, TX

// Global variables
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct
{
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct
{
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setupB()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  HoverSerial.begin(HOVER_SERIAL_BAUD);
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *)&Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
void Receive()
{
  // Check for new data availability in the Serial buffer
  if (HoverSerial.available())
  {
    incomingByte = HoverSerial.read();                                  // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame
  }
  else
  {
    return;
  }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME)
  { // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  }
  else if (idx >= 2 && idx < sizeof(SerialFeedback))
  { // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback))
  {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
    {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      Serial.print("1: ");
      Serial.print(Feedback.cmd1);
      Serial.print(" 2: ");
      Serial.print(Feedback.cmd2);
      Serial.print(" 3: ");
      Serial.print(Feedback.speedR_meas);
      Serial.print(" 4: ");
      Serial.print(Feedback.speedL_meas);
      Serial.print(" 5: ");
      Serial.print(Feedback.batVoltage);
      Serial.print(" 6: ");
      Serial.print(Feedback.boardTemp);
      Serial.print(" 7: ");
      Serial.println(Feedback.cmdLed);
    }
    else
    {
      Serial.println("Non-valid data skipped");
    }
    idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################
/**/