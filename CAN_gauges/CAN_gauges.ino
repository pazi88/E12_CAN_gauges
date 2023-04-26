
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.

//This code is made to work with both AX1201728SG stepper driver or TB6612FNG H-bridge driver. Comment out the one that isn't used to drive the gauge steppers
#define STEPPERDRIVER
//#define HBRIDGE

#include "STM32_CAN.h" //My own STM32 CAN library
#include "U8g2lib.h" //Standard U8G2 library for the two OLEDs
#include <SPI.h> //SPI is used to talk to the screens and the Flash chip

//standard X25.168 range 315 degrees at 1/3 degree full steps
#define STEPS (315*3)

#ifdef HBRIDGE
  #include <src/SwitecX25.h>
#endif

#ifdef STEPPERDRIVER
  #include <src/SwitecX12.h>
  //With the AX1201728SG, we get microstepping, so 1/12 degree micro steps. We need this value in few places
  #define MICROSTEPS (315*12)
#endif

//this defines how much filtering is applied to the values to avoid needle jumping around. 0 = no filtering, 255 = max filtering.
#define filter_amount 5
//low pass filter stolen from speeduino code
#define FILTER(input, alpha, prior) (((long)input * (256 - alpha) + ((long)prior * alpha))) >> 8
// 50 Hz rate to update data from OBD2. This can be adjusted to suit the needs.
#define OBD2UpdateRate 50
// Custom font for the screens
const uint8_t u8g2_Super_Secret_Font_18_r[2345] U8G2_FONT_SECTION("u8g2_Super_Secret_Font_18_r") = 
  "`\0\4\4\5\5\3\5\6\30\31\0\373\22\373\23\373\3\34\6A\11\20 \6\0\20\236\0!\15D"
  "\32\236\360\0\305<\221\352A\0\42\12\7Y\257\60\244~\42\0#'M\26\276$F\230\30abD"
  "\15\31\65d\14\13n$\23#L\310\30\26l\206\10\23#L\214\60\61\302\204\14\2$-\356\262\275"
  "\27X$\252%%\214\20\11BbL\20\22D\302\225\10\227LY\302sA\320\204\71\23f\4\221 "
  "$\232,J\31:\62\0%@u\372\345BP\324\71QCF\11\23\63h\230\230A\342\304\214\21("
  "f\214\300!C\204\232\21$j\220\30\243BF\20\24#f\334\30\61\343\4\211\31&J\314\60Qb"
  "F\11;&\256\230x\0&'Q\26\312d\24\345\20\202c\6\216\31H\202 \322\223F\221\214!B"
  "b\10\31$\204\314\220*C\252\220\233Gf\12'\10\2Y\233\360 \0(\30\7w\241\64f\320\230"
  "Ac\250\31\64\15\35\315\210\14\241\251\6\215\32)\32\6s\241\60f\320\230\61d\246!B\315\234\320"
  "d\314$c\206\20\31\3\0*\25)\65\257#N\220\10!!R\30\42\64B\214\220\61\301\0+\17"
  "\254\31\302%T\256\36<\60%T\256\0,\15\4\231\235\360 \304\10\21#\204\0-\10h\264\256\360"
  " \1.\10\204\30\236\360 \0/\30j\22\252\67N\334d\343&\33\67N\334d\343&\33\67N\334"
  "\70\0\60\34M\26\276c,\215\32*H\215 \65\202\324\261;C\205j\310 \42\212R\235\1\61\14"
  "H\32\276\65\21\221\7\257\346\62\32M\26\276d*\315\22\62$HaEr$\271\312\214\225+H"
  "\360\301\203\4\63\36M\26\276c,\315\22\62$H\215 \65\222\230\271\202'\211\36;\206\250\304\22U"
  "g\0\64\35M\26\276G\260:c\247P\215 \64\204\314\30\42d\210\14\42\362\340A:\222\64\1\65"
  "\33M\26\276\242F\23\222C\247(\244f\5!\242$I\36CD\202\211\252\63\0\66!M\26\276d"
  "\14\221\22\62DF\215 \71\244\314\210#,\312\220 \205+\22d\210\254Iu\6\0\67\23L\26\276"
  "\360\203\201S\16\34Hp \205\3\351\25\0\70!M\26\276c,\315\222A$H\215 \65d\324\20"
  "U\207V\20\42\61\354\62TI\212(B\3\0\71\42M\26\276c\14\221\22B$H\215\30\66b\30"
  "*D%\270)Ar\304\260\21\204\210P\223\314\20\0:\13\244\31\236\360 <\250\7\1;\21$\232"
  "\235\360 <\250\7!F\210\30!\4\0<\24\255\25\302\34t\234\251\63\207\216\221\64y\360\244I\302"
  "\1=\14\14Y\302\360 =d\17\36\30>\25\255\25\302 \226\344\301\223\7\215\216\63u\6\315\261\202"
  "b\1?\30l\26\272%\16\215\212\42\211\20\235\42W\254v\4\251\7E\220F\0@\71s\26\326)"
  "<!\263\201\203\206\216\21SB\222!ID\10\241D\204\220AbJ\214\22Sb\224\30\21\42\6\15"
  "\221b\320\20!#\30\11Q\65&\32A\3\207\261D\5\0A$R\22\312G\274\260\341\243HI\14"
  "\35B\220\10A\62\343\10\21#D\254\221G\3\211\20$B\220\4Q\2B\36O\32\312\300\306\205\13"
  "b$\310\215 \67\202\30\211&M\134\20K\207\263\7\17H\260\1C!q\26\316(\30\35#\67\205"
  "\212\24+A\60\61iZ\27$A\220D\261\42\205\312\264b\207\10\0D\31P\32\316\260\252\215\23R"
  "&\310\221 \227\20G\202\224\11'mV\1E\22N\32\306\360\203\240\64m\321\5Q:}\360\200"
  "\1F\20M\32\302\360AKZ\262\340\5I\372\22\0G\42q\26\322(\30\35#\67\205\212\24#B"
  "\60\61iR\134\65$A\220D\71\42\245\212\274qud\0H\17O\32\322@\16\277{\360\1:\374"
  "\35\1I\10D\32\242\360\7\14J\24L\26\276H\377\341\251S#\6\221 CbI\242\63\0K+"
  "Q\32\316@\256\4\261\42\244\312\20*D\246\24\221b$\312%T\267\314D\251\62\244\10\25\42U\206"
  "\30\31bE\310\225 X\0L\15M\32\302@\222\376\277|\360 \1M)T\32\342`\220a\63\317"
  "\36=\10\64\2\305\240\21(\210\220@Ad\10\222Ipr\6\215\31\64f\320\24BD\10\21\5N"
  "\37O\32\322@\20]\262d\252\26-:A\346\310\230#D\316\220\70C\342\320*\315\322\241#O!"
  "r\26\322(:\35+G\245\212\224#BP%R\234\246$A\260D\71\62\245\312\270b\210\12\0P"
  "\31O\32\306\260\250\211\13b$\310\215 \207\31\11\27M\330$#Ko\1Q$\222\366\321(:\35"
  "+G\245\212\224#BP%R\234\246$AJD\65h\312\234q\345\14\5y\0\42\0R\26O\32"
  "\316\320\304\305\3b\351p\367`D\23\27\304\322\341\303\1S\36o\26\306\27\64\325\232&\244\210\214#"
  "A\326d\272e\12\221\246C\227\214D\33Fi\0T\14O\22\302\360\63\262\364\377\25\0U\22P\32"
  "\322@\20\377\17\223\21!U\304\15\253D\0V&P\22\306@\220\4\71\22\344H\20#C\212\14)"
  "\62\204H\221!E\206\24\21r$\310\221 \207\362J\263U\1W<X\22\342@\214\12R\304H\220"
  "*D\204\220!\42\204\14\221!c\210\14\231\63\203\210\220\30Bj\310$\244\206\314\214\304\20\22\303\320"
  "\234\63t\316\220AC\6M\225$V\222\30)\0X&P\26\312P\254\4\61\62\204\10\25)E\204"
  "\34B\224f\311\232<\211\216D\251\42\244\310\224!U\242\30\11r\5Y\33Q\22\302@P\35\221R"
  "\204\10UC\214H\261\22\4\221\36\65L\232~\7\0Z\17O\26\306\341K\223\365\322d=}\360\1"
  "[\15\7{\245\360 \321\374\377\243\7\6\134\31i\22\252\60l\234\270a\343\206M\67l\234\270a\343"
  "\206M\67l\234\0]\15\7s\245\360\300\14\375\377\233\7\12^\25L\31\303D\220\234\61SCD\15"
  "\31\63\243!\203F\14\33_\7,\260\261\300\0`\11e\360\237@d\314\0a\34\255\25\276\222f\11"
  "\31\22\244F\222B\262\242\320\10R#\10\221 D\202\311\11\2b\32M\32\302\60t^\230YR\5"
  "\251\21\303\356\14\21\211\42%\226\214\60\3\0c\27\255\25\276\203HI\25\204\220\235\244\331\10B$\212"
  "\220Qt\6\0d\31M\26\302:oL\14YQ\244\4!dw\66\202\20\211\42E\326\230\30e\30"
  "\255\25\276s*\15%\243P=x@\222\350\260\21E\310(:\3\0f\22I\22\246T\306\310\21R"
  "dP\240!E\377\15\0g!Mv\301cb\310\12\62%F!\273+\22dJ\60\71\61J\314\320"
  "\21\243H\24!\223\314\14\0h\21L\32\302\60r^\234XQ$\321\251\373+\2i\12C\32\236\220"
  "\354\301\203\4j\17\346r\235Bs\42\364\377\202D\25\202\0k L\32\276\60r\36\221\30Cd\10"
  "\231\21\204N!BD\202\314\30\42c\210\14\42\61\10\25\1l\10C\32\236\360\203\4m&\264\31\336"
  "\60\302\210\221\7$\212\30!A\210\320)B\247F\235\32uj\324\251Q\247F\235\32uj\324\251Q"
  "\4n\17\254\31\302\60\342\304\212\42\211N\335_\21o\27\256\25\302\203JM\221\42\243H\20\303\63\22"
  "\204\210T\243\12\15\0p\33Mz\301\60\302\314\222*H\215\30vg\210H\24)\261d\204\231\241s"
  "\12\0q\31Mv\301cb\310\212\42%\10!\273\263\21\204H\24)\262\306\304\320yr\17\250\31\252"
  "\60\304\304\3\61\204F\315\257\0s\26\254\25\272\202F\5\241\21\204\22\242Jv\360\24\42\22j\220\0"
  "t\21)\22\246B\212\66\17\206\220\242_\235\61T\0u\17\254\31\302\60\352\376\12Q\12\27#\214\14"
  "v\34\255\21\266@\212\4!\22\204\206\220!\63\206\222QCF\215 u\316\272\222\204\0w+\264\25"
  "\326\60\212\20\42B#\310\224!\62\246\14\21\42F\306\220\230d\320\210)\10\215\230\325\21Sf\214\225"
  ")W\210\34)B\0x\31\255\21\272A\210\10\65$H!\63W\260\240\261S$\10\21\241\10\25\1"
  "y#Mr\265@l\304(\22\204\206\20\32\63\206\222ADF\215 u\316\134\301\222$G\22;f"
  "\216 \0z\16\254\25\272\260b\273b\365\335\203\7\6{\26\11s\245E\306\310\21R\364Q\231B\245"
  "\210\221\242\257\316\230\42|\10\3w\231\360\7\14}\32\11s\245@\252\220)b\363\214\24\261BeH"
  "\221\32\66Wd\254!\5\0~\15\254\230\302\42\322L\10\67\6\207\0/q\26\316\360`<\0\361"
  "\0\304\3\20\17@<\0\361\0\304\3\20\17@<\0\361\0\304\3\20\17@<\0\361\0\304\3\20\17"
  "@<\200\7\3\0\0\0";

static CAN_message_t CAN_outMsg;
static CAN_message_t CAN_inMsg;

static uint32_t RPM_timeout=millis();   // for the RPM timeout
static uint32_t VSS_timeout=millis();   // for the VSS timeout
//CAN rx buffer size increased, because it might be needed in busy CAN bus.
STM32_CAN Can1( CAN1, DEF, RX_SIZE_64, TX_SIZE_16 );

#define FLASH_BASEADRESS     0x801D400UL
#define FLASH_ODOADRESS      0x801D404UL

U8G2_SH1122_256X64_1_4W_HW_SPI upper(U8G2_R2, /* cs=*/ PC15, /* dc=*/ PA15, /* reset=*/ PA8); // Screen above the needle
U8G2_SH1122_256X64_1_4W_HW_SPI lower(U8G2_R2, /* cs=*/ PC14, /* dc=*/ PA15, /* reset=*/ U8X8_PIN_NONE); // Screen below the needle. (screens share the reset pin)

// for motors connected to digital pins
#ifdef HBRIDGE
  SwitecX25 VSSGauge(STEPS,PB6,PB7,PB9,PB8);
  SwitecX25 RPMGauge(STEPS,PB12,PB13,PB15,PB14);
#endif

#ifdef STEPPERDRIVER
  SwitecX12 VSSGauge(MICROSTEPS, PB8, PB9, 1);
  SwitecX12 RPMGauge(MICROSTEPS, PB6, PB7, 1);
#endif

// to keep track of if OBD2 requests have been sent.
bool RPM_Request=true;
bool VSS_Request=true;

uint16_t VSS,RPM,RPMsteps,VSSsteps;
// odometer value shown on screen. 1km resolution
uint32_t odometer;
uint32_t odometerOld;
// trip value shown on screen. 100m resolution
uint16_t trip;
uint16_t tripOld;
// more accurate trip and odometer. 1cm resolution.
uint64_t odometerCm;
uint64_t odometerCmStart;
uint32_t tripCm;
uint32_t tripCmStart;
// to keep track of 1sec duration to calculate trip and odometer.
int8_t oneSec;
// for now program memory flash is used to store odometer/trip. It has wear limit, so we try to avoid writing to it as much as possible. So this makes the flash to be written only once on every power up.
bool notCommitted;
// push buttons on the instrument cluster
const int leftButton = PC13;
const int rightButton = PB5;
// the instrument cluster can be kept on using this pin as output to drive home the needles.
const int powerPin = PA0;

#if ((STM32_CORE_VERSION_MINOR<=8) & (STM32_CORE_VERSION_MAJOR==1))
void requestData(HardwareTimer*){void requestData();}
void updateSteppers(HardwareTimer*){void updateSteppers();}
#endif

void updateSteppers()
{
  // the gauges only moves when update is called. So we call the update on this fuction for every 100uS.
  VSSGauge.update();
  RPMGauge.update();
}

// called in 1 second intervals to calculate driven distance.
void calcOdometer()
{
  uint32_t VSScmS = (VSS-256)/4.608; // Looks bit weird but this is the BMW VSS data converted to centimeters per second. This comes from the bmw to km/h ( (/128)-2 ) conversion and the km/h to m/s conversion ( /3.6 )
  odometerCm = odometerCm + VSScmS; // Because this is called in 1sec intervals, we just add the driven distance to the count.
  tripCm = tripCm + VSScmS; // Same for trip.
  odometer = odometerCm / 100000; // Convert centimeters to kilometers for odometer
  trip = tripCm / 10000; // Convert centimeters to 100 meters for trip.
}

void requestData()
{
  if(RPM_Request) {
    CAN_outMsg.buf[2]= 0x0C; // PID number for RPM
    Can1.write(CAN_outMsg);
    RPM_Request = false;
  }
  
  if(VSS_Request) {
    CAN_outMsg.buf[2]= 0x0D; // PID number for VSS
    Can1.write(CAN_outMsg);
    VSS_Request = false;
  }
  oneSec++;
  // check if one second has passed
  if ( oneSec >= OBD2UpdateRate ) {
    calcOdometer();
	oneSec = 0;
  }
}

void read_trip( uint32_t *data ) {
  uint32_t* base_addr = (uint32_t *) FLASH_BASEADRESS; 
  *(data) = *(base_addr);
}

void read_odometer( uint64_t *data ) {
  uint64_t* base_addr = (uint64_t *) FLASH_ODOADRESS;
  *(data) = *(base_addr);
}

void setup(void)
{
  // initialize screens
  upper.begin();
  lower.begin();
  read_odometer(&odometerCm);
  odometerCmStart = odometerCm;
  odometer = odometerCm / 100000;
  odometerOld = odometer;
  read_trip(&tripCm);
  tripCmStart = tripCm;
  trip = tripCm / 10000;
  tripOld = trip;
  Serial.begin(115200); // for debugging
  // Init CAN
  Can1.begin();
  Can1.setBaudRate(500000);
  // Filter out unwanted CAN messages.
  Can1.setMBFilterProcessing( MB0, 0x316, 0x1FFFFFFF );
  Can1.setMBFilterProcessing( MB1, 0x153, 0x1FFFFFFF );
  Can1.setMBFilterProcessing( MB2, 0x7E8, 0x1FFFFFFF );
  CAN_outMsg.len = 8; // 8 bytes in can message
  CAN_inMsg.len = 8;
  CAN_outMsg.id = 0x7df; // OBD-II PID_REQUEST
  CAN_outMsg.buf[0]= 0x02; // Sending 2 bytes
  CAN_outMsg.buf[1]= 0x01; // Mode 01 "Show current data"
  CAN_outMsg.buf[2]= 0x00; // set to zero just in case.
  CAN_outMsg.buf[3]= 0x00; // set to zero just in case.
  CAN_outMsg.buf[4]= 0x00; // set to zero just in case.
  CAN_outMsg.buf[5]= 0x00; // set to zero just in case.
  CAN_outMsg.buf[6]= 0x00; // set to zero just in case.
  CAN_outMsg.buf[7]= 0x00; // set to zero just in case.
  
  VSS = 0;
  RPM = 0;
  RPMsteps = 0;
  VSSsteps = 0;
  oneSec = 0;
  notCommitted = true;
  RPMGauge.setPosition(0);
  VSSGauge.setPosition(0);

  // setup hardwaretimer to request obd data in 50Hz pace. Otherwise the obd2 requests can be too fast. This timer is also used to calculate odometer and trip in 1sec intervals.
#if defined(TIM1)
  TIM_TypeDef *Instance1 = TIM1;
#else
  TIM_TypeDef *Instance1 = TIM2;
#endif
  TIM_TypeDef *Instance2 = TIM3;
  HardwareTimer *requestTimer = new HardwareTimer(Instance1);
  HardwareTimer *stepperTimer = new HardwareTimer(Instance2);
  requestTimer->setOverflow(OBD2UpdateRate, HERTZ_FORMAT);
  stepperTimer->setOverflow(10000, HERTZ_FORMAT);
#if ( STM32_CORE_VERSION_MAJOR < 2 )
  requestTimer->attachInterrupt(1, requestData);
  requestTimer->setMode(1, TIMER_OUTPUT_COMPARE);
  stepperTimer->attachInterrupt(1, updateSteppers);
  stepperTimer->setMode(1, TIMER_OUTPUT_COMPARE);
#else //2.0 forward
  requestTimer->attachInterrupt(requestData);
  stepperTimer->attachInterrupt(updateSteppers);
#endif
  requestTimer->resume();
  stepperTimer->resume();

  // put the odometer and trip values from memory to the screens.
  updateOdometer();
  updateTrip();
  // set the basic inputs/outputs
  pinMode(leftButton, INPUT);
  pinMode(rightButton, INPUT);
  pinMode(powerPin, OUTPUT);
  // the power pin is set to high to keep the instrument cluster powered on when ignition is turned off. This code will shut down the cluster once needles are at zero.
  digitalWrite(powerPin, HIGH);
  Serial.println("Setup done");
}

void updateOdometer()
{
  upper.firstPage();
  // we put the odometer reading to the array so that the numbers are in same location, no matter what the odometer reading is.
  int8_t odometer_array[7];
  int8_t j = 0;
  for(uint32_t i = 1000000; i > 0; i /= 10) {
    odometer_array[j] = int8_t((odometer/i) % 10 );
    j++;
  }
  // update the values from the array to the screen
  do {
    upper.setFont(u8g2_Super_Secret_Font_18_r);
    bool first_digit = false;
    for (int i=0; i<7; i++)
    {
      if ( (odometer_array[i] > 0) || first_digit || (i == 6) ) // do not draw the leading zeros
      {
        first_digit = true;
        upper.setCursor((65+((i-1)*20)), 38);
        upper.print(odometer_array[i]);
      }
    }
  } while ( upper.nextPage() );
  // set the old value to same as the current so that we can keep track when this changes.
  odometerOld = odometer;
}

void updateTrip()
{
  lower.firstPage();
  // we put the trip reading to the array so that the numbers are in same location, no matter what the trip reading is.
  int8_t trip_array[4];
  int8_t j = 0;
  for(uint16_t i = 1000; i > 0; i /= 10) {
    trip_array[j] = int8_t((trip/i) % 10 );
    j++;
  }
  // update the values from the array to the screen
  do {
    lower.setFont(u8g2_Super_Secret_Font_18_r);
    bool first_digit = false;
    for (int i=0; i<4; i++)
    {
      if ( (trip_array[i] > 0) || first_digit || (i == 3) ) // do not draw the leading zeros
      {
        first_digit = true;
        lower.setCursor((113+((i-1)*20)), 42);
        lower.print(trip_array[i]);
      }
    }
  } while ( lower.nextPage() );
  // set the old value to same as the current so that we can keep track when this changes.
  tripOld = trip;
}

void CalcRPMgaugeSteps()
{
  uint16_t tempRPMsteps = 0;
  // RPM gauge face is not linear. Below 1000 RPM has different scale than above it.
  if ( RPM < 6400 ) // < 1000rpm
  {
    tempRPMsteps = map(RPM, 0, 6400, 0, 50);
  }
  else if ( RPM < 63616 ) // >= 1000rpm
  {
    tempRPMsteps = map(RPM, 6400, 63616, 50, STEPS);
  }
  else if ( RPM >= 63616 )// limit to max steps
  {
    tempRPMsteps = STEPS;
  }
#ifdef STEPPERDRIVER
  tempRPMsteps = tempRPMsteps * 4;
#endif
  // low pass filter the step value to prevent the needle from jumping.
  RPMsteps = FILTER(tempRPMsteps, filter_amount, RPMsteps);
  RPMGauge.setPosition(RPMsteps);
}

void CalcVSSgaugeSteps()
{
  uint16_t tempVSSsteps = 0;
  // VSS gauge face is even less linear. This looks horrible, but it works.
  if ( VSS < 2816 ) // < 20km/h, Minimum is 20km/h. Less than that we set gauge at zero steps.
  {
    tempVSSsteps = 0;
  } 
  else if ( VSS < 5376 ) // < 40km/h
  {
    tempVSSsteps = map(VSS, 2816, 5376, 0, 76);
  }
  else if ( VSS < 7936 ) // < 60km/h
  {
    tempVSSsteps = map(VSS, 5376, 7936, 76, 150);
  }
  else if ( VSS < 10496 ) // < 80km/h
  {
    tempVSSsteps = map(VSS, 7936, 10496, 150, 221);
  }
  else if ( VSS < 13056 ) // < 100km/h
  {
    tempVSSsteps = map(VSS, 10496, 13056, 221, 296);
  }
  else if ( VSS < 15616 ) // < 120km/h
  {
    tempVSSsteps = map(VSS, 13056, 15616, 296, 370);
  }
  else if ( VSS < 18176 ) // < 140km/h
  {
    tempVSSsteps = map(VSS, 15616, 18176, 370, 443);
  }
  else if ( VSS < 20736 ) // < 160km/h
  {
    tempVSSsteps = map(VSS, 18176, 20736, 443, 517);
  }
  else if ( VSS < 23296 ) // < 180km/h
  {
    tempVSSsteps = map(VSS, 20736, 23296, 517, 592);
  }
  else if ( VSS < 25856 ) // < 200km/h
  {
    tempVSSsteps = map(VSS, 23296, 25856, 592, 670);
  }
  else if ( VSS < 35200 ) // >= 200km/h
  {
    tempVSSsteps = map(VSS, 25856, 35200, 670, STEPS);
  }
  else // limit to max steps
  {
    tempVSSsteps = STEPS;
  }
#ifdef STEPPERDRIVER
   VSSsteps =  VSSsteps * 4;
#endif
  // low pass filter the step value to prevent the needle from jumping.
  VSSsteps = FILTER(tempVSSsteps, filter_amount, VSSsteps);
  VSSGauge.setPosition(VSSsteps);
}

void readCanMessage()
{
  switch (CAN_inMsg.id)
  {
    case 0x316: // RPM in e39/e46 etc.
      RPM = ((CAN_inMsg.buf[3] << 8) | (CAN_inMsg.buf[2]));
      CalcRPMgaugeSteps();
      //debug:
      uint32_t tempRPM;
      tempRPM = RPM;
      // convert the e39/e46 RPM data to real RPM reading
      tempRPM = (tempRPM * 10) / 64;
      RPM_timeout = millis();             // zero the timeout
    break;
    case  0x153: // VSS in e39/e46 etc.
      VSS = ((CAN_inMsg.buf[2] << 8) | (CAN_inMsg.buf[1]));
      CalcVSSgaugeSteps();
      //debug:
      uint32_t tempVSS;
      tempVSS = VSS;
      // convert the e39/e46 VSS data to real VSS reading
      tempVSS = tempVSS >> 7; // divide by 128
      tempVSS = tempVSS - 2;
      VSS_timeout = millis();             // zero the timeout
    break;
    case  0x7E8: // OBD2 PID response.
      switch (CAN_inMsg.buf[2])
        {
          case 0x0C: // RPM
            uint32_t tempRPM;
            tempRPM = ((CAN_inMsg.buf[3] << 8) | (CAN_inMsg.buf[4]));
            RPM = (tempRPM * 16) / 10;
            RPM_Request = true;
            CalcRPMgaugeSteps();
            RPM_timeout = millis();             // zero the timeout
            break;
          case 0x0D: // VSS
            VSS = (CAN_inMsg.buf[3]);
            VSS_Request = true;
            VSS = VSS + 2;
            VSS = VSS << 7; // multiply by 128
            CalcVSSgaugeSteps();
            VSS_timeout = millis();             // zero the timeout
          break;
          default:
          // nothing to do here
          break;
        }
    break;
    default:
      // nothing to do here
    break;
  }
}

void clusterShutdown()
{
  // Set the needles back to zero
  RPMGauge.setPosition(0);
  VSSGauge.setPosition(0);
  if (notCommitted) {
	notCommitted = false;
    if ( (tripCm != tripCmStart) || (odometerCm != odometerCmStart) ) {
	  writeDataToFlash();
    }
  }
  // wait until zero
  while ( (RPMGauge.currentStep != 0) && (RPMGauge.currentStep != 0) ) {
    //Serial.println(RPMGauge.currentStep);
  }
  // turn off the cluster
  digitalWrite(powerPin, LOW);
  Serial.println("Shutdown completed");
}

void writeDataToFlash()
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t pageError = 0;
  
  /* ERASING page */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks = 1;
  EraseInitStruct.PageAddress = FLASH_BASEADRESS;
  EraseInitStruct.NbPages = 1;

  //Clear any flash errors before try writing to flash to prevent write failures.
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR) != RESET) __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR) != RESET) __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGERR);

  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &pageError) == HAL_OK){Serial.println("Flash page erased");}
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_BASEADRESS, tripCm) == HAL_OK) {Serial.println("Trip written");}
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_ODOADRESS, odometerCm) == HAL_OK) {Serial.println("Odometer written");}
  HAL_FLASH_Lock();
}

void loop(void)
{
  // check if the pushbuttons are pressed.
  if (digitalRead(leftButton) == LOW) {
    // zero out the trip
    trip = 0;
	tripCm = 0;
  }

  if ( (millis()-RPM_timeout) > 500) { // timeout, because no RPM data from CAN bus
    RPM_timeout = millis();
    RPM = 0;
    RPMGauge.setPosition(0);
    RPM_Request = true;
    Serial.println ("RPM timeout");
	// no RPM data, so we assume that the car has shut down. So proceed to shut down the cluster too.
	Serial.println ("Shutdown started");
	clusterShutdown();
  }
  if ( (millis()-VSS_timeout) > 500) { // timeout, because no VSS data from CAN bus
    VSS_timeout = millis();
    VSS = 0;
    VSSGauge.setPosition(0);
    VSS_Request = true;
    Serial.println ("VSS timeout");
  }
  
  // update the screens if the odometer and trip values have changed, because those are dead slow to update
  if ( odometer > odometerOld ) {
    updateOdometer();
  }
  if ( trip != tripOld ) {
    updateTrip();
  }

  // see if there is messages available on can bus to read.
  while (Can1.read(CAN_inMsg) ) {
    readCanMessage();
  }
}
