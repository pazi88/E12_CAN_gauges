
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
#include "STM32_CAN.h"
#include <SwitecX25.h>

//standard X25.168 range 315 degrees at 1/3 degree steps
#define STEPS (315*3)
//this defines how much filtering is applied to the values to avoid needle jumping around. 0 = no filtering, 255 = max filtering.
#define filter_amount 1

static CAN_message_t CAN_outMsg;
static CAN_message_t CAN_inMsg;

static uint32_t RPM_timeout=millis();   // for the RPM timeout
static uint32_t VSS_timeout=millis();   // for the VSS timeout

STM32_CAN Can1( CAN1, DEF, RX_SIZE_64, TX_SIZE_16 );


// For motors connected to digital pins
SwitecX25 VSSGauge(STEPS,PB6,PB7,PB9,PB8);
SwitecX25 RPMGauge(STEPS,PB12,PB13,PB15,PB14);

bool RPM_Request=true;
bool VSS_Request=true;
uint16_t VSS,RPM,RPMsteps,VSSsteps;

#if ((STM32_CORE_VERSION_MINOR<=8) & (STM32_CORE_VERSION_MAJOR==1))
void requestData(HardwareTimer*){void requestData();}
#endif

void requestData()
{
  if(RPM_Request) {
    CAN_outMsg.buf[2]= 0x0C; // PID number for RPM
    Can1.write(CAN_outMsg);
    RPM_Request = false;
    Serial.println ("RPM request");
  }
  
  if(VSS_Request) {
    CAN_outMsg.buf[2]= 0x0D; // PID number for VSS
    Can1.write(CAN_outMsg);
    VSS_Request = false;
    Serial.println ("VSS request");
  }
}

void setup(void)
{
  Serial.begin(115200); // for debugging
  // run the motors against the stops TBD: this needs to be done when powering off. Not at startup
  //VSSGauge.zero();
  //RPMGauge.zero();

  Serial.begin(115200); // debug

  Can1.begin();
  Can1.setBaudRate(500000);
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
  RPMGauge.setPosition(0);
  VSSGauge.setPosition(0);

  // setup hardwaretimer to request obd data in 50Hz pace. Otherwise the obd2 requests can be too fast.
#if defined(TIM1)
  TIM_TypeDef *Instance = TIM1;
#else
  TIM_TypeDef *Instance = TIM2;
#endif
  HardwareTimer *requestTimer = new HardwareTimer(Instance);
  requestTimer->setOverflow(50, HERTZ_FORMAT); // 50 Hz
#if ( STM32_CORE_VERSION_MAJOR < 2 )
  requestTimer->attachInterrupt(1, requestData);
  requestTimer->setMode(1, TIMER_OUTPUT_COMPARE);
#else //2.0 forward
  requestTimer->attachInterrupt(requestData);
#endif
  requestTimer->resume();

  Serial.println("Setup done");
}

#define FILTER(input, alpha, prior) (((long)input * (256 - alpha) + ((long)prior * alpha))) >> 8

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
      Serial.print ("E39/46 RPM: ");
      Serial.println (tempRPM);
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
      Serial.print ("E39/46 VSS: ");
      Serial.println (tempVSS);
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
            //debug:
            Serial.print ("OBD2 RPM: ");
            Serial.println (tempRPM >> 2);
            RPM_timeout = millis();             // zero the timeout
            break;
          case 0x0D: // VSS
            VSS = (CAN_inMsg.buf[3]);
            VSS_Request = true;
            //debug:
            Serial.print ("OBD2 VSS: ");
            Serial.println (VSS);
            VSS = VSS + 2;
            VSS = VSS << 7; // multiply by 128
            Serial.print ("Calculated VSS: ");
            Serial.println (VSS);
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

void loop(void)
{
  if ( (millis()-RPM_timeout) > 500) { // timeout, because no RPM data from CAN bus
    RPM_timeout = millis();
    RPM = 0;
    RPMGauge.setPosition(0);
    RPM_Request = true;
    Serial.println ("RPM timeout");
  }
  if ( (millis()-VSS_timeout) > 500) { // timeout, because no VSS data from CAN bus
    VSS_timeout = millis();
    VSS = 0;
    VSSGauge.setPosition(0);
    VSS_Request = true;
    Serial.println ("VSS timeout");
  }
  // the gauges only moves when update is called
  VSSGauge.update();
  RPMGauge.update();

  while (Can1.read(CAN_inMsg) ) {
    readCanMessage();
  }
}
