
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
#include <src/STM32_CAN/STM32_CAN.h>
#include <SwitecX25.h>

//standard X25.168 range 315 degrees at 1/3 degree steps
#define STEPS (315*3)
//define the needle ranges TBD: fine tune.
#define RPM_STEPS (250*3)
#define VSS_STEPS (250*3)

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
uint16_t VSS,RPM;

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
  Serial.println("Setup done");
}

void readCanMessage() {
  switch (CAN_inMsg.id)
  {
    case 0x316: // RPM in e39/e46 etc.
      VSS = ((CAN_inMsg.buf[3] << 8) | (CAN_inMsg.buf[2]));
      RPMGauge.setPosition(map(RPM, 0, 220, 0, RPM_STEPS));
      Serial.print ("E39/46 RPM: ");
      Serial.println (RPM);
    break;
    case  0x153: // VSS in e39/e46 etc.
      VSS = ((CAN_inMsg.buf[2] << 8) | (CAN_inMsg.buf[1]));
      VSS = VSS - 252;
      VSS = VSS >> 7; // divide by 128
      VSSGauge.setPosition(map(VSS, 0, 220, 0, VSS_STEPS));
      Serial.print ("E39/46 VSS: ");
      Serial.println (VSS);
    break;
    case  0x7E8: // OBD2 PID response.
      switch (CAN_inMsg.buf[2])
        {
          case 0x0C: // RPM
            RPM = ((CAN_inMsg.buf[3] << 8) | (CAN_inMsg.buf[4]));
            RPM = RPM >> 2;
            RPM_Request = true;
            RPMGauge.setPosition(map(RPM, 0, 220, 0, RPM_STEPS));
            Serial.print ("OBD2 RPM: ");
            Serial.println (RPM);
            break;
          case 0x0D: // VSS
            VSS = CAN_inMsg.buf[3];
            VSS_Request = true;
            VSSGauge.setPosition(map(VSS, 0, 220, 0, VSS_STEPS));
            Serial.print ("OBD2 VSS: ");
            Serial.println (VSS);
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
  // the gauges only moves when update is called
  VSSGauge.update();

  if(RPM_Request){
    CAN_outMsg.buf[2]= 0x0C; // PID number for RPM
    Can1.write(CAN_outMsg);
    RPM_Request = false;
  }
  
  if(VSS_Request){
    CAN_outMsg.buf[2]= 0x0D; // PID number for VSS
    Can1.write(CAN_outMsg);
    VSS_Request = false;
  }

  while (Can1.read(CAN_inMsg) ) 
  {
    readCanMessage();
  }
}
