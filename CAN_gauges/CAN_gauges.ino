#include <src/SwitecX12.h>

const int STEPS = 315 * 12;

SwitecX12 VSSGauge(STEPS, PB8, PB9, 1);
SwitecX12 RPMGauge(STEPS, PB6, PB7, 1);

#if ((STM32_CORE_VERSION_MINOR<=8) & (STM32_CORE_VERSION_MAJOR==1))
void updateSteppers(HardwareTimer*){void updateSteppers();}
#endif

void updateSteppers()
{
  // the gauges only moves when update is called. So we call the update at every loop.
  VSSGauge.update();
  RPMGauge.update();
}

void setup(void)
{
  digitalWrite(PB4, HIGH);
  
  Serial.begin(9600);
  Serial.print("Enter a step position from 0 through ");
  Serial.print(STEPS-1);
  Serial.println(".");
  
  // setup hardwaretimer to request obd data in 50Hz pace. Otherwise the obd2 requests can be too fast. This timer is also used to calculate odometer and trip in 1sec intervals.
  TIM_TypeDef *Instance2 = TIM3;
  HardwareTimer *stepperTimer = new HardwareTimer(Instance2);
  stepperTimer->setOverflow(10000, HERTZ_FORMAT);
#if ( STM32_CORE_VERSION_MAJOR < 2 )
  stepperTimer->attachInterrupt(1, updateSteppers);
  stepperTimer->setMode(1, TIMER_OUTPUT_COMPARE);
#else //2.0 forward
  stepperTimer->attachInterrupt(updateSteppers);
#endif
  stepperTimer->resume();
}

void loop(void)
{
  static int nextPos = 0;
  
  if (Serial.available()) {
    char c = Serial.read();
    if (c==10 || c==13) {
      VSSGauge.setPosition(nextPos);
	  RPMGauge.setPosition(nextPos);
      nextPos = 0;
    } else if (c>='0' && c<='9') {
      nextPos = 10*nextPos + (c-'0');
    }
  }
}
