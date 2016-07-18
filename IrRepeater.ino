unsigned int sampleRate = 38000;
byte volatile currentState = LOW;
const byte interruptPin = 0;


void setup() {
  Serial.begin(115200);
  tcConfigure();
  tcEnable();
  pinMode(interruptPin, INPUT_PULLUP);
  REG_PORT_DIRSET0 = PORT_PA23;   // Set the direction of the port pin PA17 to an output
  attachInterrupt(digitalPinToInterrupt(interruptPin), changeState, CHANGE);
}

void loop() {
  // this is only for debugging purposes!
  delay(100);
  Serial.println(currentState);
}

void micro_delay(unsigned long uSecs) {
  unsigned long start = micros();
  unsigned long endMicros = start + uSecs - 4;
  if (endMicros < start) { // Check if overflow
    while ( micros() > start ) {} // wait until overflow
  }
  while ( micros() < endMicros ) {} // normal wait
}

void changeState() {
  currentState = !currentState;
}

void pulse() {
  REG_PORT_OUTSET0 = PORT_PA23;     // Switch the output to 1 or HIGH
  micro_delay(10);            // wait for a brief moment
  REG_PORT_OUTCLR0 = PORT_PA23;     // switch off again
}

void tcConfigure()
{
  // Enable GCLK for TCC2 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset();

  // Set Timer counter Mode to 16 bits
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

  // Set TC5 mode as match frequency
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;

  TC4->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
  while (tcIsSyncing())
    ;

  sampleRate = SystemCoreClock / (TC4->COUNT16.CC[0].reg + 1);

  // Configure interrupt request
  NVIC_DisableIRQ(TC4_IRQn);
  NVIC_ClearPendingIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 0x00);
  NVIC_EnableIRQ(TC4_IRQn);

  // Enable the TC5 interrupt request
  TC4->COUNT16.INTENSET.bit.MC0 = 1;
  while (tcIsSyncing())
    ;
  Serial.println("TC4 initialized!");
}

void TC4_Handler (void)
{
  pulse();
  TC4->COUNT16.INTFLAG.bit.MC0 = 1;     // Clear interrupt
}


bool tcIsSyncing()
{
  return TC4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void tcEnable()
{
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tcIsSyncing())
    ;
}

void tcReset()
{
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing())
    ;
  while (TC4->COUNT16.CTRLA.bit.SWRST)
    ;
}

