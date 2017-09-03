#define g_sHHMMfmt "%02d:%02d"

// OnboardDisplay.m_bFlags
#define OBDF_MONO_BACKLIGHT 0x01
#define OBDF_AMMETER_DIRTY  0x80
#define OBDF_UPDATE_DISABLED 0x40

// OnboardDisplay::Update()
#define OBD_UPD_NORMAL    0
#define OBD_UPD_FORCE     1 // update even if no state transition
#define OBD_UPD_HARDFAULT 2 // update w/ hard fault
class OnboardDisplay 
{
#ifdef RED_LED_REG
  DigitalPin pinRedLed;
#endif
#ifdef GREEN_LED_REG
  DigitalPin pinGreenLed;
#endif
#if defined(RGBLCD) || defined(I2CLCD)
#ifdef I2CLCD_PCF8574
  LiquidCrystal_I2C m_Lcd;  
#else
  LiquidTWI2 m_Lcd;
#endif // I2CLCD_PCF8574
#endif // defined(RGBLCD) || defined(I2CLCD)
  uint8_t m_bFlags;
  char m_strBuf[LCD_MAX_CHARS_PER_LINE+1];
  unsigned long m_LastUpdateMs;

  int8_t updateDisabled() { return  m_bFlags & OBDF_UPDATE_DISABLED; }

  void MakeChar(uint8_t n, PGM_P bytes);
public:
  OnboardDisplay();
  void Init();

  void SetGreenLed(uint8_t state) {
#ifdef GREEN_LED_REG
    pinGreenLed.write(state);
#endif
  }

  void SetRedLed(uint8_t state) {
#ifdef RED_LED_REG
  pinRedLed.write(state);
#endif
  }
#ifdef LCD16X2
  void LcdBegin(int x,int y) { 
#ifdef I2CLCD
#ifndef I2CLCD_PCF8574
    m_Lcd.setMCPType(LTI_TYPE_MCP23008);
#endif
    m_Lcd.begin(x,y); 
    m_Lcd.setBacklight(HIGH);
#elif defined(RGBLCD)
    m_Lcd.setMCPType(LTI_TYPE_MCP23017);
    m_Lcd.begin(x,y,2); 
    m_Lcd.setBacklight(WHITE);
#endif // I2CLCD
  }
  void LcdPrint(const char *s) {
    m_Lcd.print(s); 
  }
  void LcdPrint_P(PGM_P s);
  void LcdPrint(int y,const char *s);
  void LcdPrint_P(int y,PGM_P s);
  void LcdPrint(int x,int y,const char *s);
  void LcdPrint_P(int x,int y,PGM_P s);
  void LcdPrint(int i) { 
    m_Lcd.print(i); 
  }
  void LcdSetCursor(int x,int y) { 
    m_Lcd.setCursor(x,y); 
  }
  void LcdClearLine(int y) {
    m_Lcd.setCursor(0,y);
    for (uint8_t i=0;i < LCD_MAX_CHARS_PER_LINE;i++) {
      m_Lcd.write(' ');
    }
    m_Lcd.setCursor(0,y);
  }
  void LcdClear() { 
    m_Lcd.clear();
  }
  void LcdWrite(uint8_t data) { 
    m_Lcd.write(data);
  }
  void LcdMsg(const char *l1,const char *l2);
  void LcdMsg_P(PGM_P l1,PGM_P l2);
  void LcdSetBacklightType(uint8_t t,uint8_t update=OBD_UPD_FORCE) { // BKL_TYPE_XXX
#ifdef RGBLCD
    if (t == BKL_TYPE_RGB) m_bFlags &= ~OBDF_MONO_BACKLIGHT;
    else m_bFlags |= OBDF_MONO_BACKLIGHT;
    Update(update);
#endif // RGBLCD
  }
  uint8_t IsLcdBacklightMono() {
#ifdef RGBLCD
    return (m_bFlags & OBDF_MONO_BACKLIGHT) ? 1 : 0;
#else
    return 1;
#endif // RGBLCD
  }
  void LcdSetBacklightColor(uint8_t c) {
#ifdef RGBLCD
    if (IsLcdBacklightMono()) {
      if (c) c = WHITE;
    }
    m_Lcd.setBacklight(c);
#endif // RGBLCD
  }
#ifdef RGBLCD
  uint8_t readButtons() { return m_Lcd.readButtons(); }
#endif // RGBLCD
#endif // LCD16X2

#ifdef AMMETER
  void SetAmmeterDirty(uint8_t tf) {
    if (tf) m_bFlags |= OBDF_AMMETER_DIRTY;
    else m_bFlags &= ~OBDF_AMMETER_DIRTY;
  }
  int8_t AmmeterIsDirty() { return (m_bFlags & OBDF_AMMETER_DIRTY) ? 1 : 0; }
#endif // AMMETER

  void DisableUpdate(int8_t on) {
    if (on) m_bFlags |= OBDF_UPDATE_DISABLED;
    else m_bFlags &= ~OBDF_UPDATE_DISABLED;
  }
  void Update(int8_t updmode=OBD_UPD_NORMAL); // OBD_UPD_xxx
};

