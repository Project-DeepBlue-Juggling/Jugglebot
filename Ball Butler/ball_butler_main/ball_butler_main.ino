/******************************************************************************
 * Teensy 4.0  –  dual-axis demo
 *   • Axis-0  ODrive (BLDC)     : position target  [rev]  (0=top, –ve=down)
 *   • Axis-1  TB67H420 + AS5047 : PWM drive + velocity cap
 * All limits, watchdogs, logging **inside the code** are in REV & RPS.
 * Only the CLI accepts/prints DEGREES (positive down) for convenience.
 ******************************************************************************/

#include <Arduino.h>
#include <SPI.h>

/* ── CAN + ODrive libs (order matters!) ─────────────────────────── */
#include <FlexCAN_T4.h>
#undef  CAN_ERROR_BUS_OFF                  // avoid macro clash
#include "ODriveFlexCAN.hpp"
#include "ODriveCAN.h"
struct ODriveStatus;
/* ───────────────────────────────────────────────────────────────── */

/* ---------- USER DEFAULTS (rev & rps) --------------------------- */
float soft_top_rev =  0.000f;
float soft_bot_rev = -0.150f;      // 0.150 rev  ≙ 54 deg
float abs_top_rev  = -0.0053f;     // watchdog band
float abs_bot_rev  = -0.1540f;

/* ---------------------------------------------------------------- */

/* ---------- helpers: tiny math glue ---------------------------- */
float deg2rev(float d) { return -d / 360.0f; }     // user → internal
float rev2deg(float r) { return -r * 360.0f; }     // internal → user
float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }

/* ====================  ODrive axis-0  =========================== */
#define CAN_BAUDRATE   1000000
#define ODRV0_NODE_ID  1
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
ODriveCAN odrv0(wrap_can_intf(can2), ODRV0_NODE_ID);
ODriveCAN* odrvs[] = { &odrv0 };

struct ODriveUserData {
  Heartbeat_msg_t             hb;
  Get_Encoder_Estimates_msg_t fb;
  bool got_hb=false, got_fb=false;
} odrv0_ud;

float odrv_target_rev = 0.0f;

/* ====================  Brushed axis-1  ========================== */
constexpr uint8_t  CS_PIN=10, INA1_PIN=16, INA2_PIN=15, PWM_PIN=14;
constexpr uint8_t  PWM_MAX=255;
constexpr uint8_t  PWM_MIN=50; // Clip anything below this to 0
constexpr uint16_t AS5047_NOP=0xFFFF;
constexpr float    CPR=16384.0f;

int16_t brush_pwm = 0;
float   brush_vel_rps = 0, brush_pos_rev = 0;
float   prev_ang = 0; uint32_t prev_t = 0;

/* ---------- low-level helpers ---------------------------------- */
uint16_t readAS() {
  uint16_t a;
  SPI.beginTransaction(SPISettings(1'000'000,MSBFIRST,SPI_MODE1));
  digitalWriteFast(CS_PIN,LOW ); SPI.transfer16(AS5047_NOP);        digitalWriteFast(CS_PIN,HIGH);
  digitalWriteFast(CS_PIN,LOW ); a=SPI.transfer16(AS5047_NOP);      digitalWriteFast(CS_PIN,HIGH);
  SPI.endTransaction(); return a&0x3FFF;
}
void driveBM(int16_t pwm){
  if(pwm>0){digitalWriteFast(INA1_PIN,HIGH);digitalWriteFast(INA2_PIN,LOW);}
  else if(pwm<0){digitalWriteFast(INA1_PIN,LOW);digitalWriteFast(INA2_PIN,HIGH);pwm=-pwm;}
  else {digitalWriteFast(INA1_PIN,LOW);digitalWriteFast(INA2_PIN,LOW);}
  analogWrite(PWM_PIN,constrain(pwm,0,PWM_MAX));
}

/* ====================  Serial CLI  ============================= */
String getLine(){
  static String buf;
  while(Serial.available()){
    char c=Serial.read(); if(c=='\r')continue;
    if(c=='\n'){String out=buf;buf="";return out;} buf+=c;
  } return "";
}
void help(){
  Serial.println(F(
  "Commands:\n"
  "  help                – this list\n"
  "  status              – show live data\n"
  "  odpos  <deg>        – set BLDC target (deg, +down)\n"
  "  odlim  <top> <bot>  – soft limits (deg)\n"
  "  odabs  <top> <bot>  – absolute limits (deg)\n"
  "  bmpwm  <-100..100>  – brushed PWM %\n"
  "  bmvlim <rps>        – brushed vel-limit"));
}
void show(){
  Serial.printf("OD pos tgt  : %.2f °  (soft %.2f / %.2f °  abs %.2f / %.2f °)\n",
    rev2deg(odrv_target_rev), rev2deg(soft_top_rev), rev2deg(soft_bot_rev),
    rev2deg(abs_top_rev), rev2deg(abs_bot_rev));
  Serial.printf("BM pwm    : %d   curVel %.2f rps\n",
    brush_pwm, brush_vel_rps);
}
void exec(const String& ln){
  char c[8]; float a,b; int n=sscanf(ln.c_str(),"%7s %f %f",c,&a,&b);
  if(n<1)return;
  if(!strcmp(c,"help")) help();
  else if(!strcmp(c,"status")) show();
  else if(!strcmp(c,"odpos")&&n>=2){
    odrv_target_rev = clampf(deg2rev(a), soft_bot_rev, soft_top_rev);
    Serial.printf("OD target set %.2f °\n", rev2deg(odrv_target_rev));
  }
  else if(!strcmp(c,"odlim")&&n>=3){
    soft_top_rev=deg2rev(a); soft_bot_rev=deg2rev(b);
    Serial.printf("Soft lim %.2f / %.2f °\n",a,b);
  }
  else if(!strcmp(c,"odabs")&&n>=3){
    abs_top_rev=deg2rev(a); abs_bot_rev=deg2rev(b);
    Serial.printf("ABS lim  %.2f / %.2f °\n",a,b);
  }
  else if(!strcmp(c,"bmpwm")&&n>=2){
    brush_pwm=constrain(int(a),-PWM_MAX,PWM_MAX);
    if(abs(brush_pwm) <= PWM_MIN){
      brush_pwm = 0;
    }
    Serial.printf("Brushed PWM %d %%\n",brush_pwm);
  }
  else Serial.println(F("??  try 'help'"));
}

/* ====================  CAN plumbing  =========================== */
bool setupCAN(){
  can2.begin(); can2.setBaudRate(CAN_BAUDRATE); can2.setMaxMB(16);
  can2.enableFIFO(); can2.enableFIFOInterrupt();
  can2.onReceive([](const CanMsg&m){ for(auto o:odrvs) onReceive(m,*o); });
  return true;
}
void onHB(Heartbeat_msg_t&m,void*ctx){((ODriveUserData*)ctx)->hb=m;((ODriveUserData*)ctx)->got_hb=true;}
void onFB(Get_Encoder_Estimates_msg_t&m,void*ctx){((ODriveUserData*)ctx)->fb=m;((ODriveUserData*)ctx)->got_fb=true;}

/* ============================ SETUP ============================ */
void setup(){
  pinMode(CS_PIN,OUTPUT);digitalWriteFast(CS_PIN,HIGH);
  pinMode(INA1_PIN,OUTPUT);pinMode(INA2_PIN,OUTPUT);pinMode(PWM_PIN,OUTPUT);
  analogWriteResolution(8);analogWriteFrequency(PWM_PIN,20000);driveBM(0);

  SPI.begin();
  Serial.begin(115200);while(!Serial&&millis()<3000){}
  Serial.println(F("\nDual-axis rev-internal demo – 'help' for cmds"));

  odrv0.onFeedback(onFB,&odrv0_ud); odrv0.onStatus(onHB,&odrv0_ud);
  if(!setupCAN()){Serial.println("CAN init fail");while(1);}
  while(!odrv0_ud.got_hb) pumpEvents(can2);

  odrv0.setControllerMode(3,1);           // POSITION / PASSTHROUGH
  odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
}

/* ============================ LOOP ============================= */
void loop(){
  pumpEvents(can2);
  String ln=getLine(); if(ln.length())exec(ln);

/* ---- ODrive axis-0 ------------------------------------------- */
  odrv0.setPosition(odrv_target_rev);

/* ---- Brushed axis-1 ------------------------------------------ */
  uint32_t now=micros(); uint16_t raw=readAS();
  float ang=raw/CPR, d=ang-prev_ang;
  if(d>0.5f)d-=1.0f; if(d<-0.5f)d+=1.0f;
  brush_pos_rev+=d; float dt=(now-prev_t)*1e-6f; prev_t=now;
  brush_vel_rps=dt>0?d/dt:0; prev_ang=ang;

  driveBM(brush_pwm);

/* ---- periodic print ------------------------------------------ */
  static uint32_t last=0;
  if(millis()-last>200){
    Serial.printf("OD %.1f°  |  BM %.2f rev  %.2f rps\n",
      rev2deg(odrv_target_rev), brush_pos_rev, brush_vel_rps);
    last=millis();
  }
}
