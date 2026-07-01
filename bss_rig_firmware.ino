/* =====================================================================
   B STRAIN SENSOR - Cekme Duzenegi Firmware  (Arduino Uno / Nano)  v4
   ---------------------------------------------------------------------
   SWITCH'SIZ calisir: fiziksel limit switch YOK. Bunun yerine YAZILIMSAL
   (soft) home + soft limit kullanilir:
     POS0        -> bulundugun noktayi BASLANGIC (0) yap (HOME buraya doner)
     SETMAX <mm> -> bulundugun/verilen noktayi SON (ust sinir) yap
     HOME        -> yazilimsal 0'a (baslangica) don
   Motor [0 .. max] araligi disina CIKMAZ.

   ANA MOD: DONGUSEL (cyclic):
     CYCLE <genlik_mm> <dongu_sayisi> <uc_bekleme_ms>
   Diger: PING EN1/EN0 SPMM RATE MOVE JOG GOTO STOP TARE SCALE STREAM1/0
   HX711 BLOKLAMAZ. Telemetri: T <konum_mm> <kuvvet_N> <min> <max> <hareket>
   (min=1: baslangictasin, max=1: sinirdasin)

   NOT: Fiziksel switch kullanmak istersen USE_HW_LIMITS = true yap.
   ===================================================================== */

const bool USE_HW_LIMITS = false;          // <<< switch YOK -> false

const uint8_t PIN_STEP=3, PIN_DIR=4, PIN_EN=5, HX_DT=6, HX_SCK=7, LIM_MIN=9, LIM_MAX=10;

float  spmm=1600.0, rate=60.0, homeRate=120.0;
long   curSteps=0, tgtSteps=0, maxSteps=0;   // maxSteps=0 -> ust sinir kapali
int    moveDir=0;
bool   moving=false, homing=false, enabled=false;
unsigned long lastStepUs=0, stepIntervalUs=1000;

long   hxRaw=0, hxTare=0;  float hxScale=1000.0;
bool   streaming=false;  unsigned long lastTeleMs=0;

bool   cycling=false;  long cycAmp=0, cycBase=0;  int cycTotal=0, cycDone=0;
int    cycPhase=0;     unsigned long cycDwell=0, cycDwellUntil=0;

char buf[56]; uint8_t bufi=0;

inline bool limMin(){ return USE_HW_LIMITS ? (digitalRead(LIM_MIN)==HIGH) : false; }
inline bool limMax(){ return USE_HW_LIMITS ? (digitalRead(LIM_MAX)==HIGH) : false; }
void setEnable(bool on){ digitalWrite(PIN_EN, on?LOW:HIGH); enabled=on; }

long clampT(long t){ if(t<0) t=0; if(maxSteps>0 && t>maxSteps) t=maxSteps; return t; }

bool hxUpdate(){
  if (digitalRead(HX_DT)==HIGH) return false;
  long v=0;
  for (uint8_t i=0;i<24;i++){ digitalWrite(HX_SCK,HIGH); delayMicroseconds(1);
    v=(v<<1)|digitalRead(HX_DT); digitalWrite(HX_SCK,LOW); delayMicroseconds(1); }
  digitalWrite(HX_SCK,HIGH); delayMicroseconds(1); digitalWrite(HX_SCK,LOW); delayMicroseconds(1);
  if (v & 0x800000) v|=~0xFFFFFFL;  hxRaw=v;  return true;
}
float forceN(){ return (float)(hxRaw-hxTare)/hxScale; }

void recalcInterval(float r){ float sps=r*spmm/60.0; if(sps<1)sps=1; stepIntervalUs=(unsigned long)(1000000.0/sps); }

void beginMoveTo(long target,float r){
  target=clampT(target);
  if(target==curSteps){ moving=false; return; }
  moveDir=(target>curSteps)?1:-1; tgtSteps=target;
  digitalWrite(PIN_DIR, moveDir>0?HIGH:LOW);
  if(!enabled)setEnable(true); recalcInterval(r);
  moving=true; homing=false; lastStepUs=micros();
}
void cycStretch(){ cycPhase=0; beginMoveTo(cycBase+cycAmp, rate); }
void cycReturn(){  cycPhase=2; beginMoveTo(cycBase, rate); }

void startMoveRel(float mm){ beginMoveTo(curSteps + (long)(mm*spmm), rate); }
void stopAll(){ moving=false; homing=false; cycling=false; }
inline void doStep(){ digitalWrite(PIN_STEP,HIGH); delayMicroseconds(3); digitalWrite(PIN_STEP,LOW); curSteps+=moveDir; }

void cycleAdvance(){
  moving=false;
  if (cycPhase==0){
    if (cycDwell>0){ cycPhase=1; cycDwellUntil=millis()+cycDwell; }
    else cycReturn();
  } else if (cycPhase==2){
    cycDone++;
    Serial.print(F("CYC ")); Serial.print(cycDone); Serial.print('/'); Serial.println(cycTotal);
    if (cycDone>=cycTotal){ cycling=false; Serial.println(F("CYCLEDONE")); }
    else if (cycDwell>0){ cycPhase=3; cycDwellUntil=millis()+cycDwell; }
    else cycStretch();
  }
}

void handle(char* s){
  if(!strcmp(s,"PING")){ Serial.println(F("OK BSSRIG v4")); return; }
  if(!strcmp(s,"EN1")){ setEnable(true);  Serial.println(F("OK EN1")); return; }
  if(!strcmp(s,"EN0")){ setEnable(false); Serial.println(F("OK EN0")); return; }
  if(!strcmp(s,"STOP")){ stopAll(); Serial.println(F("OK STOP")); return; }
  if(!strcmp(s,"POS0")){ curSteps=0; if(cycling){} Serial.println(F("OK POS0")); return; }   // BASLANGIC
  if(!strcmp(s,"HOME")){ homing=true; beginMoveTo(0, homeRate); homing=true; Serial.println(F("OK HOME")); return; }
  if(!strcmp(s,"STREAM1")){ streaming=true; Serial.println(F("OK STREAM1")); return; }
  if(!strcmp(s,"STREAM0")){ streaming=false; Serial.println(F("OK STREAM0")); return; }
  if(!strcmp(s,"TARE")){ long sum=0; int n=0; unsigned long t0=millis();
    while(n<16 && millis()-t0<400){ if(hxUpdate()){ sum+=hxRaw; n++; } }
    if(n>0) hxTare=sum/n; Serial.println(F("OK TARE")); return; }
  if(!strncmp(s,"CYCLE",5)){
    float vals[3]={0,0,0}; int k=0; char* p=strtok(s+5," ");
    while(p && k<3){ vals[k++]=atof(p); p=strtok(NULL," "); }
    cycBase=clampT(curSteps);
    cycAmp=(long)(vals[0]*spmm);
    if(maxSteps>0 && cycBase+cycAmp>maxSteps) cycAmp=maxSteps-cycBase;   // sona sigdir
    if(cycAmp<0) cycAmp=0;
    cycTotal=(int)vals[1]; if(cycTotal<1)cycTotal=1;
    cycDwell=(unsigned long)vals[2]; cycDone=0; cycling=true;
    if(!enabled)setEnable(true);
    Serial.println(F("OK CYCLE")); cycStretch(); return;
  }
  char* sp=strchr(s,' ');
  if(sp){ *sp=0; float v=atof(sp+1);
    if(!strcmp(s,"SPMM")){ spmm=v; Serial.println(F("OK SPMM")); return; }
    if(!strcmp(s,"RATE")){ rate=v; Serial.println(F("OK RATE")); return; }
    if(!strcmp(s,"SCALE")){ hxScale=(v==0?1:v); Serial.println(F("OK SCALE")); return; }
    if(!strcmp(s,"SETMAX")){ maxSteps=(long)(v*spmm); if(maxSteps<0)maxSteps=0; Serial.println(F("OK SETMAX")); return; }
    if(!strcmp(s,"MOVE")||!strcmp(s,"JOG")){ startMoveRel(v); Serial.println(F("OK MOVE")); return; }
    if(!strcmp(s,"GOTO")){ beginMoveTo((long)(v*spmm), rate); Serial.println(F("OK GOTO")); return; }
  }
  Serial.println(F("ERR"));
}

void setup(){
  pinMode(PIN_STEP,OUTPUT); pinMode(PIN_DIR,OUTPUT); pinMode(PIN_EN,OUTPUT);
  pinMode(HX_SCK,OUTPUT); pinMode(HX_DT,INPUT);
  if(USE_HW_LIMITS){ pinMode(LIM_MIN,INPUT_PULLUP); pinMode(LIM_MAX,INPUT_PULLUP); }
  setEnable(false); Serial.begin(115200); Serial.println(F("OK BSSRIG v4"));
}

void loop(){
  while(Serial.available()){ char c=Serial.read();
    if(c=='\n'||c=='\r'){ if(bufi){ buf[bufi]=0; handle(buf); bufi=0; } }
    else if(bufi<sizeof(buf)-1) buf[bufi++]=c; }

  if(moving){
    if(USE_HW_LIMITS && moveDir>0 && limMax()){ stopAll(); Serial.println(F("LIM MAX")); }
    else if(USE_HW_LIMITS && moveDir<0 && limMin()){ stopAll(); Serial.println(F("LIM MIN")); }
    else if(micros()-lastStepUs>=stepIntervalUs){
      doStep(); lastStepUs+=stepIntervalUs;
      bool reached = (moveDir>0 && curSteps>=tgtSteps)||(moveDir<0 && curSteps<=tgtSteps);
      if(reached){
        if(cycling) cycleAdvance();
        else { moving=false; if(homing){ homing=false; } Serial.println(F("DONE")); }
      }
    }
  }
  if(cycling && !moving && (cycPhase==1||cycPhase==3) && (long)(millis()-cycDwellUntil)>=0){
    if(cycPhase==1) cycReturn(); else cycStretch();
  }

  hxUpdate();
  if(streaming && millis()-lastTeleMs>=50){
    lastTeleMs=millis();
    int mn=(curSteps<=0)?1:0;
    int mx=(maxSteps>0 && curSteps>=maxSteps)?1:0;
    Serial.print(F("T ")); Serial.print(curSteps/spmm,3); Serial.print(' ');
    Serial.print(forceN(),3); Serial.print(' ');
    Serial.print(mn); Serial.print(' '); Serial.print(mx); Serial.print(' ');
    Serial.println(moving?1:0);
  }
}
