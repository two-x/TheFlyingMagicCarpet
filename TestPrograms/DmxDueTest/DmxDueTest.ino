/* TODO: add header */

#include <DmxSimple.h>

//======Compiler Definitions
#define NUMMEGABARS 1 //the number of Mega Bars connected to the DMX bus. 4-ch mode assumed. Mega bars must be addressed as 001, 005, 009, etc

//======Program Variables
int MegaBarRed[NUMMEGABARS]; 
int MegaBarGreen[NUMMEGABARS];
int MegaBarBlue[NUMMEGABARS];
int MegaBarBrightness[NUMMEGABARS];

//======Function Definitions
void update_megabars (int offset)
{
  int j;
  offset = offset % NUMMEGABARS;
  for (int i=0; (i<NUMMEGABARS); i++)
  {
    j = ((i + offset) % NUMMEGABARS);
    DmxSimple.write( ((4*j)+1), MegaBarRed[i]);
    DmxSimple.write( ((4*j)+2), MegaBarGreen[i]);
    DmxSimple.write( ((4*j)+3), MegaBarBlue[i]);
    DmxSimple.write( ((4*j)+4), MegaBarBrightness[i]);
  }
}

void clear_megabars ()
{
  // initialize all the outputs to 0: 
  for (int thisMegaBar = 0; thisMegaBar < NUMMEGABARS; thisMegaBar++)
  {
    MegaBarRed[thisMegaBar] = 0; 
    MegaBarGreen[thisMegaBar] = 0;
    MegaBarBlue[thisMegaBar] = 0;  
  }
  
  update_megabars ( 0 );
}

volatile uint32_t *REG;
uint32_t MASK;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print( "setup\n" );
  DmxSimple.usePin(3);
}

void loop() {
  Serial.print( "loop\n" );
  static bool red = true;
  clear_megabars();
  for (int i=0; (i<NUMMEGABARS); i++) {
    if ( red ) {
       MegaBarRed[i] = 255;
    } else {
       MegaBarGreen[i] = 255;
    }
    MegaBarBrightness[i] = 255;
  }
  update_megabars(0);
  if ( red ) {
     digitalWrite(LED_BUILTIN, HIGH);
  } else {
     digitalWrite(LED_BUILTIN, LOW);
  }
  red = !red;
  delay( 500 );
}
