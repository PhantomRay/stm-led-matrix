/
#include <stdio.h> 
#include <stdlib.h> 
#include <signal.h> 
#include <pthread.h> 
#include <unistd.h> 
#include <time.h> 
#include <math.h>
#include <sys/time.h> 
#include <pigpio.h>
// the dimensions of your "screen", means the pixel size of all your modules you want to use put together in one row
#define DISPLAY_WIDTH 128*2
# define DISPLAY_HEIGHT 64*3 
#define DISPLAY_SCAN_LINES 32 // scan lines are usually half the 
#display height define EFFECT 1 // 1: Gray gradient (static) 2: Random gray static 3: Random color static 4: Animated 
#Rainbow 5: Animated random gray static
// leave this like it is, the pins are carefully selected to not interfere with special purpose pins also pins with the 
// same function are grouped together so they can be set with one command to save CPU cycles
#define PIN_CLK 17 //11 define PIN_LE 4 //7 define PIN_OE 18 //12 define PIN_A 22 //15 define PIN_B 23 //16 define 
#PIN_C 24 //18 define PIN_D 25 //22 define PIN_E 15 //10
const unsigned long MASK_SCAN_LINE = (1<<PIN_A)|(1<<PIN_B)|(1<<PIN_C)|(1<<PIN_D)|(1<<PIN_E);
#define PIN1_R1 11 //23 define PIN1_G1 27 //13 define PIN1_B1 7 //26 define PIN1_R2 8 //24 define PIN1_G2 9 //21 define 
#PIN1_B2 10 //19
const unsigned long MASK_COLOR1_RED = (1<<PIN1_R1)|(1<<PIN1_R2); const unsigned long MASK_COLOR1_GREEN = 
(1<<PIN1_G1)|(1<<PIN1_G2); const unsigned long MASK_COLOR1_BLUE = (1<<PIN1_B1)|(1<<PIN1_B2); const unsigned long 
MASK_COLOR1 = MASK_COLOR1_RED | MASK_COLOR1_GREEN | MASK_COLOR1_BLUE;
#define PIN2_R1 12 define PIN2_G1 5 define PIN2_B1 6 define PIN2_R2 19 define PIN2_G2 13 define PIN2_B2 20
const unsigned long MASK_COLOR2_RED = (1<<PIN2_R1)|(1<<PIN2_R2); const unsigned long MASK_COLOR2_GREEN = 
(1<<PIN2_G1)|(1<<PIN2_G2); const unsigned long MASK_COLOR2_BLUE = (1<<PIN2_B1)|(1<<PIN2_B2); const unsigned long 
MASK_COLOR2 = MASK_COLOR2_RED | MASK_COLOR2_GREEN | MASK_COLOR2_BLUE;
#define PIN3_R1 14 define PIN3_G1 2 define PIN3_B1 3 define PIN3_R2 26 define PIN3_G2 16 define PIN3_B2 21
const unsigned long MASK_COLOR3_RED = (1<<PIN3_R1)|(1<<PIN3_R2); const unsigned long MASK_COLOR3_GREEN = 
(1<<PIN3_G1)|(1<<PIN3_G2); const unsigned long MASK_COLOR3_BLUE = (1<<PIN3_B1)|(1<<PIN3_B2); const unsigned long 
MASK_COLOR3 = MASK_COLOR3_RED | MASK_COLOR3_GREEN | MASK_COLOR3_BLUE; const unsigned long MASK_COLOR_RED = 
MASK_COLOR1_RED | MASK_COLOR2_RED | MASK_COLOR3_RED; const unsigned long MASK_COLOR_BLUE = MASK_COLOR1_BLUE | 
MASK_COLOR2_BLUE | MASK_COLOR3_BLUE; const unsigned long MASK_COLOR_GREEN = MASK_COLOR1_GREEN | MASK_COLOR2_GREEN | 
MASK_COLOR3_GREEN; const unsigned long MASK_COLOR = MASK_COLOR_RED | MASK_COLOR_BLUE | MASK_COLOR_GREEN;
#define BUFFER_TYPE uint16_t
// global variables
unsigned int displayBufferSize = DISPLAY_WIDTH*DISPLAY_HEIGHT*3; // the buffers need to hold 16 bit of image data per 
pixel over 6 outputs (RGB1 + RGB2) unsigned int displayBufferLineSize = DISPLAY_WIDTH*3; unsigned int 
displayNumberChips = DISPLAY_WIDTH / 16; unsigned char pins[26] = {PIN_CLK, PIN_LE, PIN_OE, PIN_A, PIN_B, PIN_C, PIN_D, 
PIN_E, PIN1_R1, PIN1_G1, PIN1_B1, PIN1_R2, PIN1_G2, PIN1_B2, PIN2_R1, PIN2_G1, PIN2_B1, PIN2_R2, PIN2_G2, PIN2_B2, 
PIN3_R1, PIN3_G1, PIN3_B1, PIN3_R2, PIN3_G2, PIN3_B2}; uint16_t Translate8To16Bit[256] = 
{0,46,92,139,186,233,280,327,375,422,470,519,567,615,664,713,762,812,861,911,961,1011,1061,1112,1163,1214,1265,1317,1368,1420,1473,1525,1578,1631,1684,1737,1791,1844,1899,1953,2007,2062,2117,2173,2228,2284,2340,2397,2453,2510,2568,2625,2683,2741,2799,2858,2917,2976,3036,3096,3156,3216,3277,3338,3399,3461,3523,3586,3648,3711,3775,3838,3902,3967,4032,4097,4162,4228,4294,4361,4428,4495,4563,4631,4699,4768,4838,4907,4978,5048,5119,5191,5262,5335,5407,5481,5554,5628,5703,5778,5853,5929,6006,6083,6160,6238,6317,6396,6476,6556,6636,6718,6799,6882,6965,7048,7132,7217,7302,7388,7475,7562,7650,7739,7828,7918,8008,8099,8191,8284,8377,8472,8567,8662,8759,8856,8954,9053,9153,9253,9355,9457,9560,9664,9769,9875,9982,10090,10199,10309,10420,10532,10645,10760,10875,10991,11109,11228,11348,11469,11591,11715,11840,11967,12094,12223,12354,12486,12620,12755,12891,13030,13169,13311,13454,13599,13746,13895,14045,14198,14352,14509,14667,14828,14991,15157,15324,15494,15667,15842,16020,16200,16383,16569,16758,16951,17146,17345,17547,17752,17961,18174,18391,18612,18837,19067,19301,19539,19783,20032,20286,20546,20812,21083,21361,21646,21938,22237,22544,22859,23183,23516,23859,24211,24575,24950,25338,25739,26153,26583,27029,27493,27975,28478,29003,29553,30130,30736,31375,32051,32767,33530,34345,35221,36167,37195,38322,39567,40959,42537,44359,46514,49151,52551,57343,65535}; 
unsigned char activeScreenBuffer = 0; BUFFER_TYPE *screenBuffer[2]; uint64_t epochMicro = 0; void initializeEpoch() {
  struct timespec ts ; clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ; epochMicro = (uint64_t)ts.tv_sec * (uint64_t)1000000 
  + (uint64_t)(ts.tv_nsec / 1000L) ;
}
long micros() { struct timespec ts ; clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ; uint64_t now = (uint64_t)ts.tv_sec * 
  (uint64_t)1000000 + (uint64_t)(ts.tv_nsec / 1000) ; return (uint32_t)(now - epochMicro) ;
}
void delayMicrosecondsHard (unsigned int howLong) { struct timeval tNow, tLong, tEnd ; gettimeofday (&tNow, NULL) ; 
  tLong.tv_sec = howLong / 1000000 ; tLong.tv_usec = howLong % 1000000 ; timeradd (&tNow, &tLong, &tEnd) ; while 
  (timercmp (&tNow, &tEnd, <))
    gettimeofday (&tNow, NULL) ;
}
void delayMicroseconds (unsigned int howLong) { struct timespec sleeper ; unsigned int uSecs = howLong % 1000000 ; 
  unsigned int wSecs = howLong / 1000000 ; /**/ if (howLong == 0)
    return ; else if (howLong < 100) delayMicrosecondsHard (howLong) ; else { sleeper.tv_sec = wSecs ; sleeper.tv_nsec 
    = (long)(uSecs * 1000L) ; nanosleep (&sleeper, NULL) ;
  }
}
void digitalWriteEvenFaster(unsigned long data, unsigned long mask) {
    // GPIO.out writes multiple pins at the same time (ESP-IDF command) GPIO.out = (GPIO.out & ~mask) | data;
    gpioWrite_Bits_0_31_Clear(mask); gpioWrite_Bits_0_31_Set(data&mask);
    //bits = (data>>32) & (mask>>32); gpioWrite_Bits_32_53_Set(bits);
}
void sendClock() { gpioWrite(PIN_CLK, 1); gpioWrite(PIN_CLK, 0);
}
void sendPwmClock(unsigned char clocks) { while (clocks--) { gpioWrite(PIN_OE, 1); gpioWrite(PIN_OE, 0);
    }
}
void sendLatch(unsigned char clocks) { gpioWrite(PIN_LE, 1); while(clocks--) { sendClock();
    }
    gpioWrite(PIN_LE, 0);
}
void sendScanLine(unsigned char line) { unsigned long scanLine = 0x00000000; if(line & 0x1) scanLine |= (1<<PIN_A); 
    if(line>>1 & 0x1) scanLine |= (1<<PIN_B); if(line>>2 & 0x1) scanLine |= (1<<PIN_C); if(line>>3 & 0x1) scanLine |= 
    (1<<PIN_D); if(line>>4 & 0x1) scanLine |= (1<<PIN_E); digitalWriteEvenFaster(scanLine, MASK_SCAN_LINE);
}
void cacheWrite(BUFFER_TYPE *buffer, unsigned int posX, unsigned int posY, unsigned char red, unsigned char green, 
unsigned char blue) {
    unsigned int bufferPos = posX*3 + posY * displayBufferLineSize; /*unsigned int inputMask = 0x8000; BUFFER_TYPE 
    outputmask = 0x07; BUFFER_TYPE rgb = 0x00; unsigned int red16Bit = Translate8To16Bit[red]; unsigned int green16Bit 
    = Translate8To16Bit[green]; unsigned int blue16Bit = Translate8To16Bit[blue];
    // data is saved in a format where the refresh method just needs to send it out instead of formatting it first, 
    // which makes it 7 fps faster the highest bit of each color is saved in a char and written into the buffer, then 
    // the second highest bit and so on since the RGB values of two different lines are sent at the same time, when y 
    // is bigger than the scan line area the data is written into different bits of the buffer data
    for(unsigned char x = 0; x < 3; x++) { rgb = 0x00; inputMask = 0x8000 >> x; if(red16Bit & inputMask) rgb += 1; 
	if(green16Bit & inputMask) rgb += 2; if(blue16Bit & inputMask) rgb += 4;
        // write data without changing the surrounding bits
	buffer[bufferPos + x] = (buffer[bufferPos + x] & ~outputmask) | rgb;
    }*/
    buffer[bufferPos] = Translate8To16Bit[red]; buffer[bufferPos+1] = Translate8To16Bit[green]; buffer[bufferPos+2] = 
    Translate8To16Bit[blue];
}
void loop() {} void LedWallRefresh() { BUFFER_TYPE *activeBuffer = screenBuffer[activeScreenBuffer]; // save a 
    reference to the active buffer to avoid sending data from a different cycle in between unsigned int pos; unsigned 
    int bufferPos; unsigned long sendBuffer = 0; unsigned long red, green, blue; unsigned long outMask; sendLatch(3); 
    // send vsync since the generation of the output signal in the ICN2053 chips is directly tied to the input clock 
    // signal when receiving pixel data, the order and amount of clock cycles, latches, PWM clock and so on can not be 
    // changed.
    for(unsigned int y = 0; y < DISPLAY_SCAN_LINES; y++) { // 0-N scan lines * 2 = pixel height for(unsigned int x = 0; 
        x < 16; x++) { // 0-15 because 1 chip has 16 outputs
            bufferPos = y * displayBufferLineSize + x * 16; sendScanLine(y % 2 * DISPLAY_SCAN_LINES / 2 + x); // sends 
            0-N scan lines in every 2 (4 combined) data lines
	    //sendScanLine(1);
            sendPwmClock(138); // send 138 clock cycles for PWM generation inside the ICN2053 chips - this needs to be 
            exactly 138 pulses for(unsigned int sect = 0; sect < displayNumberChips; sect++) { // 0-N number of chips
                pos = bufferPos + sect * 16 * 3; pos = y*displayBufferLineSize + x*3 + sect*3*16; for(int bit = 15; bit 
                >= 0 && bit <= 15; bit--) { // 0-16 bits of pixel data
	sendBuffer = 0; outMask = 1<<bit; red = activeBuffer[pos] & outMask; green = activeBuffer[pos+1] & outMask; 
	blue = activeBuffer[pos+2] & outMask;
       	  sendBuffer |= (red>>bit)<<PIN1_R1; sendBuffer |= (green>>bit)<<PIN1_G1; sendBuffer |= 
	  (blue>>bit)<<PIN1_B1; blue = activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize+2] & outMask; red = 
	  activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize] & outMask; green = 
	  activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize+1] & outMask; sendBuffer |= (red>>bit)<<PIN1_R2; 
	  sendBuffer |= (green>>bit)<<PIN1_G2; sendBuffer |= (blue>>bit)<<PIN1_B2;
	
	  //if (sect==0&&x==0&&y==31) printf("%i %08x\n", bit, sendBuffer&MASK_COLOR); printf("%08x\n", MASK_COLOR);
  	  
	  blue = activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize*2+2] & outMask; red = 
	  activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize*2] & outMask; green = 
	  activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize*2+1] & outMask; sendBuffer |= (red>>bit)<<PIN2_R1; 
	  sendBuffer |= (green>>bit)<<PIN2_G1; sendBuffer |= (blue>>bit)<<PIN2_B1;
	  
	  blue = activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize*3+2] & outMask; red = 
	  activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize*3] & outMask; green = 
	  activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize*3+1] & outMask; sendBuffer |= (red>>bit)<<PIN2_R2; 
	  sendBuffer |= (green>>bit)<<PIN2_G2; sendBuffer |= (blue>>bit)<<PIN2_B2;
	  
	  blue = activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize*4+2] & outMask; red = 
	  activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize*4] & outMask; green = 
	  activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize*4+1] & outMask; sendBuffer |= (red>>bit)<<PIN3_R1; 
	  sendBuffer |= (green>>bit)<<PIN3_G1; sendBuffer |= (blue>>bit)<<PIN3_B1;
	  
	  blue = activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize*5+2] & outMask; red = 
	  activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize*5] & outMask; green = 
	  activeBuffer[pos+DISPLAY_SCAN_LINES*displayBufferLineSize*5+1] & outMask; sendBuffer |= (red>>bit)<<PIN3_R2; 
	  sendBuffer |= (green>>bit)<<PIN3_G2; sendBuffer |= (blue>>bit)<<PIN3_B2;
	  
                    digitalWriteEvenFaster(sendBuffer, MASK_COLOR);
	            //digitalWriteEvenFaster(x==0&&sect==0 ? MASK_COLOR:0, MASK_COLOR);
                    if(sect == displayNumberChips - 1 && bit == 0) gpioWrite(PIN_LE, 1); // send a latch for 1 clock 
                    after every chip was written to sendClock();
                }
                gpioWrite(PIN_LE, 0);
            }
        }
    }
}
/* * tasks */ char running = 1; void *refreshTask(void *pvParameters) { (void) pvParameters; while(running) { 
        LedWallRefresh();
        //vTaskDelay(1); // call ESP-IDF command vTaskDelay to avoid watchdog restarting CPU
	//usleep(1); delayMicroseconds(100);
    }
}
void *dataTask(void *pvParameters) { (void) pvParameters;
    // start time measurement
    unsigned long start = micros(); unsigned long time;
    // everything from this point on is not documented, implement your own data processing or generation here. use: 
    // cacheWrite(*inactiveBuffer, x, y, r, g, b); to set a pixel in the inactive buffer, then swap buffers
    #if EFFECT < 3 || EFFECT == 5
        unsigned char gray = 0;
    #endif if EFFECT > 3
        unsigned int pos1 = 0; unsigned int pos2 = 0; unsigned int posXY = 0; unsigned int displayBufferHalfScreenSize 
        = (DISPLAY_HEIGHT - 1) * displayBufferLineSize; unsigned int posXYOutput = 0; unsigned int posXYInput = 0; 
        unsigned char outputmask = 0x38; unsigned char inputmask = 0x07;
    #endif if EFFECT == 4
        float widthDivRad = (float)DISPLAY_WIDTH / (2.0*M_PI); float one20DegreeRad = 2.0*M_PI / 3.0; float count = 0; 
        unsigned char red = 0; unsigned char green = 0; unsigned char blue = 0;
    #endif
    while(running) { unsigned char inactiveScreenBuffer = (activeScreenBuffer + 1) & 0x1; BUFFER_TYPE *inactiveBuffer = 
        screenBuffer[inactiveScreenBuffer]; BUFFER_TYPE *activeBuffer = screenBuffer[activeScreenBuffer]; for (unsigned 
        int x = 0; x < DISPLAY_WIDTH; x++) {
            for (unsigned int y = 0; y < DISPLAY_HEIGHT; y++) {
		#if EFFECT == 0
		    cacheWrite(inactiveBuffer, x, y, 0, 0, 255);
		#endif
                #if EFFECT == 1
                    if(y / 8 % 2 == 0) gray = (unsigned char)(x * 256 / DISPLAY_WIDTH); else gray = (unsigned 
                    char)((DISPLAY_WIDTH - x - 1) * 256 / DISPLAY_WIDTH); cacheWrite(inactiveBuffer, x, y, gray, gray, 
                    gray);
                #endif if EFFECT == 2
                    gray = (unsigned char)random(); cacheWrite(inactiveBuffer, x, y, gray, gray, gray);
                #endif if EFFECT == 3
                    cacheWrite(inactiveBuffer, x, y, (unsigned char)random(), (unsigned char)random(), (unsigned 
                    char)random());
                #endif if EFFECT > 3
                    if(y > 0 && y < DISPLAY_HEIGHT) { posXY = (DISPLAY_HEIGHT - y - 1) * displayBufferLineSize + x*3; 
                        for (unsigned int bit = 0; bit < 3; bit++) {
                            pos1 = posXY + bit; pos2 = pos1 + displayBufferLineSize; inactiveBuffer[pos2] = 
                            activeBuffer[pos1];
                        }
                    }
                #endif
            }
            #if EFFECT > 3
                posXYOutput = x * 3; posXYInput = displayBufferHalfScreenSize + posXYOutput; for (unsigned int bit = 0; 
                bit < 3; bit++) {
                    inactiveBuffer[posXYOutput + bit] = (activeBuffer[posXYInput + bit]);
                }
            #endif if EFFECT == 4
                float sine1Hertz = (float)x / widthDivRad + count; red = (unsigned char)((sin(sine1Hertz ) + 1) * 128); 
                green = (unsigned char)((sin(sine1Hertz + one20DegreeRad ) + 1) * 128); blue = (unsigned 
                char)((sin(sine1Hertz + one20DegreeRad * 2) + 1) * 128); cacheWrite(inactiveBuffer, x, 0, red, green, 
                blue);
            #endif if EFFECT == 5
                gray = random(256); cacheWrite(inactiveBuffer, x, 0, gray, gray, gray);
            #endif
        }
        
        #if EFFECT == 4
            count += 0.1; if(count > (2.0*M_PI)) count -= 2.0*M_PI;
        #endif
        // swap buffers
        activeScreenBuffer = inactiveScreenBuffer;
        // calculate time delayed by effect generation, set delay to reach fixed 30 fps vTaskDelay(5 / 
        //portTICK_PERIOD_MS); // 5ms delay to allow other background tasks to do something
        delayMicroseconds(500); time = 28333 - (micros() - start); if(time < 28333) delayMicroseconds(time); start = 
        micros();
    }
}
/* * normal methods */ void sendConfiguration(unsigned int data, unsigned char latches) { unsigned char num = 
    displayNumberChips; unsigned int dataMask; unsigned long zero = 0x00000000; unsigned long rgbrgbMask = MASK_COLOR; 
    // all 6 RGB pins high
    latches = 16 - latches;
    // send config data to all chips involved (4 per 64 pixel), then latch for 1 clock
    while(num--) { for(unsigned char x = 0; x < 16; x++) { dataMask = 0x8000 >> x; digitalWriteEvenFaster(data & 
            dataMask ? rgbrgbMask : zero, rgbrgbMask); if(num == 0 && x == latches) gpioWrite(PIN_LE, 1); sendClock();
        }
        gpioWrite(PIN_LE, 0);
    }
}
pthread_t refreshThread, dataThread; static void terminate() { running = 0; void* val; pthread_join(refreshThread, 
	&val); pthread_join(dataThread, &val); gpioTerminate(); exit(0);
}
static void intHandler(int d) { terminate();
}
/* * Arduino setup + loop */ void setup() { if(gpioInitialise() < 0) { exit(-1);
    }
    signal(SIGINT, terminate);
    //struct sigaction act; act.sa_handler=intHandler; sigaction(SIGINT | SIGQUIT, &act, NULL);
    initializeEpoch();
    //pigpio_start(); create screen buffers (uses ESP-IDF malloc command to find a piece of dynamic memory to hold the 
    // buffer) we use double buffering to be able to manipulate one buffer while the display refresh uses the other 
    // buffer
    screenBuffer[0] = (BUFFER_TYPE*)malloc(displayBufferSize*sizeof(BUFFER_TYPE)); screenBuffer[1] = 
    (BUFFER_TYPE*)malloc(displayBufferSize*sizeof(BUFFER_TYPE));
    // fill buffers with 0
    for (unsigned int x = 0; x < displayBufferSize; x++) { screenBuffer[0][x] = 0; screenBuffer[1][x] = 0;
    }
    // setup pins
    for (unsigned char x = 0; x < 26; x++) { gpioSetMode(pins[x], PI_OUTPUT); gpioWrite(pins[x], 0);
    }
    // send screen configuration to the ICN2053 chips this configuration was created by @ElectronicsInFocus 
    // https://www.youtube.com/watch?v=nhCGgTd7OHg also the original C++ code this Arduino version is based on was 
    // created by him it is unknown to me what this configuration does, there is no information about it whatsoever on 
    // the internet (not that I could find) if someone has a document describing the configuration registers please 
    // send it to me!
    sendLatch(14); // Pre-active command sendLatch(12); // Enable all output channels sendLatch(3); // Vertical sync 
    sendLatch(14); // Pre-active command sendConfiguration(0b0001111101110000, 4); // Write config register 1 (4 
    latches) sendLatch(14); // Pre-active command sendConfiguration(0xffff, 6); // Write config register 2 (6 latches) 
    sendLatch(14); // Pre-active command sendConfiguration(0b0100000011110011, 8); // Write config register 3 (8 
    latches) sendLatch(14); // Pre-active command sendConfiguration(0x0000, 10); // Write config register 4 (10 
    latches) sendLatch(14); // Pre-active command sendConfiguration(0x0000, 2); // Write debug register (2 latches)
    // initialize the refresh and data "acquisition" tasks to run on separate CPU cores this is an ESP-IDF feature to 
    // run tasks (endless loops) on different cores callback, name, stack size, null, priority 3-0 (0 lowest), null, 
    // core (0 or 1)
    //xTaskCreatePinnedToCore(refreshTask, "refreshTask", 1024, NULL, 3, NULL, 0); xTaskCreatePinnedToCore(dataTask, 
    //"dataTask", 1024, NULL, 1, NULL, 1);
    pthread_create(&refreshThread, NULL, refreshTask, NULL); pthread_create(&dataThread, NULL, dataTask, NULL);
}
int main(int argc, char *argv[]) { setup(); while(running){ loop();
	}
}
