#include <stdio.h>
#include <wiringPi.h>

#define LED	18 //led 18

int main (void)
{
 printf ("rapsberry Pi\n");

 wiringPiSetupGpio ();
 pinMode (LED, OUTPUT);

 long pwinit = 1520; //calibration  width in microseconds
 int x;
 for (x= 0; x<100; x++)
 {
 digitalWrite(LED, HIGH);
 delayMicroseconds (pwinit);
 digitalWrite(LED, LOW);
 delayMicroseconds (165000-pwinit);
 }

 long pw = 1100;
 for (x=0; x<50000; x++)
 {
 digitalWrite(LED, HIGH) ;
 delayMicroseconds (pw) ;
 digitalWrite(LED, LOW) ;
 delayMicroseconds ((16500-pw)) ;
 }
 return 0;
}
