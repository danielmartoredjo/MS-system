#include <stdio.h>
#include <wiringPi.h>

#define LED	17 //led 17

int main (void)
{
 printf ("rapsberry Pi\n");

 wiringPiSetupGpio ();
 pinMode (LED, OUTPUT);

while(1){
 long pw = 1030; //pulse width in microseconds
 int x;
 for (x=0; x<50; x++)
 {
 digitalWrite(LED, HIGH) ;
 delayMicroseconds (pw) ;
 digitalWrite(LED, LOW) ;
 delayMicroseconds ((16620-pw)) ;
 }
 pw = 1800;
 for (x=0; x<50; x++)
 {
 digitalWrite(LED, HIGH) ;
 delayMicroseconds (pw) ;
 digitalWrite(LED, LOW) ;
 delayMicroseconds ((16620-pw)) ;
 }
}
}
