/*
 *  PWM_Analyzer.c
 */

#include <stdint.h>
#include <wiringPi.h>
#include <unistd.h>

#define PWM_MOTOR_INPUT_PIN 20

int main(void)
{
    // setup wiringPi
    wiringPiSetup();
    pinMode(PWM_MOTOR_INPUT_PIN, INPUT);
    while(1)
    {
        int bail = 1000;
        while((--bail > 0) && (digitalRead(PWM_MOTOR_INPUT_PIN) != HIGH))
        {
            usleep(50);
        }
        int startPulse = micros();

        bail = 1000;
        while((--bail > 0) && (digitalRead(PWM_MOTOR_INPUT_PIN) == HIGH))
        {
            usleep(50);
        }

        int pwmWidth = startPulse - micros();

        printf("Pulse Breedte: %d\n", pwmWidth);

        sleep(0.1);
    }
    return 0;
}

/*
 *  PWM_Analyzer.c - EOF
 */
