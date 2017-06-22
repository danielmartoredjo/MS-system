/*
 *  PWM_Analyzer.c
 */

#include <stdio.h>
#include <wiringPi.h>
#include <unistd.h>

#define PWM_MOTOR_INPUT_PIN 20

int main(void)
{
    // setup wiringPi
    FILE *pwmDD;
    wiringPiSetupGpio();
    pinMode(PWM_MOTOR_INPUT_PIN, INPUT);
    int bail;
    long startPulse;
    long pwmWidth;
    int pwmOld = 1520, pwmNew = 1520;
    while(1)
    {
        bail = 1000;
        while((digitalRead(PWM_MOTOR_INPUT_PIN) != HIGH))
        {
            // delayMicroseconds (10);
            usleep(50);
            // printf("LOW b:%d\n", bail);
        }
        startPulse = micros();

        bail = 1000;
        while((digitalRead(PWM_MOTOR_INPUT_PIN) == HIGH))
        {
            // delayMicroseconds (10);
            usleep(50);
            // printf("HIGH b:%d\n", bail);
        }
        // usleep(50);
        pwmWidth = micros() - startPulse;

        if (pwmWidth < 1600)
        {
            printf("Stop\n");
            pwmNew = 1520;
        }
        else if (pwmWidth >= 1800)
        {
            printf("Drive\n");
            pwmNew = 1600;
        }

        if (pwmNew != pwmOld)
        {
            pwmDD = fopen("/dev/pwm_drv","w");
            if (pwmNew == 1600)
            {
                fprintf(pwmDD,"M1600");
            } else {
                fprintf(pwmDD,"S");
            }
            fclose(pwmDD);
            pwmOld = pwmNew;
        }

        sleep(0.2);
    }
    return 0;
}

/*
 *  PWM_Analyzer.c - EOF
 */
