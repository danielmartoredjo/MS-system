/*
 * File: rfcomm_client.c
 * version: 1.4.2
 */
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

//#define SERVER_ADDR "B8:27:EB:5D:7B:C9" //Daniel
#define SERVER_ADDR "B8:27:EB:70:15:29" //Slave

int connect_bluetooth(int* s, int* status);

int main(int argc, char **argv){
	int s, status;
	char str[30];
	FILE *file;
	//int speed = 0;
	int accX = 0;
	int accY = 0;
	int accZ = 0;
        char message[]="VXXXX_AxYYYY_AyYYYY_AzYYYY"; //XXXX is the speed in rpm and YYYY is the acceleration in something
        int i = 0;

	connect_bluetooth(&s, &status);

	while(1){
		file = fopen( "/dev/pwm_drv" , "r");
		if (file) {
    			while (fscanf(file, "%s", str)!=EOF)
        			printf("%s\n",str);
				message[1]=str[4];
				message[2]=str[5];
				message[3]=str[6];
				message[4]=str[7];
    			fclose(file);
		}

		if( status >= 0 ){
//			message[1]=(speed/1000)%10 + '0';
//			message[2]=(speed/100)%10 + '0';
//			message[3]=(speed/10)%10 + '0';
//			message[4]=speed%10 + '0';
			message[8]=(accX/1000)%10 + '0';
			message[9]=(accX/100)%10 + '0';
			message[10]=(accX/10)%10 + '0';
			message[11]=accX%10 + '0';
			message[15]=(accY/1000)%10 + '0';
                        message[16]=(accY/100)%10 + '0';
                        message[17]=(accY/10)%10 + '0';
                        message[18]=accY%10 + '0';
			message[22]=(accZ/1000)%10 + '0';
                        message[23]=(accZ/100)%10 + '0';
                        message[24]=(accZ/10)%10 + '0';
                        message[25]=accZ%10 + '0';

                        status = write(s, message, 27);
                }

		// send a message
    		else{
			perror("uh oh");
                        printf("retry connection %d\n", status);
                        sleep(2);
                        close(s);
                        connect_bluetooth(&s, &status);
		}
	}
    	return 0;
}

int connect_bluetooth(int *s, int *status){
	struct sockaddr_rc addr = { 0 };
        char dest[18] = SERVER_ADDR;

        // allocate a socket
        *s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

        // set the connection parameters (who to connect to)
        addr.rc_family = AF_BLUETOOTH;
        addr.rc_channel = (uint8_t) 1;
        str2ba( dest, &addr.rc_bdaddr );

        // connect to server
        *status = connect(*s, (struct sockaddr *)&addr, sizeof(addr));
        return 0;
}

