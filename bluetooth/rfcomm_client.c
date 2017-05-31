/*
 * File: rfcomm_client.c
 * version: 1.2
 */
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#define SERVER_ADDR "B8:27:EB:5D:7B:C9"

int connect_bluetooth(int* s, int* status);

int main(int argc, char **argv){
	int s, status;
        char message[]="hello:000";
        int i = 0;

//	while(1){
//		int s, status;
//		char message[]="hello:000";
//		int i;

	connect_bluetooth(&s, &status);
	/*while(1){
                // send a message
                if( status == 0 ){
                        message[6]=(i/100)%10 + '0';
                        message[7]=(i/10)%10 + '0';
                        message[8]=i%10 + '0';
                        i++;
                        status = write(s, message, 10);
                        //status = write(s, "hello2", 8);
                }
                //Retry connecting
//              while(status < 0){
//                      printf("retry connection %d\n", status);
//                      sleep(2);
//                      close(s);
//                      connect_bluetooth(&s, &status);
//              }

                if( status < 0 ) perror("uh oh");{
                        printf("retry connection %d\n", status);
                        sleep(2);
                        close(s);
                        connect_bluetooth(&s, &status);
                }
        }*/

	while(1){
		if( status >= 0 ){
			message[6]=(i/100)%10 + '0';
                        message[7]=(i/10)%10 + '0';
                        message[8]=i%10 + '0';
                        i++;
                        status = write(s, message, 10);
                        //status = write(s, "hello2", 8);
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

