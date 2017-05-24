#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

int main(int argc, char **argv)
{
    struct sockaddr_rc addr = { 0 };
    int s, status;
    char dest[18] = "B8:27:EB:AF:FF:31";
//while(1){
    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( dest, &addr.rc_bdaddr );
//while(1){
    // connect to server
    status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
while(1){
    // send a message
    if( status == 0 ) {
        status = write(s, "exit", 6);
	printf("send\n");    
	}

    if( status < 0 ){
	 perror("uh oh");
	break;
	}
	sleep(5);
}    close(s);

    return 0;
}
