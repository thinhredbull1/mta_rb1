#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define ON 1
#define OFF 0
const byte address[6] = "00001";
RF24 radio(48,53); //CE, CSN
byte msg[2];
void setup_NRF(){ 
    //============================================================Module NRF24
    radio.begin();                     
    // radio.setAutoAck(1);               
    // radio.setRetries(1,1);             
    // radio.setDataRate(RF24_1MBPS);    // Tốc độ truyền
    // radio.setPALevel(RF24_PA_MAX);      // Dung lượng tối đa
    // radio.setChannel(10);               // Đặt kênh
    // radio.openWritingPipe(pipe);        // mở kênh
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
}
    
void sendMsg(bool on_off){
    if(ON)msg[0]=0x01;
    else msg[0]=0x00;
    radio.write(&msg, sizeof(msg));
}