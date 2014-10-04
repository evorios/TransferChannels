/*
TMRh20 2014
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation.

evorios
Upgraded example "Transfer" for library RF24(TMRh20)

- changing channel on TX and RX synchronously every 16 second, total time is ~36 min
- added button for change timeout of flushing TX buffer
- displaying table with bitrates at channels after channel #127

See: https://github.com/TMRh20/RF24/tree/master/examples/Transfer
*/
#include <SPI.h>
#include "RF24.h"
#include "printf.h"

// all timing in microseconds

#define TX 1
#define RX 0
#define numCh 128
#define changeCh ((uint32_t)1<<24) // timer in us for change channel
#define say (changeCh>>3) // timer in us for logging, 8 times each channel
#define debounceTimeout ((uint32_t)1<<21) // debouncing for push button, ~2 sec
#define max_uint32_t ((uint32_t)-1)
/*************  USER Configuration *****************************/
// Hardware configuration
// Set up nRF24L01 radio on SPI bus plus pins 7 & 8
RF24 radio(7,8);
/***************************************************************/
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xE7E7E7E7E7LL, 0xC2C2C2C2C2LL };
uint32_t data[8];
uint8_t role=RX, ch=0;

void setup(void) {
	Serial.begin(115200);
	printf_begin();

	printf("\n\rRF24/examples/Transfer Rates/\n\r");

	// detect role through wire connected to pins # 5 and # 6 in the RX
	pinMode(5, OUTPUT);
	digitalWrite(5, 0); // connect to GND
	pinMode(6, INPUT_PULLUP); // connect to VCC via pullup resistor
	if (digitalRead(6)){ // detect VCC
		printf("transmit\n\r");
		role = TX;
	}
	else printf("receive\n\r"); // detect GND
	pinMode(6, INPUT);

	// Setup and configure rf radio
	radio.begin();

	radio.setChannel(ch);
	radio.setDataRate(RF24_2MBPS); // Set RF24_1MBPS or RF24_250KBPS, if you try support old packet format
	radio.setPALevel(RF24_PA_MAX);

	radio.setAutoAck(1); // Disable autoACK, if you try support old packet format
	radio.setRetries(15,0); // Disable Auto Retransmit by set ARC=0, if you try support old packet format, enable autoACK for use it
	radio.enableDynamicAck(); // It need for enable of using of NO_ACK bit flag in the packet, enable autoACK for use it

	radio.setAddressWidth(5); // address width (3 bytes not recommended)
	radio.setPayloadSize(32); // payload size of static packets (does not affect the dynamic packets)
	//radio.enableDynamicPayloads(); // enable various length of payload (dynamic packets), enable autoACK for use it
	//radio.enableAckPayload(); // enable dynamic creation and sending of custom ACK playload (and various length of payload?), enable autoACK for use it
	radio.setCRCLength(RF24_CRC_16); // CRC length for packet, disabling CRC not recommended

	radio.openWritingPipe(pipes[role]);
	radio.openReadingPipe(1,pipes[1-role]);
	
	radio.printDetails(); // Dump the configuration of the rf unit for debugging
	if (!role) radio.startListening(); // Start listening
	else {
		//randomSeed(analogRead(0)); //Seed for random number generation
		for(int i=0; i<8; i++) data[i] = max_uint32_t; //random(0xFFFFFFFF);
	}
}

void increaseFlushTxTimer(void){ // push button, which increases the pause between flushing
	if (micros() - data[3] > debounceTimeout) data[2] += 200;
	data[3] = micros();
}

void loop(void){
	if(role){ //TX
		uint32_t now, changeTime = changeCh<<1, sayTime = 0, pauseTime;
		// At channel #0 TX will stay ~32 seconds, then ~16 seconds per channel
		data[0] = 0;
		data[2] = 3000; //flushTxTimer
		data[3] = 0; //last pushing

		pinMode(3, INPUT_PULLUP); // connect a button to pin #3 and GND
		attachInterrupt(1, increaseFlushTxTimer, RISING);
		printf("radio.transmit\n\r");
		now = micros();
		pauseTime = micros();

		for(;;){
			if(now >= changeTime){ // time for change channel
				if ((++ch) >= numCh){
					printf("\n\rstop transmit\n\r");
					for(;;);
				}
				radio.setChannel(ch);
				changeTime += changeCh;
				printf("ch: %u\n\r", ch);
			}

			if (now >= sayTime){
				data[1] = changeTime - now; // synchronization
				sayTime += say;
			}

			cli(); // disable button until write to SPI
			data[0] += radio.writeFast(&data,32,1);
			sei();

			now = micros();
			if(now - pauseTime >= data[2]){
				pauseTime = now;
				radio.txStandBy(); // Need to drop out of TX mode every 4ms if sending a steady stream of multicast data
			}
		}
	}

	else{ //RX
		uint32_t now, sayTime=micros(), changeTxTime=max_uint32_t, changeRxTime=max_uint32_t, startRx=0, startTime=micros();
		uint16_t bitRates[numCh], bitRate, received=0, sended, packetSize;
		packetSize = 329; // max packet size in bits, see datasheet
		bitRates[ch] = 0;

		for(;;){
			if(radio.available()){
				do{
					radio.read(&data,32);
					++received;
				}while(radio.available());

				if(data[1] < changeTxTime) {
					changeTxTime = data[1];
					sayTime = micros(); // synchronization message log
					changeRxTime = sayTime + changeTxTime; // synchronization of change channel
					if(received < 100) sayTime += say;
				}
			}

			now = micros();
			if(now >= changeRxTime){ // time for change channel
				if ((++ch) >= numCh){
					printf("\n\r\n\rCh\tRate\n\r");
					for(uint8_t i=0; i < numCh; ++i) printf("%d\t%u\n\r", i, bitRates[i]);
					for(;;);
				}
				radio.setChannel(ch);
				bitRates[ch] = 0;
				startTime = micros();
				sayTime = startTime + say;
				startRx = data[0];
				received = 0;
				changeRxTime += changeCh;
				changeTxTime = max_uint32_t;
			}

			if(now >= sayTime){ // time for post in message log
				sended = data[0] - startRx;
				bitRate = (received*packetSize*1000)/(now - startTime);
				if (bitRates[ch] < bitRate) bitRates[ch] = bitRate;
				printf("%u kbit/s,\t%u%% received,\t%u sended,\t%d ch\t%lu us\n\r", bitRate, received*100/sended, sended, ch, data[2]);
				startTime = now;
				sayTime += say;
				startRx = data[0];
				received = 0;
			}
		}
	}
}