/*
    Ben Birch

    v2.0 (hopefully final version)

    Jacdac Scanner for micro:bit v2

    This scans the bus for connected devices and prints their deivce class over serial.
    Maintains a list of connected devices, with a 2 second timeout for stale devices.


    Uses ZSingleWireSerial for 1Mbps single-wire communication due to hardware limitations using the built-in UART.
    Tried extensively using the built-in UART in half-duplex mode, but was unable to get reliable communication at 1Mbps,
    Maybe this is an issue with my code, or maybe the micro:bit v2 UART hardware is just not up to the task.

    Anyhow, ZSingleWireSerial works well for this purpose.

    This program assumes you are using the Jacdaptor which puts the data on pin P12.
    
    I was under the impression that you could simply sniff the bus in order to get the class names, but after talking to james,
    This wasnt the case and you have to send a request on the bus to get the class name. However, this was much more
    painless than i expected and it even worked first try. (skill not luck)


    UPDATE:

    I was wrong. It needed a full rewrite which took a full day.
    Basically, I wasnt getting the actual device ID in the response, i was misinterpreting a service packet, so it was the same for some devices
    which made me think it was working.

    Now, i have properly looked through codal-jacdac and implemented the protocol correctly. 
    Now, it scans all device IDs and requests the  product identifier for each one individually.



*/



#include "MicroBit.h"
#include "ZSingleWireSerial.h"

// Config
#define MAX_DEVICES         16
#define DEVICE_TIMEOUT_MS   5000
#define REQUEST_INTERVAL_MS 500
#define PRINT_INTERVAL_MS   1000

// Jacdac protocol constants
#define JD_CMD_ANNOUNCE     0x0000      // announce command identifier ( when a device announces itsself it will broadcast this )
#define JD_CMD_GET_PID      0x1181      // get product identifier command identifier
#define JD_FLAG_COMMAND     0x01        // the command flag, use this when sending a command e.g. get_product_identifier


MicroBit uBit;
ZSingleWireSerial *sws;


// Device struct for storage
struct Device { 
    uint64_t id; 
    uint32_t pid; 
    uint32_t lastSeen, lastReq; 
    bool havePid; 
};

Device devices[MAX_DEVICES];
int deviceCount = 0;

uint8_t rxBuf[256], txBuf[16];
volatile bool rxBusy = false, rxDone = false, txDone = false;

// Print hex values
void printHex64(uint64_t v) {
    const char *h = "0123456789ABCDEF";
    for (int i = 60; i >= 0; i -= 4) uBit.serial.printf("%c", h[(v >> i) & 0xF]);
}

void printHex32(uint32_t v) {
    const char *h = "0123456789ABCDEF";
    for (int i = 28; i >= 0; i -= 4) uBit.serial.printf("%c", h[(v >> i) & 0xF]);
}


// swiped this from codal-jacdac
uint16_t crc16(const uint8_t *d, int len) {
    uint16_t crc = 0xFFFF;
    while (len--) { uint8_t x = (crc >> 8) ^ *d++; x ^= x >> 4; crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x; }
    return crc;
}

// set up to listen for incoming packets
// aborts any ongoing DMA operations
// configures pin interrupts
// sets rxBusy/rxDone flags
void listen() {
    sws->abortDMA();
    sws->setMode(SingleWireDisconnected);
    sws->p.setDigitalValue(1);
    sws->p.getDigitalValue(PullMode::Up);
    sws->p.eventOn(DEVICE_PIN_INTERRUPT_ON_EDGE);
    rxBusy = rxDone = false;
}

// IRQ handler for SWS events
void onSwsEvent(uint16_t e) {
    if (e == SWS_EVT_DATA_RECEIVED || e == SWS_EVT_ERROR) rxDone = true;
    if (e == SWS_EVT_DATA_SENT) txDone = true;
}

// IRQ handler for falling edge on SWS pin
// starts a receive DMA if not already busy
void onFall(int v) {
    if (v || rxBusy) return;
    sws->p.eventOn(DEVICE_PIN_EVENT_NONE);
    rxBusy = true;
    sws->receiveDMA(rxBuf, sizeof(rxBuf));
}


// send a get_product_identifier request to the given targetId
// PACKET STRUCTURE:
/*
example from jacdac docs:

Bytes	Value	Offset	Size	Name	Description
568c	0x8c56	0	2	frame_crc	CRC
04	4	2	1	frame_size	Size of the data field in bytes.
01		3	1	frame_flags	Flags specific to this frame.
39dca0a9a6a1cad3		4	8	device_identifiter	64-bit device identifier
00	0	12	1	packet_size	The size of the payload field. Maximum size is 236 bytes.
00	0	13	1	service_index	A number that specifies an operation and code combination.
8111	0x1181	14	2	service_command	Identifier for the command

yeah this was painful i cant lie



*/


#define JD_FRAME_SIZE_OFFSET        2
#define JD_FLAG_OFFSET              3
#define JD_DEVICE_IDENTIFIER_OFFSET 4
#define JD_PACKET_SIZE_OFFSET       12
#define JD_SERVICE_INDEX_OFFSET     13
#define JD_SERVICE_COMMAND_OFFSET   14



void sendPidRequest(uint64_t targetId) {

    // build jacdac frame
    memset(txBuf, 0, 16);
    txBuf[JD_FRAME_SIZE_OFFSET] = 0x04;           
    txBuf[JD_FLAG_OFFSET] = JD_FLAG_COMMAND;
    memcpy(txBuf + JD_DEVICE_IDENTIFIER_OFFSET, &targetId, 8);
    txBuf[JD_SERVICE_COMMAND_OFFSET] = JD_CMD_GET_PID & 0xFF;
    txBuf[JD_SERVICE_COMMAND_OFFSET + 1] = JD_CMD_GET_PID >> 8;
    uint16_t c = crc16(txBuf + 2, 14);
    txBuf[0] = c & 0xFF;
    txBuf[1] = c >> 8;

    // prepare the bus
    sws->abortDMA();
    // disable interrupts while transmitting
    sws->p.eventOn(DEVICE_PIN_EVENT_NONE);
    rxBusy = rxDone = false;
    sws->setMode(SingleWireDisconnected);

    if (sws->p.getDigitalValue() == 0) { listen(); return; }

    // send break
    sws->p.setDigitalValue(0); target_wait_us(11);
    sws->p.setDigitalValue(1); target_wait_us(50);

    // send packet
    txDone = false;
    sws->sendDMA(txBuf, 16);
    uint32_t txStart = system_timer_current_time();
    while (!txDone && (system_timer_current_time() - txStart) < 10);
    if (!txDone) sws->abortDMA();

    // re-enable interrupts and set up to receive response
    sws->setMode(SingleWireDisconnected);
    sws->p.setDigitalValue(0); target_wait_us(11);
    sws->p.setDigitalValue(1);
    sws->p.getDigitalValue(PullMode::Up);
    sws->p.eventOn(DEVICE_PIN_INTERRUPT_ON_EDGE);
}

// this is called when we see a device announce itsslef
// gets timestamp, and sets its last seen
// if new device, instanciates a new device based on the id and adds to list
void sawDevice(uint64_t id) {
    if (!id) return;
    uint32_t now = system_timer_current_time();
    for (int i = 0; i < deviceCount; i++)
        if (devices[i].id == id) { devices[i].lastSeen = now; return; }
    if (deviceCount < MAX_DEVICES)
        devices[deviceCount++] = {id, 0, now, 0, false};
}

// process a received packet
// looks for announce packets and pid response packets
// updates device list accordingly
void processRx() {
    // find start of packet
    // there are some 0xF8 bytes at the start that we need to skip passt
    // these are idle bytes detected after the initial break signal for a few ms
    int off = 0;
    while (off < 4 && rxBuf[off] == 0xF8) off++;
    uint8_t *p = rxBuf + off;
    if (p[0] == 0 && p[2] == 0) return;


    // extract id and cmd from packet
    uint64_t id; 
    memcpy(&id, p + 4, 8);
    uint16_t cmd = p[14] | (p[15] << 8);

    // process based on cmd
    // if announce, call sawDevice
    // if get_pid response, extract pid and update device entry
    if (p[13] == 0 && cmd == JD_CMD_ANNOUNCE) sawDevice(id);
    else if (p[13] == 0 && cmd == JD_CMD_GET_PID) {
        uint32_t pid; memcpy(&pid, p + 16, 4);
        for (int i = 0; i < deviceCount; i++)
            if (devices[i].id == id) {
                devices[i].pid = pid;
                devices[i].havePid = true;
                devices[i].lastSeen = system_timer_current_time();
                break;
            }
    }
}

// send get_product_identifier requests to devices that we have seen but dont yet have a pid for
void trySendRequests() {
    if (rxBusy) return;
    uint32_t now = system_timer_current_time();
    for (int i = 0; i < deviceCount; i++) {
        if (
            devices[i].havePid || 
            now - devices[i].lastSeen > DEVICE_TIMEOUT_MS || 
            now - devices[i].lastReq < REQUEST_INTERVAL_MS
        ) continue;

        devices[i].lastReq = now;
        sendPidRequest(devices[i].id);
        fiber_sleep(2); // this is needed so that we dont try and read our own transmission. 2 is arbitrary but works
        return;
    }
}

// serial prints
void printDevices() {
    uBit.serial.printf("begin\r\n");
    uint32_t now = system_timer_current_time();
    for (int i = 0; i < deviceCount; i++) {
        if (now - devices[i].lastSeen > DEVICE_TIMEOUT_MS) continue;
        printHex64(devices[i].id);
        uBit.serial.printf("-");
        if (devices[i].havePid) printHex32(devices[i].pid);
        else uBit.serial.printf("unknown");
        uBit.serial.printf("\r\n");
    }
    uBit.serial.printf("end\r\n");
}



int main() {
    // init micro:bit runtime
    uBit.init();

    // setup serial for outputting list
    uBit.serial.setBaud(115200);

    // set up single wire serial.
    // cant just bit bang on the built in uart at 1Mbps reliably
    // believe me i tried.
    sws = new ZSingleWireSerial(uBit.io.P12);
    sws->setBaud(1000000);
    sws->p.setIRQ(onFall);
    sws->setIRQ(onSwsEvent);
    listen();

    uint32_t rxDeadline = 0, lastPrint = 0;

    while (1) {

        // handle receive timeout
        uint32_t now = system_timer_current_time();
        if (rxBusy && !rxDone) {
            if (!rxDeadline) rxDeadline = now + 20;
            else if (now > rxDeadline) { listen(); rxDeadline = 0; }
        }

        // process received packet
        if (rxDone) {
            rxDone = false;
            processRx();
            memset(rxBuf, 0, sizeof(rxBuf));
            listen();
            rxDeadline = 0;
        }

        // if any devices dont have a pid or an open request, try send one
        trySendRequests();

        // print device list periodically
        if (now - lastPrint > PRINT_INTERVAL_MS) {
            lastPrint = now;
            if (deviceCount) printDevices();
        }

        uBit.sleep(1);
    }
}
