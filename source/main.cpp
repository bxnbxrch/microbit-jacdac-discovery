/*
    Ben Birch

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




*/


#include "MicroBit.h"
#include "ZSingleWireSerial.h"


// main loop
#define RX_TIMEOUT_MS        5
#define PRINT_INTERVAL_MS    1000
#define MAIN_LOOP_SLEEP_MS   1

// jacdac frames
#define FRAME_FLAG           0xF8
#define MAX_FRAME_FLAGS      4

// jaccdac offsets
#define ID_OFFSET            4
#define SERVICE_INDEX_OFFSET 13
#define SERVICE_OP_OFFSET    14
#define NAME_OFFSET          16
#define DESC_OFFSET          20

// service operations
#define SVC_OP_ANNOUNCE              0x0000
#define SVC_OP_DEVICE_NAME_RESPONSE  0x8180




MicroBit uBit;
ZSingleWireSerial *sws;

// recieve buffer
uint8_t rxBuf[256];

// flags
volatile bool rxDone, rxBusy;

// max devices
#define MAX_DEVICES 16


// struct with id, desc, lastSeen, name to clean up code a bit
struct DeviceInfo {
    // UUID
    uint64_t id;
    
    // class name
    char name[32];

    // device descriptor
    uint32_t desc;

    // timestamps
    uint32_t lastSeen;
    uint32_t devReqTime;

    // has the get device class name request been sent
    bool devReqSent;
};


DeviceInfo devices[MAX_DEVICES];

int devCount = 0;
bool changed = false;


void listen();


// mbit serial doesnt like printing hex
// quick function to print hex
template<typename T>
void printHex(T val){
    const char hex[] = "0123456789ABCDEF";
    uint8_t *p = (uint8_t*)&val;
    for (int i = sizeof(T) - 1; i >= 0; i--)
        uBit.serial.printf("%c%c", hex[p[i] >> 4], hex[p[i] & 0xF]);
}

// this is called when we see any message from a device.
// if the device is new:
//  add it to the list

// if the deivice is known:
//  update its last seen time to maintain list
void sawDevice(uint64_t id, uint32_t desc) {
    if (!id) return;

    uint32_t now = system_timer_current_time();

    for (int i = 0; i < devCount; i++) {
        if (devices[i].id == id) {
            devices[i].lastSeen = now;
            if (desc) devices[i].desc = desc;
            return;
        }
    }

    if (devCount >= MAX_DEVICES) return;
    devices[devCount++] = { id, "", desc, now, 0, false };

    changed = true;
}


// print the list to serial
// also maintains the list by removing stale devices
// (those not seen for more than 2 seconds)
void printDevices() {
    uint32_t now = system_timer_current_time();
    int active = 0;
    for (int i = 0; i < devCount; i++)
        if (now - devices[i].lastSeen < 2000) active++;
    
    uBit.serial.printf("begin\r\n");
    for (int i = 0; i < devCount; i++) {
        if (now - devices[i].lastSeen < 2000) {
            if (devices[i].name[0]) {
                uBit.serial.printf("%s\r\n", devices[i].name);
            } else if (devices[i].desc) {
                printHex(devices[i].desc);
                uBit.serial.printf("\r\n");
            } else {
                printHex(devices[i].id);
                uBit.serial.printf("\r\n");
            }
        }
    }
    uBit.serial.printf("end\r\n");
}

// sends the request for the devices class name. This is the ID that is the same for devices that are the same
// e.g. two temperature sensors will have the same device class name
// This allows us to differentiate devices which have the same services but differnet footprints 
void sendGetDeviceClassName(uint64_t targetId) {
    uint8_t pkt[20] = {0};
    pkt[0] = 0x00; 
    pkt[1] = 0x00; 
    pkt[2] = 0x0C; 
    pkt[3] = 0x00; 
    pkt[12] = 0x00;
    pkt[13] = 0x00;
    pkt[14] = 0x80;
    pkt[15] = 0x01;

    memcpy(pkt + 4, &targetId, 8); // place the target id in the packet
    sws->sendDMA(pkt, 16);
}

// if a device doenst have a name, and iasnt stale, try send a request for its name
// only send one request every 500ms per device, and not if we are already waiting for a response
void trySendRequests() {
    uint32_t now = system_timer_current_time();
    if (rxBusy) return;
    for (int i = 0; i < devCount; i++) {
        if (!devices[i].name[0] && (now - devices[i].lastSeen < 2000)) {
            if (!devices[i].devReqSent || (now - devices[i].devReqTime > 500)) {
                devices[i].devReqSent = true;
                devices[i].devReqTime = now;
                sendGetDeviceClassName(devices[i].id);
                fiber_sleep(2);
                listen();
                break;
            }
        }
    }
}
// IRQ handler for when data is recieved. 
// this has to be as small as possible due to 1Mbps
// sets rxDone flag when a full packet is recieved
void onRx(uint16_t e) { if (e == SWS_EVT_DATA_RECEIVED || e == SWS_EVT_ERROR) rxDone = true; }


// IRQ handler for falling edge on the bus
// starts a DMA recieve into rxbuf
void onFall(int v) {
    if (v || rxBusy) return;
    sws->p.eventOn(DEVICE_PIN_EVENT_NONE);
    rxBusy = true;
    sws->receiveDMA(rxBuf, 256);
}

// set up to listen for packets
void listen() {

    // reset flags
    sws->abortDMA();
    
    // set up for next recieve
    sws->setMode(SingleWireDisconnected);
    sws->p.setDigitalValue(1);
    sws->p.getDigitalValue(PullMode::Up);
    sws->p.eventOn(DEVICE_PIN_INTERRUPT_ON_EDGE);
    rxBusy = rxDone = false;
}




int main() {

    // initialise and set up serial
    uBit.init();
    uBit.serial.setBaud(115200);
    uBit.serial.printf("\r\nJacdac Scanner\r\n");

    // set up single wire serial on pin P12
    sws = new ZSingleWireSerial(uBit.io.P12);
    
    // 1Mbps
    sws->setBaud(1000000);

    // setup IRQs
    sws->p.setIRQ(onFall);
    sws->setIRQ(onRx);

    // sniff
    listen();

    uint32_t rxTime = 0;
    uint32_t printTime = 0;

    while (1) {
        uint32_t now = system_timer_current_time();

        // rx timeout handling
        if (rxBusy && !rxDone) {
            if (!rxTime)
                rxTime = now + RX_TIMEOUT_MS;
            else if (now > rxTime) {
                listen();
                rxTime = 0;
            }
        }

        // rx packet processing
        if (rxDone) {
            rxDone = false;

            // skip jacadc frame flags 
            int off = 0;
            while (off < MAX_FRAME_FLAGS && rxBuf[off] == FRAME_FLAG)
                off++;

            uint8_t* pkt = rxBuf + off;

            uint8_t svcIdx = pkt[SERVICE_INDEX_OFFSET];
            uint16_t svcOp =
                pkt[SERVICE_OP_OFFSET] |
                (pkt[SERVICE_OP_OFFSET + 1] << 8);

            // Control service: device name response
            if (svcIdx == 0 && svcOp == SVC_OP_DEVICE_NAME_RESPONSE) {
                uint64_t id;
                memcpy(&id, pkt + ID_OFFSET, 8);

                for (int i = 0; i < devCount; i++) {
                    if (devices[i].id == id) {
                        const char* name = (const char*)(pkt + NAME_OFFSET);
                        int j = 0;
                        while (name[j] && j < 31) {
                            devices[i].name[j] = name[j];
                            j++;
                        }
                        devices[i].name[j] = 0;
                        changed = true;
                        break;
                    }
                }
            }
            // Control service: announce packet
            else if (svcIdx == 0 && svcOp == SVC_OP_ANNOUNCE) {
                uint64_t id;
                uint32_t desc;
                memcpy(&id,   pkt + ID_OFFSET,   8);
                memcpy(&desc, pkt + DESC_OFFSET, 4);
                sawDevice(id, desc);
            }

            memset(rxBuf, 0, sizeof(rxBuf));
            listen();
            rxTime = 0;
        }

        trySendRequests();

        // Print every 2 seconds
        if (changed || now - printTime > PRINT_INTERVAL_MS) {
            printTime = now;
            if (changed || devCount)
                printDevices();
            changed = false;
        }

        uBit.sleep(MAIN_LOOP_SLEEP_MS);
    }
}


