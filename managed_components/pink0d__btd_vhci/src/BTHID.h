/* Copyright (C) 2013 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#ifndef _bthid_h_
#define _bthid_h_

#include "BTD.h"
#include "hidboot.h"

#define KEYBOARD_PARSER_ID      0
#define MOUSE_PARSER_ID         1
#define NUM_PARSERS             2

/** This BluetoothService class implements support for Bluetooth HID devices. */
class BTHID : public BluetoothService {
public:
        /**
         * Constructor for the BTHID class.
         * @param  p   Pointer to the BTD class instance.
         * @param  pair   Set this to true in order to pair with the device. If the argument is omitted then it will not pair with it. One can use ::PAIR to set it to true.
         * @param  pin   Write the pin to BTD#btdPin. If argument is omitted, then "0000" will be used.
         */
        BTHID(bool pair = false, const char *pin = "0000");

        /** @name BluetoothService implementation */
        /** Used this to disconnect the devices. */
        void disconnect();
        /**@}*/

        /**
         * Get HIDReportParser.
         * @param  id ID of parser.
         * @return    Returns the corresponding HIDReportParser. Returns NULL if id is not valid.
         */
        HIDReportParser *GetReportParser(uint8_t id) {
                if (id >= NUM_PARSERS)
                        return NULL;
                return pRptParser[id];
        };

        /**
         * Set HIDReportParser to be used.
         * @param  id  Id of parser.
         * @param  prs Pointer to HIDReportParser.
         * @return     Returns true if the HIDReportParser is set. False otherwise.
         */
        bool SetReportParser(uint8_t id, HIDReportParser *prs) {
                if (id >= NUM_PARSERS)
                        return false;
                pRptParser[id] = prs;
                return true;
        };

        /**
         * Set HID protocol mode.
         * @param mode HID protocol to use. Either USB_HID_BOOT_PROTOCOL or HID_RPT_PROTOCOL.
         */
        void setProtocolMode(uint8_t mode) {
                protocolMode = mode;
        };

        /**@{*/
        /**
         * Used to set the leds on a keyboard.
         * @param data See ::KBDLEDS in hidboot.h
         */
        void setLeds(struct KBDLEDS data) {
                setLeds(*((uint8_t*)&data));
        };
        void setLeds(uint8_t data);
        /**@}*/

        /** True if a device is connected */
        bool connected;

        /** Call this function to allow BTD connect to saved device */
        void connect(uint8_t *saved_bdaddr) {
                if(pBtd) {
                        for (uint8_t i=0; i<6; i++)
                                pBtd->saved_bdaddr[i] = saved_bdaddr[i];
                }
        }
        /** Call this to start the pairing sequence with a device */
        void pair(void) {
                if(pBtd)
                        pBtd->pairWithHID();
        };
        /** Get Bluetooth address of connected device */
        void getAddr(uint8_t *addr) {
                if(pBtd) {
                        for (uint8_t i=0; i<6; i++)
                                addr[i] = pBtd->disc_bdaddr[i];
                }       
        }

        /**
         * Used to get the micros() of the last Bluetooth DATA input report received
         * This can be used detect if the connection to a Bluetooth device is lost fx if the battery runs out or if it gets out of range.
         * @return      Timestamp in microseconds of the last Bluetooth DATA input report received
         */
        uint64_t getLastMessageTime() {
                return lastBtDataInputIntMicros;
        };

protected:
        /** @name BluetoothService implementation */
        /**
         * Used to pass acldata to the services.
         * @param ACLData Incoming acldata.
         */
        void ACLData(uint8_t* ACLData);
        /** Used to run part of the state machine. */
        void Run();
        /** Use this to reset the service. */
        void Reset();
        /**
         * Called when a device is successfully initialized.
         * Use attachOnInit(void (*funcOnInit)(void)) to call your own function.
         * This is useful for instance if you want to set the LEDs in a specific way.
         */
        void onInit() {
                if(pFuncOnInit)
                        pFuncOnInit(); // Call the user function
                OnInitBTHID();
        };
        /**@}*/

        /** @name Overridable functions */
        /**
         * Used to parse Bluetooth HID data to any class that inherits this class.
         * @param len The length of the incoming data.
         * @param buf Pointer to the data buffer.
         */
        virtual void ParseBTHIDData(uint8_t len __attribute__((unused)), uint8_t *buf __attribute__((unused))) {
                return;
        };
        /**
         * Same as ParseBTHIDData for reports that are sent through the
         * interrupt pipe (in response to a GET_REPORT).
         */
        virtual void ParseBTHIDControlData(uint8_t len __attribute__((unused)), uint8_t *buf __attribute__((unused))) {
                return;
        }
        /** Called when a device is connected */
        virtual void OnInitBTHID() {
                return;
        };
        /** Used to reset any buffers in the class that inherits this */
        virtual void ResetBTHID() {
                return;
        }
        /**@}*/

        /** L2CAP source CID for HID_Control */
        uint8_t control_scid[2];

        /** L2CAP source CID for HID_Interrupt */
        uint8_t interrupt_scid[2];

        uint8_t l2cap_sdp_state;
        uint8_t sdp_scid[2]; // L2CAP source CID for SDP

private:
        HIDReportParser *pRptParser[NUM_PARSERS]; // Pointer to HIDReportParsers.

        uint8_t l2capoutbuf[BULK_MAXPKTSIZE]; // General purpose buffer for l2cap out data
        void SDP_Command(uint8_t* data, uint8_t nbytes);
        void serviceNotSupported(uint8_t transactionIDHigh, uint8_t transactionIDLow);

        /** Set report protocol. */
        void setProtocol();
        uint8_t protocolMode;

        void SDP_task();
        void L2CAP_task(); // L2CAP state machine

        bool activeConnection; // Used to indicate if it already has established a connection
        bool SDPConnected;

        /* Variables used for L2CAP communication */
        uint8_t control_dcid[2]; // L2CAP device CID for HID_Control - Always 0x0070
        uint8_t interrupt_dcid[2]; // L2CAP device CID for HID_Interrupt - Always 0x0071
        uint8_t sdp_dcid[2];
        uint8_t l2cap_state;

        uint64_t lastBtDataInputIntMicros; // Variable used to store the micros value of the last Bluetooth DATA input report received
};
#endif
