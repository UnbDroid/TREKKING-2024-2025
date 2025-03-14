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

#include "BTHID.h"

#include "esp_timer.h"

static const char *LOG_TAG = "BTHID";

BTHID::BTHID(bool pair, const char *pin) :
BluetoothService(BTD::instance()), // Pointer to USB class instance - mandatory
protocolMode(USB_HID_BOOT_PROTOCOL) {
        for(uint8_t i = 0; i < NUM_PARSERS; i++)
                pRptParser[i] = NULL;

        pBtd->pairWithHIDDevice = pair;
        pBtd->btdPin = pin;

        /* Set device cid for the control and intterrupt channelse - LSB */
        sdp_dcid[0] = 0x50; // 0x0050
        sdp_dcid[1] = 0x00;
        control_dcid[0] = 0x70; // 0x0070
        control_dcid[1] = 0x00;
        interrupt_dcid[0] = 0x71; // 0x0071
        interrupt_dcid[1] = 0x00;

        Reset();
}

void BTHID::Reset() {
        connected = false;
        activeConnection = false;
        SDPConnected = false;
        l2cap_event_flag = 0; // Reset flags
        l2cap_sdp_state = L2CAP_SDP_WAIT;
        l2cap_state = L2CAP_WAIT;
        ResetBTHID();
}

void BTHID::disconnect() { // Use this void to disconnect the device
        if(SDPConnected)
                pBtd->l2cap_disconnection_request(hci_handle, ++identifier, sdp_scid, sdp_dcid);
        // First the HID interrupt channel has to be disconnected, then the HID control channel and finally the HCI connection
        pBtd->l2cap_disconnection_request(hci_handle, ++identifier, interrupt_scid, interrupt_dcid);
        Reset();
        l2cap_sdp_state = L2CAP_DISCONNECT_RESPONSE;
        l2cap_state = L2CAP_INTERRUPT_DISCONNECT;
}

void BTHID::ACLData(uint8_t* l2capinbuf) {
        if(!connected) {
                if(l2capinbuf[8] == L2CAP_CMD_CONNECTION_REQUEST) {
                        if((l2capinbuf[12] | (l2capinbuf[13] << 8)) == SDP_PSM && !pBtd->sdpConnectionClaimed) {
                                pBtd->sdpConnectionClaimed = true;
                                hci_handle = pBtd->hci_handle; // Store the HCI Handle for the connection
                                l2cap_sdp_state = L2CAP_SDP_WAIT; // Reset state
                        }
                }
        }

        if(!pBtd->l2capConnectionClaimed && pBtd->incomingHIDDevice && !connected && !activeConnection) {
                if(l2capinbuf[8] == L2CAP_CMD_CONNECTION_REQUEST) {
                        if((l2capinbuf[12] | (l2capinbuf[13] << 8)) == HID_CTRL_PSM) {
                                pBtd->incomingHIDDevice = false;
                                pBtd->l2capConnectionClaimed = true; // Claim that the incoming connection belongs to this service
                                activeConnection = true;
                                hci_handle = pBtd->hci_handle; // Store the HCI Handle for the connection
                                l2cap_state = L2CAP_WAIT;
                        }
                }
        }

        if(checkHciHandle(l2capinbuf, hci_handle)) { // acl_handle_ok
                if((l2capinbuf[6] | (l2capinbuf[7] << 8)) == 0x0001U) { // l2cap_control - Channel ID for ACL-U
                        if(l2capinbuf[8] == L2CAP_CMD_COMMAND_REJECT) {

                                ESP_LOGD(LOG_TAG, "L2CAP Command Rejected - Reason: %02X%02X%02X%02X%02X%02X", l2capinbuf[13],l2capinbuf[12],l2capinbuf[17],l2capinbuf[16],l2capinbuf[15],l2capinbuf[14]);

                        } else if(l2capinbuf[8] == L2CAP_CMD_CONNECTION_RESPONSE) {
                                if(((l2capinbuf[16] | (l2capinbuf[17] << 8)) == 0x0000) && ((l2capinbuf[18] | (l2capinbuf[19] << 8)) == SUCCESSFUL)) { // Success
                                        if(l2capinbuf[14] == sdp_dcid[0] && l2capinbuf[15] == sdp_dcid[1]) {

                                                ESP_LOGV(LOG_TAG, "SDP Connection Complete");

                                                identifier = l2capinbuf[9];
                                                sdp_scid[0] = l2capinbuf[12];
                                                sdp_scid[1] = l2capinbuf[13];

                                                ESP_LOGD(LOG_TAG, "Send SDP Config Request");

                                                identifier++;
                                                pBtd->l2cap_config_request(hci_handle, identifier, sdp_scid);
                                        } else if(l2capinbuf[14] == control_dcid[0] && l2capinbuf[15] == control_dcid[1]) {

                                                ESP_LOGV(LOG_TAG, "HID Control Connection Complete");

                                                identifier = l2capinbuf[9];
                                                control_scid[0] = l2capinbuf[12];
                                                control_scid[1] = l2capinbuf[13];
                                                l2cap_set_flag(L2CAP_FLAG_CONTROL_CONNECTED);
                                        } else if(l2capinbuf[14] == interrupt_dcid[0] && l2capinbuf[15] == interrupt_dcid[1]) {

                                                ESP_LOGV(LOG_TAG, "HID Interrupt Connection Complete");

                                                identifier = l2capinbuf[9];
                                                interrupt_scid[0] = l2capinbuf[12];
                                                interrupt_scid[1] = l2capinbuf[13];
                                                l2cap_set_flag(L2CAP_FLAG_INTERRUPT_CONNECTED);
                                        }
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_CONNECTION_REQUEST) {

                                ESP_LOGV(LOG_TAG, "L2CAP Connection Request - PSM: %02X%02X SCID: %02X%02X Identifier:%02X", l2capinbuf[13],l2capinbuf[12],l2capinbuf[15],l2capinbuf[14],l2capinbuf[9]);

                                if((l2capinbuf[12] | (l2capinbuf[13] << 8)) == SDP_PSM) {
                                        identifier = l2capinbuf[9];
                                        sdp_scid[0] = l2capinbuf[14];
                                        sdp_scid[1] = l2capinbuf[15];
                                        l2cap_set_flag(L2CAP_FLAG_CONNECTION_SDP_REQUEST);
                                } else if((l2capinbuf[12] | (l2capinbuf[13] << 8)) == HID_CTRL_PSM) {
                                        identifier = l2capinbuf[9];
                                        control_scid[0] = l2capinbuf[14];
                                        control_scid[1] = l2capinbuf[15];
                                        l2cap_set_flag(L2CAP_FLAG_CONNECTION_CONTROL_REQUEST);
                                } else if((l2capinbuf[12] | (l2capinbuf[13] << 8)) == HID_INTR_PSM) {
                                        identifier = l2capinbuf[9];
                                        interrupt_scid[0] = l2capinbuf[14];
                                        interrupt_scid[1] = l2capinbuf[15];
                                        l2cap_set_flag(L2CAP_FLAG_CONNECTION_INTERRUPT_REQUEST);
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_CONFIG_RESPONSE) {
                                if((l2capinbuf[16] | (l2capinbuf[17] << 8)) == 0x0000) { // Success
                                        if(l2capinbuf[12] == sdp_dcid[0] && l2capinbuf[13] == sdp_dcid[1]) {

                                                ESP_LOGV(LOG_TAG, "SDP Configuration Complete");

                                                identifier = l2capinbuf[9];
                                                l2cap_set_flag(L2CAP_FLAG_CONFIG_SDP_SUCCESS);
                                        } else if(l2capinbuf[12] == control_dcid[0] && l2capinbuf[13] == control_dcid[1]) {

                                                ESP_LOGV(LOG_TAG, "HID Control Configuration Complete");

                                                identifier = l2capinbuf[9];
                                                l2cap_set_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS);
                                        } else if(l2capinbuf[12] == interrupt_dcid[0] && l2capinbuf[13] == interrupt_dcid[1]) {

                                                ESP_LOGV(LOG_TAG, "HID Interrupt Configuration Complete");

                                                identifier = l2capinbuf[9];
                                                l2cap_set_flag(L2CAP_FLAG_CONFIG_INTERRUPT_SUCCESS);
                                        }
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_CONFIG_REQUEST) {
                                if(l2capinbuf[12] == sdp_dcid[0] && l2capinbuf[13] == sdp_dcid[1]) {

                                        ESP_LOGV(LOG_TAG, "SDP Configuration Request");

                                        pBtd->l2cap_config_response(hci_handle, l2capinbuf[9], sdp_scid);
                                } else if(l2capinbuf[12] == control_dcid[0] && l2capinbuf[13] == control_dcid[1]) {

                                        ESP_LOGV(LOG_TAG, "HID Control Configuration Request");

                                        pBtd->l2cap_config_response(hci_handle, l2capinbuf[9], control_scid);
                                } else if(l2capinbuf[12] == interrupt_dcid[0] && l2capinbuf[13] == interrupt_dcid[1]) {

                                        ESP_LOGV(LOG_TAG, "HID Interrupt Configuration Request");

                                        pBtd->l2cap_config_response(hci_handle, l2capinbuf[9], interrupt_scid);
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_DISCONNECT_REQUEST) {
                                if(l2capinbuf[12] == sdp_dcid[0] && l2capinbuf[13] == sdp_dcid[1]) {

                                        ESP_LOGD(LOG_TAG, "Disconnect Request: SDP Channel");

                                        identifier = l2capinbuf[9];
                                        l2cap_set_flag(L2CAP_FLAG_DISCONNECT_SDP_REQUEST);
                                } else if(l2capinbuf[12] == control_dcid[0] && l2capinbuf[13] == control_dcid[1]) {

                                        ESP_LOGD(LOG_TAG, "Disconnect Request: Control Channel");

                                        identifier = l2capinbuf[9];
                                        pBtd->l2cap_disconnection_response(hci_handle, identifier, control_dcid, control_scid);
                                        Reset();
                                } else if(l2capinbuf[12] == interrupt_dcid[0] && l2capinbuf[13] == interrupt_dcid[1]) {

                                        ESP_LOGD(LOG_TAG, "Disconnect Request: Interrupt Channel");

                                        identifier = l2capinbuf[9];
                                        pBtd->l2cap_disconnection_response(hci_handle, identifier, interrupt_dcid, interrupt_scid);
                                        Reset();
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_DISCONNECT_RESPONSE) {
                                if(l2capinbuf[12] == sdp_scid[0] && l2capinbuf[13] == sdp_scid[1]) {

                                        ESP_LOGV(LOG_TAG, "Disconnect Response: SDP Channel");

                                        identifier = l2capinbuf[9];
                                        l2cap_set_flag(L2CAP_FLAG_DISCONNECT_RESPONSE);
                                } else if(l2capinbuf[12] == control_scid[0] && l2capinbuf[13] == control_scid[1]) {

                                        ESP_LOGV(LOG_TAG, "Disconnect Response: Control Channel");

                                        identifier = l2capinbuf[9];
                                        l2cap_set_flag(L2CAP_FLAG_DISCONNECT_CONTROL_RESPONSE);
                                } else if(l2capinbuf[12] == interrupt_scid[0] && l2capinbuf[13] == interrupt_scid[1]) {

                                        ESP_LOGV(LOG_TAG, "Disconnect Response: Interrupt Channel");

                                        identifier = l2capinbuf[9];
                                        l2cap_set_flag(L2CAP_FLAG_DISCONNECT_INTERRUPT_RESPONSE);
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_INFORMATION_REQUEST) {

                                ESP_LOGD(LOG_TAG, "Information request");

                                identifier = l2capinbuf[9];
                                pBtd->l2cap_information_response(hci_handle, identifier, l2capinbuf[12], l2capinbuf[13]);
                        }

                        else {
                                identifier = l2capinbuf[9];
                                ESP_LOGV(LOG_TAG, "L2CAP Unknown Signaling Command: 0x%02X", l2capinbuf[8]);
                        }

                } else if(l2capinbuf[6] == sdp_dcid[0] && l2capinbuf[7] == sdp_dcid[1]) { // SDP
                        if(l2capinbuf[8] == SDP_SERVICE_SEARCH_REQUEST) {

                                ESP_LOGV(LOG_TAG, "SDP_SERVICE_SEARCH_REQUEST");

                                // Send response
                                l2capoutbuf[0] = SDP_SERVICE_SEARCH_RESPONSE;
                                l2capoutbuf[1] = l2capinbuf[9];//transactionIDHigh;
                                l2capoutbuf[2] = l2capinbuf[10];//transactionIDLow;

                                l2capoutbuf[3] = 0x00; // MSB Parameter Length
                                l2capoutbuf[4] = 0x05; // LSB Parameter Length = 5

                                l2capoutbuf[5] = 0x00; // MSB TotalServiceRecordCount
                                l2capoutbuf[6] = 0x00; // LSB TotalServiceRecordCount = 0

                                l2capoutbuf[7] = 0x00; // MSB CurrentServiceRecordCount
                                l2capoutbuf[8] = 0x00; // LSB CurrentServiceRecordCount = 0

                                l2capoutbuf[9] = 0x00; // No continuation state

                                SDP_Command(l2capoutbuf, 10);
                        } else if(l2capinbuf[8] == SDP_SERVICE_ATTRIBUTE_REQUEST) {

                                ESP_LOGV(LOG_TAG, "SDP_SERVICE_ATTRIBUTE_REQUEST");

                                // Send response
                                l2capoutbuf[0] = SDP_SERVICE_ATTRIBUTE_RESPONSE;
                                l2capoutbuf[1] = l2capinbuf[9];//transactionIDHigh;
                                l2capoutbuf[2] = l2capinbuf[10];//transactionIDLow;

                                l2capoutbuf[3] = 0x00; // MSB Parameter Length
                                l2capoutbuf[4] = 0x05; // LSB Parameter Length = 5

                                l2capoutbuf[5] = 0x00; // MSB AttributeListByteCount
                                l2capoutbuf[6] = 0x02; // LSB AttributeListByteCount = 2

                                // TODO: What to send?
                                l2capoutbuf[7] = 0x35; // Data element sequence - length in next byte
                                l2capoutbuf[8] = 0x00; // Length = 0

                                l2capoutbuf[9] = 0x00; // No continuation state

                                SDP_Command(l2capoutbuf, 10);
                        } else if(l2capinbuf[8] == SDP_SERVICE_SEARCH_ATTRIBUTE_REQUEST) {

                                ESP_LOGV(LOG_TAG, "SDP_SERVICE_SEARCH_ATTRIBUTE_REQUEST ");
                                uint16_t uuid;
                                if((l2capinbuf[16] << 8 | l2capinbuf[17]) == 0x0000) // Check if it's sending the UUID as a 128-bit UUID
                                        uuid = (l2capinbuf[18] << 8 | l2capinbuf[19]);
                                else // Short UUID
                                        uuid = (l2capinbuf[16] << 8 | l2capinbuf[17]);
                                ESP_LOGV(LOG_TAG, "UUID: 0x%04x", uuid);

                                uint16_t length = l2capinbuf[11] << 8 | l2capinbuf[12];
                                ESP_LOGV(LOG_TAG, "Data Length: %d", length);                                

                                serviceNotSupported(l2capinbuf[9], l2capinbuf[10]); // The service is not supported
                        }

                        else {
                                ESP_LOGV(LOG_TAG, "Unknown PDU: 0x%02X", l2capinbuf[8]);
                        }

                } else if(l2capinbuf[6] == interrupt_dcid[0] && l2capinbuf[7] == interrupt_dcid[1]) { // l2cap_interrupt

                        ESP_LOGV(LOG_TAG, "L2CAP Interrupt: ");

                        if(l2capinbuf[8] == 0xA1) { // HID BT DATA (0xA0) | Report Type (Input 0x01)
                                lastBtDataInputIntMicros = esp_timer_get_time();

                                uint16_t length = ((uint16_t)l2capinbuf[5] << 8 | l2capinbuf[4]);
                                ParseBTHIDData((uint8_t)(length - 1), &l2capinbuf[9]); // First byte will be the report ID

                                switch(l2capinbuf[9]) { // Report ID
                                        case 0x01: // Keyboard or Joystick events
                                                if(pRptParser[KEYBOARD_PARSER_ID])
                                                        pRptParser[KEYBOARD_PARSER_ID]->Parse(0, (uint8_t)(length - 2), &l2capinbuf[10]); // Use reinterpret_cast again to extract the instance
                                                break;

                                        case 0x02: // Mouse events
                                                if(pRptParser[MOUSE_PARSER_ID])
                                                        pRptParser[MOUSE_PARSER_ID]->Parse(0, (uint8_t)(length - 2), &l2capinbuf[10]); // Use reinterpret_cast again to extract the instance
                                                break;

                                        default:
                                                ESP_LOGV(LOG_TAG, "Unknown Report type: %02X", l2capinbuf[9]);
                                                break;

                                }
                        } else {

                                ESP_LOGV(LOG_TAG, "Unhandled L2CAP interrupt report: 0x%02X", l2capinbuf[8]);

                        }
                } else if(l2capinbuf[6] == control_dcid[0] && l2capinbuf[7] == control_dcid[1]) { // l2cap_control

                        ESP_LOGV(LOG_TAG, "L2CAP Control: ");

                        if(l2capinbuf[8] == 0xA3) { // HID BT DATA (0xA0) | Report Type (Feature 0x03)
                                uint16_t length = ((uint16_t)l2capinbuf[5] << 8 | l2capinbuf[4]);
                                ParseBTHIDControlData((uint8_t)(length - 1), &l2capinbuf[9]); // First byte will be the report ID
                        } else {

                                ESP_LOGV(LOG_TAG, "Unhandled L2CAP control report: 0x%02X", l2capinbuf[8]);

                        }
                }

                else {
                        ESP_LOGV(LOG_TAG, "Unsupported L2CAP Data - Channel ID: 0x%02X 0x%02X", l2capinbuf[7], l2capinbuf[6]);
                }

                SDP_task();
                L2CAP_task();
        }
}

void BTHID::SDP_task() {
        switch(l2cap_sdp_state) {
                case L2CAP_SDP_WAIT:
                        if(l2cap_check_flag(L2CAP_FLAG_CONNECTION_SDP_REQUEST)) {
                                l2cap_clear_flag(L2CAP_FLAG_CONNECTION_SDP_REQUEST); // Clear flag

                                ESP_LOGD(LOG_TAG, "SDP Incoming Connection Request");

                                pBtd->l2cap_connection_response(hci_handle, identifier, sdp_dcid, sdp_scid, PENDING);
                                //delay(1);
                                pBtd->l2cap_connection_response(hci_handle, identifier, sdp_dcid, sdp_scid, SUCCESSFUL);
                                identifier++;
                                //delay(1);
                                pBtd->l2cap_config_request(hci_handle, identifier, sdp_scid);
                                l2cap_sdp_state = L2CAP_SDP_SUCCESS;
                        } else if(l2cap_check_flag(L2CAP_FLAG_DISCONNECT_SDP_REQUEST)) {
                                l2cap_clear_flag(L2CAP_FLAG_DISCONNECT_SDP_REQUEST); // Clear flag
                                SDPConnected = false;

                                ESP_LOGD(LOG_TAG, "Disconnected SDP Channel");

                                pBtd->l2cap_disconnection_response(hci_handle, identifier, sdp_dcid, sdp_scid);
                        }
                        break;
                case L2CAP_SDP_SUCCESS:
                        if(l2cap_check_flag(L2CAP_FLAG_CONFIG_SDP_SUCCESS)) {
                                l2cap_clear_flag(L2CAP_FLAG_CONFIG_SDP_SUCCESS); // Clear flag

                                ESP_LOGD(LOG_TAG, "SDP Successfully Configured");

                                SDPConnected = true;
                                l2cap_sdp_state = L2CAP_SDP_WAIT;
                        }
                        break;

                case L2CAP_DISCONNECT_RESPONSE: // This is for both disconnection response from the RFCOMM and SDP channel if they were connected
                        if(l2cap_check_flag(L2CAP_FLAG_DISCONNECT_RESPONSE)) {

                                ESP_LOGD(LOG_TAG, "Disconnected L2CAP Connection");

                                pBtd->hci_disconnect(hci_handle);
                                hci_handle = -1; // Reset handle
                                Reset();
                        }
                        break;
        }
}

void BTHID::L2CAP_task() {
        switch(l2cap_state) {
                        /* These states are used if the HID device is the host */
                case L2CAP_CONTROL_SUCCESS:
                        if(l2cap_check_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS)) {

                                ESP_LOGD(LOG_TAG, "HID Control Successfully Configured");

                                setProtocol(); // Set protocol before establishing HID interrupt channel
                                l2cap_state = L2CAP_INTERRUPT_SETUP;
                        }
                        break;

                case L2CAP_INTERRUPT_SETUP:
                        if(l2cap_check_flag(L2CAP_FLAG_CONNECTION_INTERRUPT_REQUEST)) {

                                ESP_LOGD(LOG_TAG, "HID Interrupt Incoming Connection Request");

                                pBtd->l2cap_connection_response(hci_handle, identifier, interrupt_dcid, interrupt_scid, PENDING);
                                //delay(1);
                                pBtd->l2cap_connection_response(hci_handle, identifier, interrupt_dcid, interrupt_scid, SUCCESSFUL);
                                identifier++;
                                //delay(1);
                                pBtd->l2cap_config_request(hci_handle, identifier, interrupt_scid);

                                l2cap_state = L2CAP_INTERRUPT_CONFIG_REQUEST;
                        }
                        break;

                        /* These states are used if the Arduino is the host */
                case L2CAP_CONTROL_CONNECT_REQUEST:
                        if(l2cap_check_flag(L2CAP_FLAG_CONTROL_CONNECTED)) {

                                ESP_LOGD(LOG_TAG, "Send HID Control Config Request");

                                identifier++;
                                pBtd->l2cap_config_request(hci_handle, identifier, control_scid);
                                l2cap_state = L2CAP_CONTROL_CONFIG_REQUEST;
                        }
                        break;

                case L2CAP_CONTROL_CONFIG_REQUEST:
                        if(l2cap_check_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS)) {
                                setProtocol(); // Set protocol before establishing HID interrupt channel
                                //delay(1); // Short delay between commands - just to be sure

                                ESP_LOGD(LOG_TAG, "Send HID Interrupt Connection Request");

                                identifier++;
                                pBtd->l2cap_connection_request(hci_handle, identifier, interrupt_dcid, HID_INTR_PSM);
                                l2cap_state = L2CAP_INTERRUPT_CONNECT_REQUEST;
                        }
                        break;

                case L2CAP_INTERRUPT_CONNECT_REQUEST:
                        if(l2cap_check_flag(L2CAP_FLAG_INTERRUPT_CONNECTED)) {

                                ESP_LOGD(LOG_TAG, "Send HID Interrupt Config Request");

                                identifier++;
                                pBtd->l2cap_config_request(hci_handle, identifier, interrupt_scid);
                                l2cap_state = L2CAP_INTERRUPT_CONFIG_REQUEST;
                        }
                        break;

                case L2CAP_INTERRUPT_CONFIG_REQUEST:
                        if(l2cap_check_flag(L2CAP_FLAG_CONFIG_INTERRUPT_SUCCESS)) { // Now the HID channels is established

                                ESP_LOGD(LOG_TAG, "HID Channels Established");

                                pBtd->connectToHIDDevice = false;
                                pBtd->pairWithHIDDevice = false;
                                connected = true;
                                onInit();
                                l2cap_state = L2CAP_DONE;
                        }
                        break;

                case L2CAP_DONE:
                        break;

                case L2CAP_INTERRUPT_DISCONNECT:
                        if(l2cap_check_flag(L2CAP_FLAG_DISCONNECT_INTERRUPT_RESPONSE)) {

                                ESP_LOGD(LOG_TAG, "Disconnected Interrupt Channel");

                                identifier++;
                                pBtd->l2cap_disconnection_request(hci_handle, identifier, control_scid, control_dcid);
                                l2cap_state = L2CAP_CONTROL_DISCONNECT;
                        }
                        break;

                case L2CAP_CONTROL_DISCONNECT:
                        if(l2cap_check_flag(L2CAP_FLAG_DISCONNECT_CONTROL_RESPONSE)) {

                                ESP_LOGD(LOG_TAG, "Disconnected Control Channel");

                                pBtd->hci_disconnect(hci_handle);
                                hci_handle = -1; // Reset handle
                                l2cap_event_flag = 0; // Reset flags
                                l2cap_state = L2CAP_WAIT;
                        }
                        break;
        }
}

void BTHID::Run() {
        switch(l2cap_state) {
                case L2CAP_WAIT:
                        if(pBtd->connectToHIDDevice && !pBtd->l2capConnectionClaimed && !connected && !activeConnection) {
                                pBtd->l2capConnectionClaimed = true;
                                activeConnection = true;

                                ESP_LOGD(LOG_TAG, "Send HID Control Connection Request");

                                hci_handle = pBtd->hci_handle; // Store the HCI Handle for the connection
                                l2cap_event_flag = 0; // Reset flags
                                identifier = 0;
                                pBtd->l2cap_connection_request(hci_handle, identifier, control_dcid, HID_CTRL_PSM);
                                l2cap_state = L2CAP_CONTROL_CONNECT_REQUEST;
                        } else if(l2cap_check_flag(L2CAP_FLAG_CONNECTION_CONTROL_REQUEST)) {

                                ESP_LOGD(LOG_TAG, "HID Control Incoming Connection Request");

                                pBtd->l2cap_connection_response(hci_handle, identifier, control_dcid, control_scid, PENDING);
                                //delay(1);
                                pBtd->l2cap_connection_response(hci_handle, identifier, control_dcid, control_scid, SUCCESSFUL);
                                identifier++;
                                //delay(1);
                                pBtd->l2cap_config_request(hci_handle, identifier, control_scid);
                                l2cap_state = L2CAP_CONTROL_SUCCESS;
                        }
                        break;
        }
}

void BTHID::SDP_Command(uint8_t* data, uint8_t nbytes) { // See page 223 in the Bluetooth specs
        pBtd->L2CAP_Command(hci_handle, data, nbytes, sdp_scid[0], sdp_scid[1]);
}

void BTHID::serviceNotSupported(uint8_t transactionIDHigh, uint8_t transactionIDLow) { // See page 235 in the Bluetooth specs
        l2capoutbuf[0] = SDP_SERVICE_SEARCH_ATTRIBUTE_RESPONSE;
        l2capoutbuf[1] = transactionIDHigh;
        l2capoutbuf[2] = transactionIDLow;
        l2capoutbuf[3] = 0x00; // MSB Parameter Length
        l2capoutbuf[4] = 0x05; // LSB Parameter Length = 5
        l2capoutbuf[5] = 0x00; // MSB AttributeListsByteCount
        l2capoutbuf[6] = 0x02; // LSB AttributeListsByteCount = 2

        /* Attribute ID/Value Sequence: */
        l2capoutbuf[7] = 0x35; // Data element sequence - length in next byte
        l2capoutbuf[8] = 0x00; // Length = 0
        l2capoutbuf[9] = 0x00; // No continuation state

        SDP_Command(l2capoutbuf, 10);
}

/************************************************************/
/*                    HID Commands                          */

/************************************************************/
void BTHID::setProtocol() {

        ESP_LOGD(LOG_TAG, "Set protocol mode: 0x%02X", protocolMode);

        if (protocolMode != USB_HID_BOOT_PROTOCOL && protocolMode != HID_RPT_PROTOCOL) {

                ESP_LOGD(LOG_TAG, "Not a valid protocol mode. Using Boot protocol instead.");

                protocolMode = USB_HID_BOOT_PROTOCOL; // Use Boot Protocol by default
        }
        uint8_t command = 0x70 | protocolMode; // Set Protocol, see Bluetooth HID specs page 33
        pBtd->L2CAP_Command(hci_handle, &command, 1, control_scid[0], control_scid[1]);
}

void BTHID::setLeds(uint8_t data) {
        uint8_t buf[3];
        buf[0] = 0xA2; // HID BT DATA_request (0xA0) | Report Type (Output 0x02)
        buf[1] = 0x01; // Report ID
        buf[2] = data;
        pBtd->L2CAP_Command(hci_handle, buf, 3, interrupt_scid[0], interrupt_scid[1]);
}
