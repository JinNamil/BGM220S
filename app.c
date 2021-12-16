/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "app_log.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "buzz2.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_led_instances.h"
typedef struct {
    uint8_t flags_len;     // Length of the Flags field.
    uint8_t flags_type;    // Type of the Flags field.
    uint8_t flags;         // Flags field.
    uint8_t mandata_ten;   // Length of the Manufacturer Data field.
    uint8_t mandata_type;  // Type of the Manufacturer Data field.
    uint8_t comp_id[2];    // Company ID field.
    uint8_t beac_type[2];  // Beacon Type field.
    uint8_t uuid[16];      // 128-bit Universally Unique Identifier (UUID). The UUID is an identifier for the company using the beacon.
    uint8_t maj_num[2];    // Beacon major number. Used to group related beacons.
    uint8_t min_num[2];    // Beacon minor number. Used to specify individual beacons within a group.
    uint8_t tx_power;      // The Beacon's measured RSSI at 1 meter distance in dBm. See the iBeacon specification for measurement guidelines.
  }bcn_beacon_adv_data;
// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
static buzz2_t buzz2;
extern sl_pwm_instance_t sl_pwm_buzzer;
static bool report_button_flag = false;

// Updates the Report Button characteristic.
static sl_status_t update_report_button_characteristic(void);
// Sends notification of the Report Button characteristic.
static sl_status_t send_report_button_notification(void);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  // Make sure there will be no button events before the boot event.
  sl_button_disable(SL_SIMPLE_BUTTON_INSTANCE(0));
  buzz2.pwm = sl_pwm_buzzer;
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  // Check if there was a report button interaction.
  if (report_button_flag) {
    sl_status_t sc;

    report_button_flag = false; // Reset flag

    buzz2_play_sound(&buzz2, 2700, 50, 5);
//    sc = update_report_button_characteristic();
//    app_log_status_error(sc);
//
//    if (sc == SL_STATUS_OK) {
//      sc = send_report_button_notification();
//      app_log_status_error(sc);
//    }

  }

  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

static uint8_t find_service_in_advertisement(uint8_t *data, uint8_t len)
{
  uint8_t ad_field_length;
  uint8_t ad_field_type;
  uint8_t i = 0;
  // Parse advertisement packet
  while (i < len) {
    ad_field_length = data[i];
    ad_field_type = data[i + 1];
    // Partial ($02) or complete ($03) list of 16-bit UUIDs
    if (ad_field_type == 0x02 || ad_field_type == 0x03) {
      // compare UUID to Health Thermometer service UUID
      // if (memcmp(&data[i + 2], thermo_service, 2) == 0) {
        return 1;
      // }
    }
    // advance to the next AD struct
    i = i + ad_field_length + 1;
  }
  return 0;
}
#define SCAN_PASSIVE                  0
/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];
  uint8_t advData[255] = {0,};
  uint8_t deviceName[16] = {0,};
  bd_addr searchAddr = {0,};
  uint16_t convertData = 0;
//  uint8_t connectAddr[6] = {0x36, 0x0B, 0xD9, 0x5E, 0xCF, 0xD0};
  uint8_t connectAddr[6] = {0x70, 0x2B, 0x2C, 0xB7, 0xB2, 0x0C};
  bcn_beacon_adv_data ibeaconData = {0,};
  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_log_status_error(sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_log_status_error(sc);
      app_log_info("firmware version v0.2\n");
      app_log_info("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   address_type ? "static random" : "public device",
                   address.addr[5],
                   address.addr[4],
                   address.addr[3],
                   address.addr[2],
                   address.addr[1],
                   address.addr[0]);
      #if CENTRAL_MODE
      ///////////////////////////////////////////////////////
      ///
      // Set passive scanning on 1Mb PHY
      sc = sl_bt_scanner_set_mode(gap_1m_phy, 0);
      app_assert_status(sc);
      // Set scan interval and scan window:  50%duty cycle
      sc = sl_bt_scanner_set_timing(gap_1m_phy, 20, 10);
      app_assert_status(sc);

      // Start scanning
      sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
      app_assert_status_f(sc, "Failed to start discovery #1\n");
      ///////////////////////////////////////////////////////
      // Create an advertising set.
      #endif

      #if PERIPHERAL_MODE
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_log_status_error(sc);
      uint8_t adv_data[3] = {0xAA, 0xBB, 0xCC};
      // Set custom advertising data.
      sc = sl_bt_advertiser_set_data(advertising_set_handle,
                                    0,
                                    sizeof(adv_data),
                                    adv_data);
      app_assert_status(sc);
      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        1500,   // adv. duration
        0);  // max. num. adv. events
      app_log_status_error(sc);
      // Start general advertising and enable connections.
      // sc = sl_bt_advertiser_start(
      //   advertising_set_handle,
      //   sl_bt_advertiser_user_data,
      //   sl_bt_advertiser_non_connectable);
      // app_log_status_error(sc);
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      #endif
      // Button events can be received from now on.
      sl_button_enable(SL_SIMPLE_BUTTON_INSTANCE(0));

      // Check the report button state, then update the characteristic and
      // send notification.
      sc = update_report_button_characteristic();
      app_log_status_error(sc);

      if (sc == SL_STATUS_OK) {
        sc = send_report_button_notification();
        app_log_status_error(sc);
      }
      break;

    case sl_bt_evt_scanner_scan_report_id:
      app_log_info("scan address: ");
      for(int i = 0; i < 6; i++)
      {
          searchAddr.addr[i] = evt->data.evt_scanner_scan_report.address.addr[i];
          printf("%02X ", evt->data.evt_scanner_scan_report.address.addr[i]);
      }
      printf("\n");
      app_log_info("advertising data: ");
      for(int i = 0; i < evt->data.evt_scanner_scan_report.data.len; i++)
      {
          advData[i] = evt->data.evt_scanner_scan_report.data.data[i];
          printf("%02X ", evt->data.evt_scanner_scan_report.data.data[i]);  //[header len(n)][...][n][name len m][...][m]
      }
      printf("\n");

      if(memcmp(searchAddr.addr, connectAddr, 6) == 0)
      {
          app_log_info("search my peripheral!\nscan stop\n");
          sc = sl_bt_scanner_stop();
          app_log_status_error(sc);
          memcpy(&ibeaconData, evt->data.evt_scanner_scan_report.data.data, sizeof(ibeaconData));
          app_log_info("[ibeacon data]\n");
          printf("--------------------------------------------\n");
          printf("flag length: %d\n", ibeaconData.flags_len);
          printf("flag type: 0x%02X\n", ibeaconData.flags_type);
          printf("flag: 0x%02X\n", ibeaconData.flags);
          printf("manufacturer length: %d\n", ibeaconData.mandata_ten);
          printf("manufacturer type: 0x%02X\n", ibeaconData.mandata_type);
          printf("company id: 0x%02X%02X\n", ibeaconData.comp_id[1], ibeaconData.comp_id[0]);
          printf("beacon type: 0x%02X%02X\n", ibeaconData.beac_type[1], ibeaconData.beac_type[0]);

          printf("uuid: ");
          for(int i = 0; i < 16; i++)
          {
            printf("%02X ", ibeaconData.uuid[i]);
          }
          printf("\n");

          convertData = (uint16_t)((ibeaconData.maj_num[1]) | (ibeaconData.maj_num[0]<<8));
          printf("beacon major number: %d\n", convertData);
          convertData = (uint16_t)((ibeaconData.min_num[1]) | (ibeaconData.min_num[0]<<8));
          printf("beacon minor number: %d\n", convertData);
          printf("tx power: %02X\n", ibeaconData.tx_power);
          printf("--------------------------------------------\n");

          sc = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
                                    evt->data.evt_scanner_scan_report.address_type,
                                    sl_bt_gap_1m_phy,
                                    &evt->data.evt_connection_opened.connection);
          app_log_status_error(sc);
      }
      break;
    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      app_log_info("Connection opened.\n");
//      if(evt->data.evt_connection_opened.master == 0)
//      {
//        sc = sl_bt_sm_increase_security(evt->data.evt_connection_opened.connection);
//        app_log_status_error(sc);
//      }
      app_log_info("central address: ");
      for(int i = 0; i < 6; i++)
      {
          app_log_info("%02X ", evt->data.evt_connection_opened.address.addr[i]);
      }
      app_log_info("Device information: %d, %d\r\n", evt->data.evt_connection_opened.master, evt->data.evt_connection_opened.address_type);

      break;

    case sl_bt_evt_sm_bonded_id:
      app_log("Successful bonding\r\n");
      break;

    case  sl_bt_evt_sm_bonding_failed_id:
      app_log("Bonding failed, reason: 0x%2X\r\n", evt->data.evt_sm_bonding_failed.reason);
      /* Previous bond is broken, delete it and close connection, host must retry at least once */
      sl_bt_sm_delete_bondings();
      break;
    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      app_log_info("Connection closed.\n");

      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    case sl_bt_evt_sync_opened_id:
        printf("sl_bt_evt_sync_opened_id\r\n");
        break;
    case sl_bt_evt_sync_closed_id:
        printf("sl_bt_evt_sync_closed_id\r\n");
        break;
    case sl_bt_evt_sync_data_id:
        printf("sl_bt_evt_sync_data_id\r\n");
        break;
    case sl_bt_evt_sm_passkey_display_id:
        printf("sl_bt_evt_sm_passkey_display_id\r\n");
        break;
    case sl_bt_evt_sm_passkey_request_id:
        printf("sl_bt_evt_sm_passkey_request_id\r\n");
        break;
    case sl_bt_evt_sm_confirm_passkey_id:
        printf("sl_bt_evt_sm_confirm_passkey_id\r\n");
        break;
    case sl_bt_evt_sm_list_bonding_entry_id:
        printf("sl_bt_evt_sm_list_bonding_entry_id\r\n");
        break;
    case sl_bt_evt_sm_list_all_bondings_complete_id:
        printf("sl_bt_evt_sm_list_all_bondings_complete_id\r\n");
        printf("    Event Parameters:\r\n");
        printf("        bonding:      0x%02x\r\n", evt->data.evt_sm_list_bonding_entry.bonding);
        printf  ("        address:      %02x", evt->data.evt_sm_list_bonding_entry.address.addr[0]);
        printf  (":%02x", evt->data.evt_sm_list_bonding_entry.address.addr[1]);
        printf  (":%02x", evt->data.evt_sm_list_bonding_entry.address.addr[2]);
        printf  (":%02x", evt->data.evt_sm_list_bonding_entry.address.addr[3]);
        printf  (":%02x", evt->data.evt_sm_list_bonding_entry.address.addr[4]);
        printf(":%02x\r\n", evt->data.evt_sm_list_bonding_entry.address.addr[5]);
        printf("        address type: 0x%02x\r\n", evt->data.evt_sm_list_bonding_entry.address_type);
        break;
    case sl_bt_evt_sm_confirm_bonding_id:
        printf("sl_bt_evt_sm_confirm_bonding_id\r\n");
        printf("    Event Parameters:\r\n");
        printf("        connection:     0x%02x\r\n", evt->data.evt_sm_confirm_bonding.connection);
        printf("        bonding_handle: %02d",   evt->data.evt_sm_confirm_bonding.bonding_handle);
        break;
    // -------------------------------
    // This event indicates that the value of an attribute in the local GATT
    // database was changed by a remote GATT client.
    case sl_bt_evt_gatt_server_attribute_value_id:
      // The value of the gattdb_led_control characteristic was changed.
      if (gattdb_led_control == evt->data.evt_gatt_server_characteristic_status.characteristic) {
        uint8_t data_recv;
        size_t data_recv_len;

        // Read characteristic value.
        sc = sl_bt_gatt_server_read_attribute_value(gattdb_led_control,
                                                    0,
                                                    sizeof(data_recv),
                                                    &data_recv_len,
                                                    &data_recv);
        (void)data_recv_len;
        app_log_status_error(sc);

        if (sc != SL_STATUS_OK) {
          break;
        }

        // Toggle LED.
        if (data_recv == 0x00) {
          sl_led_turn_off(SL_SIMPLE_LED_INSTANCE(0));
          app_log_info("LED off.\n");
        } else if (data_recv == 0x01) {
          sl_led_turn_on(SL_SIMPLE_LED_INSTANCE(0));
          app_log_info("LED on.\n");
        } else {
          app_log_error("Invalid attribute value: 0x%02x\n", (int)data_recv);
        }
      }
      break;

    // -------------------------------
    // This event occurs when the remote device enabled or disabled the
    // notification.
    case sl_bt_evt_gatt_server_characteristic_status_id:
      if (gattdb_report_button == evt->data.evt_gatt_server_characteristic_status.characteristic) {
        // A local Client Characteristic Configuration descriptor was changed in
        // the gattdb_report_button characteristic.
        if (evt->data.evt_gatt_server_characteristic_status.client_config_flags
            & sl_bt_gatt_notification) {
          // The client just enabled the notification. Send notification of the
          // current button state stored in the local GATT table.
          app_log_info("Notification enabled.");

          sc = send_report_button_notification();
          app_log_status_error(sc);
        } else {
          app_log_info("Notification disabled.\n");
        }
      }
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/***************************************************************************//**
 * Simple Button
 * Button state changed callback
 * @param[in] handle Button event handle
 ******************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
  if (SL_SIMPLE_BUTTON_INSTANCE(0) == handle) {
    report_button_flag = true;
  }
}

/***************************************************************************//**
 * Updates the Report Button characteristic.
 *
 * Checks the current button state and then writes it into the local GATT table.
 ******************************************************************************/
static sl_status_t update_report_button_characteristic(void)
{
  sl_status_t sc;
  uint8_t data_send;

  switch (sl_button_get_state(SL_SIMPLE_BUTTON_INSTANCE(0))) {
    case SL_SIMPLE_BUTTON_PRESSED:
      data_send = (uint8_t)SL_SIMPLE_BUTTON_PRESSED;
      break;

    case SL_SIMPLE_BUTTON_RELEASED:
      data_send = (uint8_t)SL_SIMPLE_BUTTON_RELEASED;
      break;

    default:
      // Invalid button state
      return SL_STATUS_FAIL; // Invalid button state
  }

  // Write attribute in the local GATT database.
  sc = sl_bt_gatt_server_write_attribute_value(gattdb_report_button,
                                               0,
                                               1,
                                               0xFF);
  if (sc == SL_STATUS_OK) {
    app_log_info("Attribute written: 0x%02x", (int)data_send);
  }

  return sc;
}

/***************************************************************************//**
 * Sends notification of the Report Button characteristic.
 *
 * Reads the current button state from the local GATT database and sends it as a
 * notification.
 ******************************************************************************/
static sl_status_t send_report_button_notification(void)
{
  sl_status_t sc;
  uint8_t data_send[32] = {0xAA,};
  size_t data_len;

  // Read report button characteristic stored in local GATT database.
  sc = sl_bt_gatt_server_read_attribute_value(gattdb_report_button,
                                              0,
                                              sizeof(data_send),
                                              &data_len,
                                              &data_send);
  if (sc != SL_STATUS_OK) {
    return sc;
  }

  // Send characteristic notification.
  sc = sl_bt_gatt_server_notify_all(gattdb_report_button,
                                    sizeof(data_send),
                                    &data_send);
  if (sc == SL_STATUS_OK) {
    app_log_append(" Notification sent: 0x%02x\n", (int)data_send);
  }
  return sc;
}
