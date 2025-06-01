/*
 * hid_keyboard_custom.c
 *
 * Versão adaptada para Raspberry Pi Pico W:
 * - Botões conectados em GPIO 16 (W), 15 (A), 12 (S), 18 (D)
 * - LEDs: 
 *     * Azul (GPIO 14) pisca enquanto não houver conexão ativa.
 *     * Verde (GPIO 13) acende quando houver conexão HID estabelecida.
 *     * Vermelho (GPIO 17) acende por 2 segundos caso a conexão falhe ou seja encerrada.
 *
 * Este arquivo baseia-se no exemplo hid_keyboard_demo do BTstack, porém
 * reorganiza a lógica para atender aos requisitos de três LEDs e quatro botões.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "btstack.h"

#define PIN_BTN_W      16
#define PIN_BTN_A      15
#define PIN_BTN_S      12
#define PIN_BTN_D      18

#define PIN_LED_BLUE   14
#define PIN_LED_GREEN  13
#define PIN_LED_RED    17

#define BLINK_INTERVAL_MS      500
#define RED_LED_ON_MS         2000
#define KEY_DOWN_MS             20
#define KEY_DELAY_MS            20
#define DEBOUNCE_MS            150

#define REPORT_ID 0x01

static const uint8_t hid_descriptor_keyboard[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, REPORT_ID,
    0x75, 0x01, 0x95, 0x08, 0x05, 0x07, 0x19, 0xE0,
    0x29, 0xE7, 0x15, 0x00, 0x25, 0x01, 0x81, 0x02,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x03, 0x95, 0x05,
    0x75, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05,
    0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x03,
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0xFF,
    0x05, 0x07, 0x19, 0x00, 0x29, 0xFF, 0x81, 0x00,
    0xC0
};

// [ESTADO DO HID]
static uint8_t hid_service[300], device_id_record[100];
static const char hid_name[] = "PicoW HID KB";
static uint16_t hid_cid;
static btstack_ring_buffer_t ring_buffer;
static uint8_t ring_storage[16];
static btstack_timer_source_t debounce_timer, send_timer, blink_timer, red_timer;
static uint8_t pending_keycode, pending_modifier;
static bool sending = false;
static int debounce_gpio = -1;

enum { BOOT, IDLE, CONN, CONN_OK } state = BOOT;

// [LEDs]
static bool blue = false;
static void led_blink(btstack_timer_source_t *ts) {
    blue = !blue;
    gpio_put(PIN_LED_BLUE, blue);
    btstack_run_loop_set_timer(ts, BLINK_INTERVAL_MS);
    btstack_run_loop_add_timer(ts);
}

static void led_red_off(btstack_timer_source_t *ts) {
    (void) ts;
    gpio_put(PIN_LED_RED, 0);
    gpio_put(PIN_LED_BLUE, 0);
    btstack_run_loop_set_timer_handler(&blink_timer, led_blink);
    btstack_run_loop_set_timer(&blink_timer, BLINK_INTERVAL_MS);
    btstack_run_loop_add_timer(&blink_timer);
}

// [ENVIO HID]
static void send_report(int mod, int code) {
    uint8_t report[] = { 0xA1, REPORT_ID, mod, 0, code, 0, 0, 0, 0, 0 };
    hid_device_send_interrupt_message(hid_cid, report, sizeof(report));
}

static void send_next(btstack_timer_source_t *ts);
static void key_up(btstack_timer_source_t *ts) {
    (void) ts;
    send_report(0, 0);
    btstack_run_loop_set_timer_handler(&send_timer, send_next);
    btstack_run_loop_set_timer(&send_timer, KEY_DELAY_MS);
    btstack_run_loop_add_timer(&send_timer);
}

static void send_next(btstack_timer_source_t *ts) {
    (void) ts;
    uint8_t ch, kc = 0, mod = 0;
    uint32_t read = 0;
    btstack_ring_buffer_read(&ring_buffer, &ch, 1, &read);
    if (!read) return sending = false, (void)0;
    sending = true;
    if (ch == 'w') { kc = 26; mod = 0; }
    else if (ch == 'a') { kc = 4; mod = 0; }
    else if (ch == 's') { kc = 22; mod = 0; }
    else if (ch == 'd') { kc = 7; mod = 0; }
    else return;
    pending_keycode = kc;
    pending_modifier = mod;
    hid_device_request_can_send_now_event(hid_cid);
}

static void queue_char(char c) {
    btstack_ring_buffer_write(&ring_buffer, (uint8_t*)&c, 1);
    if (!sending) send_next(NULL);
}

// [GPIO E DEBOUNCE]
static void debounce_handler(btstack_timer_source_t *ts) {
    (void) ts;
    char key = 0;
    switch (debounce_gpio) {
        case PIN_BTN_W: key = 'w'; break;
        case PIN_BTN_A: key = 'a'; break;
        case PIN_BTN_S: key = 's'; break;
        case PIN_BTN_D: key = 'd'; break;
        default: return;
    }
    queue_char(key);
    debounce_gpio = -1;
}

static void gpio_handler(uint gpio, uint32_t events) {
    if (!(events & GPIO_IRQ_EDGE_FALL)) return;
    if (debounce_gpio != -1) return; // ignora se debounce em andamento
    debounce_gpio = gpio;
    btstack_run_loop_set_timer_handler(&debounce_timer, debounce_handler);
    btstack_run_loop_set_timer(&debounce_timer, DEBOUNCE_MS);
    btstack_run_loop_add_timer(&debounce_timer);
}

static void init_gpio(void) {
    const int btns[] = { PIN_BTN_W, PIN_BTN_A, PIN_BTN_S, PIN_BTN_D };
    for (int i = 0; i < 4; i++) {
        gpio_init(btns[i]);
        gpio_set_dir(btns[i], GPIO_IN);
        gpio_pull_up(btns[i]);
        gpio_set_irq_enabled_with_callback(btns[i], GPIO_IRQ_EDGE_FALL, true, gpio_handler);
    }
    gpio_init(PIN_LED_BLUE); gpio_set_dir(PIN_LED_BLUE, GPIO_OUT);
    gpio_init(PIN_LED_GREEN); gpio_set_dir(PIN_LED_GREEN, GPIO_OUT);
    gpio_init(PIN_LED_RED); gpio_set_dir(PIN_LED_RED, GPIO_OUT);
}

// [PACOTES HCI]
static void handler(uint8_t type, uint16_t chan, uint8_t *packet, uint16_t size) {
    (void)chan; (void)size;
    if (type != HCI_EVENT_PACKET) return;
    uint8_t evt = hci_event_packet_get_type(packet);
    switch (evt) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                state = IDLE;
                btstack_run_loop_set_timer_handler(&blink_timer, led_blink);
                btstack_run_loop_set_timer(&blink_timer, BLINK_INTERVAL_MS);
                btstack_run_loop_add_timer(&blink_timer);
            }
            break;
        case HCI_EVENT_HID_META: {
            uint8_t sub = hci_event_hid_meta_get_subevent_code(packet);
            switch (sub) {
                case HID_SUBEVENT_CONNECTION_OPENED: {
                    if (hid_subevent_connection_opened_get_status(packet) != ERROR_CODE_SUCCESS) {
                        state = IDLE; hid_cid = 0;
                        btstack_run_loop_remove_timer(&blink_timer);
                        gpio_put(PIN_LED_BLUE, 0); gpio_put(PIN_LED_RED, 1);
                        btstack_run_loop_set_timer_handler(&red_timer, led_red_off);
                        btstack_run_loop_set_timer(&red_timer, RED_LED_ON_MS);
                        btstack_run_loop_add_timer(&red_timer);
                        return;
                    }
                    state = CONN_OK;
                    hid_cid = hid_subevent_connection_opened_get_hid_cid(packet);
                    btstack_run_loop_remove_timer(&blink_timer);
                    gpio_put(PIN_LED_BLUE, 0); gpio_put(PIN_LED_GREEN, 1);
                    break;
                }
                case HID_SUBEVENT_CONNECTION_CLOSED:
                    state = IDLE; hid_cid = 0;
                    btstack_run_loop_remove_timer(&send_timer);
                    gpio_put(PIN_LED_GREEN, 0); gpio_put(PIN_LED_RED, 1);
                    btstack_run_loop_set_timer_handler(&red_timer, led_red_off);
                    btstack_run_loop_set_timer(&red_timer, RED_LED_ON_MS);
                    btstack_run_loop_add_timer(&red_timer);
                    break;
                case HID_SUBEVENT_CAN_SEND_NOW:
                    if (pending_keycode) {
                        send_report(pending_modifier, pending_keycode);
                        pending_keycode = 0;
                        btstack_run_loop_set_timer_handler(&send_timer, key_up);
                        btstack_run_loop_set_timer(&send_timer, KEY_DOWN_MS);
                        btstack_run_loop_add_timer(&send_timer);
                    } else {
                        send_report(0, 0);
                        btstack_run_loop_set_timer_handler(&send_timer, send_next);
                        btstack_run_loop_set_timer(&send_timer, KEY_DELAY_MS);
                        btstack_run_loop_add_timer(&send_timer);
                    }
                    break;
            }
        }
    }
}

// [MAIN]
int btstack_main(int argc, const char *argv[]) {
    (void)argc; (void)argv;
    stdio_init_all();
    init_gpio();
    gap_discoverable_control(1);
    gap_set_class_of_device(0x2540);
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE);
    gap_set_allow_role_switch(true);
    l2cap_init();
    sdp_init();

    hid_sdp_record_t params = {
        0x2540, 33, 0, 1, 1, 1, 0, 1600, 3200, 3200,
        hid_descriptor_keyboard, sizeof(hid_descriptor_keyboard), hid_name
    };
    hid_create_sdp_record(hid_service, sdp_create_service_record_handle(), &params);
    sdp_register_service(hid_service);
    device_id_create_sdp_record(device_id_record, sdp_create_service_record_handle(),
        DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH, BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
    sdp_register_service(device_id_record);
    hid_device_init(0, sizeof(hid_descriptor_keyboard), hid_descriptor_keyboard);

    static btstack_packet_callback_registration_t cb;
    cb.callback = handler;
    hci_add_event_handler(&cb);
    hid_device_register_packet_handler(handler);
    btstack_ring_buffer_init(&ring_buffer, ring_storage, sizeof(ring_storage));
    hci_power_control(HCI_POWER_ON);
    return 0;
}
