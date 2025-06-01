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

// -------------------------------------------------------------
// Definições de pinos para botões e LEDs
// -------------------------------------------------------------
#define PIN_BTN_W      16  // Tecla “W”
#define PIN_BTN_A      15  // Tecla “A”
#define PIN_BTN_S      12  // Tecla “S”
#define PIN_BTN_D      18  // Tecla “D”

#define PIN_LED_BLUE   14  // LED azul (em modo de espera / advertising)
#define PIN_LED_GREEN  13  // LED verde (conexão estabelecida)
#define PIN_LED_RED    17  // LED vermelho (erro ou desconexão)

// -------------------------------------------------------------
// Parâmetros de tempo para LEDs e envio de teclas
// -------------------------------------------------------------
#define BLINK_INTERVAL_MS      500   // Intervalo para piscar LED azul
#define RED_LED_ON_MS         2000   // Tempo que o LED vermelho fica aceso após erro

#define KEY_DOWN_MS             20   // Duração do “pressionar tecla”
#define KEY_DELAY_MS            20   // Intervalo entre envios de teclas

// ----------------------------------------------------------------
// Descriptor HID de teclado (igual ao USB HID Spec 1.1, App B.1)
// ----------------------------------------------------------------
#define REPORT_ID 0x01

static const uint8_t hid_descriptor_keyboard[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, REPORT_ID,   //   Report ID
    // Modifier byte (8 bits)
    0x75, 0x01,        //     Report Size (1)
    0x95, 0x08,        //     Report Count (8)
    0x05, 0x07,        //     Usage Page (Key Codes)
    0x19, 0xE0,        //     Usage Minimum (Keyboard LeftControl)
    0x29, 0xE7,        //     Usage Maximum (Keyboard Right GUI)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    // Reserved byte
    0x75, 0x01,        //     Report Size (1)
    0x95, 0x08,        //     Report Count (8)
    0x81, 0x03,        //     Input (Const,Var,Abs)
    // LEDs (output) + Padding
    0x95, 0x05,        //     Report Count (5)
    0x75, 0x01,        //     Report Size (1)
    0x05, 0x08,        //     Usage Page (LEDs)
    0x19, 0x01,        //     Usage Minimum (Num Lock)
    0x29, 0x05,        //     Usage Maximum (Kana)
    0x91, 0x02,        //     Output (Data,Var,Abs)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x03,        //     Report Size (3)
    0x91, 0x03,        //     Output (Const,Var,Abs)
    // Keys (6 bytes)
    0x95, 0x06,        //     Report Count (6)
    0x75, 0x08,        //     Report Size (8)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0xFF,        //     Logical Maximum (255)
    0x05, 0x07,        //     Usage Page (Key Codes)
    0x19, 0x00,        //     Usage Minimum (0)
    0x29, 0xFF,        //     Usage Maximum (255)
    0x81, 0x00,        //     Input (Data,Array)
    0xC0               // End Collection
};

// -------------------------------------------------------------
// Tabelas de conversão de caracteres para códigos HID (layout US)
// -------------------------------------------------------------
#define CHAR_ILLEGAL    0xFF
#define CHAR_RETURN     '\n'
#define CHAR_ESCAPE     27
#define CHAR_TAB        '\t'
#define CHAR_BACKSPACE  0x7F

static const uint8_t keytable_us_no_shift[] = {
    CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    'a','b','c','d','e','f','g','h','i','j',
    'k','l','m','n','o','p','q','r','s','t',
    'u','v','w','x','y','z',
    '1','2','3','4','5','6','7','8','9','0',
    CHAR_RETURN, CHAR_ESCAPE, CHAR_BACKSPACE, CHAR_TAB, ' ',
    '-','=','[',']','\\', CHAR_ILLEGAL, ';','\'', '`', ',',
    '.', '/', CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    '*','-','+','\n','1','2','3','4','5',
    '6','7','8','9','0','.',0xA7
};

static const uint8_t keytable_us_shift[] = {
    CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    'A','B','C','D','E','F','G','H','I','J',
    'K','L','M','N','O','P','Q','R','S','T',
    'U','V','W','X','Y','Z',
    '!','@','#','$','%','^','&','*','(',')',
    CHAR_RETURN, CHAR_ESCAPE, CHAR_BACKSPACE, CHAR_TAB, ' ',
    '_','+','{','}','|', CHAR_ILLEGAL, ':','"', '~', '<',
    '>','?', CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL, CHAR_ILLEGAL,
    '*','-','+','\n','1','2','3','4','5',
    '6','7','8','9','0','.',0xB1
};

// -------------------------------------------------------------
// Variáveis de estado do HID e buffers de envio
// -------------------------------------------------------------
static uint8_t hid_service_record[300];
static uint8_t device_id_record[100];
static const char hid_name[] = "PicoW HID KB";

static uint16_t hid_cid = 0;
static uint8_t  hid_boot_mode = 0;

// Buffer ring para fila de caracteres a enviar
static btstack_ring_buffer_t    ring_buffer;
static uint8_t                  ring_buffer_storage[16];

// Timers BTstack
static btstack_timer_source_t   timer_send;
static btstack_timer_source_t   timer_blink_blue;
static btstack_timer_source_t   timer_red_off;

// Flag para controle de envio
static uint8_t pending_modifier = 0;
static uint8_t pending_keycode  = 0;
static bool    sending_in_progress = false;

// Estado de aplicativo/bluetooth
static enum {
    ST_BOOTING,
    ST_IDLE,
    ST_CONNECTING,
    ST_CONNECTED
} app_state = ST_BOOTING;

// -----------------------------------------------------------------
// Protótipos de funções
// -----------------------------------------------------------------
static void blink_blue_led(btstack_timer_source_t *ts);
static void turn_off_red_led(btstack_timer_source_t *ts);
static void button_irq_handler(uint gpio, uint32_t events);
static void send_report(int modifier, int keycode);
static void on_key_up(btstack_timer_source_t *ts);
static void send_next_char(btstack_timer_source_t *ts);
static void queue_char(char c);
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

// -------------------------------------------------------------
// Converte caractere ASCII em keycode + modificador (US)
// -------------------------------------------------------------
static bool lookup_in_table(uint8_t ch, const uint8_t *tbl, int len, uint8_t *kc) {
    for (int i = 0; i < len; i++) {
        if (tbl[i] == ch) {
            *kc = (uint8_t)i;
            return true;
        }
    }
    return false;
}

static bool find_keycode_mod(uint8_t ch, uint8_t *kc, uint8_t *mod) {
    if (lookup_in_table(ch, keytable_us_no_shift, sizeof(keytable_us_no_shift), kc)) {
        *mod = 0;
        return true;
    }
    if (lookup_in_table(ch, keytable_us_shift, sizeof(keytable_us_shift), kc)) {
        *mod = 0x02;  // SHIFT
        return true;
    }
    return false;
}

// -------------------------------------------------------------
// Inicializa GPIOs dos botões e LEDs
// -------------------------------------------------------------
static void init_gpio_pins(void) {
    // LEDs como saída, inicializa todos apagados
    gpio_init(PIN_LED_BLUE);
    gpio_set_dir(PIN_LED_BLUE, GPIO_OUT);
    gpio_put(PIN_LED_BLUE, 0);

    gpio_init(PIN_LED_GREEN);
    gpio_set_dir(PIN_LED_GREEN, GPIO_OUT);
    gpio_put(PIN_LED_GREEN, 0);

    gpio_init(PIN_LED_RED);
    gpio_set_dir(PIN_LED_RED, GPIO_OUT);
    gpio_put(PIN_LED_RED, 0);

    // Botões como entrada com pull-up, IRQ na borda de descida
    const uint pins_btn[] = { PIN_BTN_W, PIN_BTN_A, PIN_BTN_S, PIN_BTN_D };
    for (int i = 0; i < 4; i++) {
        gpio_init(pins_btn[i]);
        gpio_set_dir(pins_btn[i], GPIO_IN);
        gpio_pull_up(pins_btn[i]);
        gpio_set_irq_enabled_with_callback(pins_btn[i], GPIO_IRQ_EDGE_FALL, true, &button_irq_handler);
    }
}

// -------------------------------------------------------------
// Handler de IRQ de botões: quando um botão é pressionado,
// coloca-se o caractere correspondente na fila de envio.
// -------------------------------------------------------------
static void button_irq_handler(uint gpio, uint32_t events) {
    if (!(events & GPIO_IRQ_EDGE_FALL)) return;
    char to_queue = 0;
    switch (gpio) {
        case PIN_BTN_W: to_queue = 'w'; break;
        case PIN_BTN_A: to_queue = 'a'; break;
        case PIN_BTN_S: to_queue = 's'; break;
        case PIN_BTN_D: to_queue = 'd'; break;
        default: return;
    }
    queue_char(to_queue);
}

// -------------------------------------------------------------
// Funções para controle de LEDs via timers BTstack
// -------------------------------------------------------------
static bool blue_state = false;
static void blink_blue_led(btstack_timer_source_t *ts) {
    (void)ts;
    blue_state = !blue_state;
    gpio_put(PIN_LED_BLUE, blue_state);
    btstack_run_loop_set_timer(&timer_blink_blue, BLINK_INTERVAL_MS);
    btstack_run_loop_add_timer(&timer_blink_blue);
}

static void turn_off_red_led(btstack_timer_source_t *ts) {
    (void)ts;
    gpio_put(PIN_LED_RED, 0);
    // Reiniciar piscada azul
    blue_state = false;
    gpio_put(PIN_LED_BLUE, 0);
    btstack_run_loop_set_timer(&timer_blink_blue, BLINK_INTERVAL_MS);
    btstack_run_loop_add_timer(&timer_blink_blue);
}

// -------------------------------------------------------------
// Funções de envio de relatórios HID
// -------------------------------------------------------------
static void send_report(int modifier, int keycode) {
    // Relatório padrão: [0xA1, REPORT_ID, modifier, 0, keycode, 0,0,0,0,0]
    uint8_t report[10] = { 0xA1, REPORT_ID, (uint8_t)modifier, 0, (uint8_t)keycode, 0, 0, 0, 0, 0 };
    hid_device_send_interrupt_message(hid_cid, report, sizeof(report));
}

// Após “key down” (report com tecla), agendar key up
static void on_key_up(btstack_timer_source_t *ts) {
    (void)ts;
    send_report(0, 0);  // envia relatório “todos os botões soltos”
    // Agendar próximo caractere
    btstack_run_loop_set_timer(&timer_send, KEY_DELAY_MS);
    btstack_run_loop_set_timer_handler(&timer_send, send_next_char);
    btstack_run_loop_add_timer(&timer_send);
}

// Pega próximo caractere da fila e solicita “can send now”
static void send_next_char(btstack_timer_source_t *ts) {
    (void)ts;
    uint8_t ch;
    uint32_t read_bytes = 0;
    btstack_ring_buffer_read(&ring_buffer, &ch, 1, &read_bytes);
    if (read_bytes == 0) {
        sending_in_progress = false;
        return;
    }
    sending_in_progress = true;
    uint8_t kc = 0, mod = 0;
    if (find_keycode_mod(ch, &kc, &mod)) {
        pending_keycode  = kc;
        pending_modifier = mod;
        hid_device_request_can_send_now_event(hid_cid);
    } else {
        // caractere inválido; tenta próximo após delay
        btstack_run_loop_set_timer(&timer_send, KEY_DELAY_MS);
        btstack_run_loop_set_timer_handler(&timer_send, send_next_char);
        btstack_run_loop_add_timer(&timer_send);
    }
}

// Coloca caractere ASCII na fila
static void queue_char(char c) {
    btstack_ring_buffer_write(&ring_buffer, (uint8_t *)&c, 1);
    if (!sending_in_progress) {
        send_next_char(NULL);
    }
}

// -------------------------------------------------------------
// Callback do BTstack (recebe eventos HCI/HID)
// -------------------------------------------------------------
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t evt = hci_event_packet_get_type(packet);
    switch (evt) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            // Pronto para ser descoberto/emparelhado
            app_state = ST_IDLE;
            // Liga piscada do LED azul
            btstack_run_loop_set_timer_handler(&timer_blink_blue, blink_blue_led);
            btstack_run_loop_set_timer(&timer_blink_blue, BLINK_INTERVAL_MS);
            btstack_run_loop_add_timer(&timer_blink_blue);
            break;

        case HCI_EVENT_USER_CONFIRMATION_REQUEST:
            // Auto-aceita SSP (só loga no console)
            log_info("SSP Confirmation: %06" PRIu32 "\n",
                     hci_event_user_confirmation_request_get_numeric_value(packet));
            break;

        case HCI_EVENT_HID_META: {
            uint8_t sub = hci_event_hid_meta_get_subevent_code(packet);
            switch (sub) {
                case HID_SUBEVENT_CONNECTION_OPENED: {
                    uint8_t status = hid_subevent_connection_opened_get_status(packet);
                    if (status != ERROR_CODE_SUCCESS) {
                        // Falha ao conectar
                        app_state = ST_IDLE;
                        hid_cid = 0;
                        // Desliga LED azul, mostra erro em vermelho
                        btstack_run_loop_remove_timer(&timer_blink_blue);
                        gpio_put(PIN_LED_BLUE, 0);
                        gpio_put(PIN_LED_RED, 1);
                        // Agenda apagar vermelho e voltar a piscar azul
                        btstack_run_loop_set_timer_handler(&timer_red_off, turn_off_red_led);
                        btstack_run_loop_set_timer(&timer_red_off, RED_LED_ON_MS);
                        btstack_run_loop_add_timer(&timer_red_off);
                        return;
                    }
                    // Conexão estabelecida com sucesso
                    hid_cid = hid_subevent_connection_opened_get_hid_cid(packet);
                    app_state = ST_CONNECTED;
                    // Desliga azul, acende verde
                    btstack_run_loop_remove_timer(&timer_blink_blue);
                    gpio_put(PIN_LED_BLUE, 0);
                    gpio_put(PIN_LED_GREEN, 1);
#ifdef HAVE_BTSTACK_STDIN
                    printf("Conectado! Digite algo no terminal...\n");
#else
                    printf("Conectado! Enviando texto de demonstração...\n");
                    // Inicia envio de texto de demonstração (exemplo fixo)
                    // Podemos colocar aqui um texto fixo, mas mantemos fila vazia,
                    // aguardando botões neste exemplo. Se quiser texto demo, descomente:
                    // queue_char('H'); queue_char('e'); queue_char('l'); queue_char('l'); queue_char('o');
                    // send_next_char(NULL);
#endif
                    break;
                }
                case HID_SUBEVENT_CONNECTION_CLOSED: {
                    // Host desconectou
                    app_state = ST_IDLE;
                    hid_cid = 0;
                    btstack_run_loop_remove_timer(&timer_send);
                    // Apaga verde, mostra vermelho
                    gpio_put(PIN_LED_GREEN, 0);
                    gpio_put(PIN_LED_RED, 1);
                    // Apaga vermelho após alguns segundos e retoma azul piscando
                    btstack_run_loop_set_timer_handler(&timer_red_off, turn_off_red_led);
                    btstack_run_loop_set_timer(&timer_red_off, RED_LED_ON_MS);
                    btstack_run_loop_add_timer(&timer_red_off);
                    break;
                }
                case HID_SUBEVENT_CAN_SEND_NOW: {
                    // Hora de enviar key down ou key up
                    if (pending_keycode) {
                        send_report(pending_modifier, pending_keycode);
                        // Agendar key up
                        pending_keycode  = 0;
                        pending_modifier = 0;
                        btstack_run_loop_set_timer_handler(&timer_send, on_key_up);
                        btstack_run_loop_set_timer(&timer_send, KEY_DOWN_MS);
                        btstack_run_loop_add_timer(&timer_send);
                    } else {
                        send_report(0, 0);
                        // Próximo caractere
                        btstack_run_loop_set_timer_handler(&timer_send, send_next_char);
                        btstack_run_loop_set_timer(&timer_send, KEY_DELAY_MS);
                        btstack_run_loop_add_timer(&timer_send);
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        }
        default:
            break;
    }
}

// -------------------------------------------------------------
// Função principal do BTstack, chamada após hci_power_control
// -------------------------------------------------------------
int btstack_main(int argc, const char *argv[]) {
    (void)argc;
    (void)argv;

    // Inicialização geral do stdio e GPIO
    stdio_init_all();
    init_gpio_pins();

    // Configurações do GAP (Bluetooth clássico)
    gap_discoverable_control(1);
    gap_set_class_of_device(0x2540); // CoD: Keyboard
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE);
    gap_set_allow_role_switch(true);

    // Inicializa L2CAP
    l2cap_init();

#ifdef ENABLE_BLE
    sm_init();  // Só se BLE habilitado
#endif

    // Inicializa SDP e registra serviço HID
    sdp_init();
    memset(hid_service_record, 0, sizeof(hid_service_record));
    hid_sdp_record_t params = {
        0x2540, 33,
        0, 1, 1, 1,
        hid_boot_mode,
        1600, 3200,
        3200,
        hid_descriptor_keyboard,
        sizeof(hid_descriptor_keyboard),
        hid_name
    };
    hid_create_sdp_record(hid_service_record, sdp_create_service_record_handle(), &params);
    btstack_assert(de_get_len(hid_service_record) <= sizeof(hid_service_record));
    sdp_register_service(hid_service_record);

    // Device ID SDP (uso de Bluetooth Vendor ID da BlueKitchen)
    device_id_create_sdp_record(device_id_record, sdp_create_service_record_handle(),
                                DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH,
                                BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
    btstack_assert(de_get_len(device_id_record) <= sizeof(device_id_record));
    sdp_register_service(device_id_record);

    // Inicializa o dispositivo HID
    hid_device_init(hid_boot_mode, sizeof(hid_descriptor_keyboard), hid_descriptor_keyboard);

    // Registra callback para eventos HCI e HID
    static btstack_packet_callback_registration_t hci_cb;
    hci_cb.callback = &packet_handler;
    hci_add_event_handler(&hci_cb);
    hid_device_register_packet_handler(&packet_handler);

#ifdef HAVE_BTSTACK_STDIN
    // Se usar stdin (host PC), configurar parsing de endereço BD
    // (Exemplo de BD Addr fixo; no seu caso, remova ou ajuste conforme necessário)
    // bd_addr_t bd_addr;
    // sscanf_bd_addr("00:1A:7D:DA:71:13", bd_addr);
    // btstack_stdin_setup(stdin_process);
#endif

    // Inicializa ring buffer para caracteres
    btstack_ring_buffer_init(&ring_buffer, ring_buffer_storage, sizeof(ring_buffer_storage));

    // Liga o controlador Bluetooth
    hci_power_control(HCI_POWER_ON);

    return 0;
}
