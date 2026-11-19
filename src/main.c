/* ============================================================================
 * PROJECT: S32K144 - BUOC 4: CANH BAO NHIET DO PIN LAPTOP QUA CAN
 *
 * Cung 1 file main.c cho ca 2 board, chi doi BOARD_NUMBER truoc khi nap:
 *   - Board 1 (node cam bien): doc chan DO cua module nhiet dien tro, moi 500 ms
 *     gui 1 khung CAN ID 0x101:
 *       [0] = trang thai pin: 0 = binh thuong, 1 = qua nong
 *       [1] = so dem khung (tang 1 moi lan gui)
 *       [2] = muc logic tho cua chan DO (de debug)
 *   - Board 2 (node hien thi): nhan khung ID 0x101, hien len LCD 1602 (I2C 0x27)
 *
 * Module cam bien chi co DO (khong co AO): DO doi muc khi nhiet do vuot nguong
 * chinh bang bien tro xanh (den DO-LED tren module sang khi vuot nguong).
 *
 * Noi day:
 *   Board 1: module VCC -> VDD (J8), GND -> GND, DO -> ADC0 SEL12 (PTC14) tren J8
 *   Board 2: LCD SCL -> PTA3, SDA -> PTA2 (qua level shifter nhu bai I2C), GND chung
 *   CAN    : J13 CANH - CANH, CANL - CANL, GND chung
 *
 * LED02 (LED 3 mau, sang khi chan o muc 0):
 *   Board 1: xanh duong nhay = gui CAN duoc, do nhay = khong ai nhan
 *   Board 2: xanh duong nhay = pin binh thuong, do nhay = pin qua nong hoac mat ket noi
 * ============================================================================*/

#include "sdk_project_config.h"
#include "osif.h"
#include <stdint.h>
#include <stdbool.h>

/* ===== DOI SO NAY TRUOC KHI NAP: 1 = board cam bien, 2 = board LCD ===== */
#define BOARD_NUMBER        1u
/* 1 = board 2 hien len LCD 1602 (can them i2c_pal + chan PTA2/PTA3 + file lcd1602_i2c)
 * 0 = khong dung LCD, board 2 chi bao bang LED02 (khong can cau hinh them gi) */
#define USE_LCD             1u

#if (USE_LCD == 1u)
#include "lcd1602_i2c.h"
#endif

/* --- FlexCAN (ten lay theo file Peripherals Tool sinh ra) --- */
#define CAN_INST            INST_FLEXCAN_CONFIG_1
#define CAN_RX_MB           0u
#define CAN_TX_MB           1u
#define CAN_ID_BATTERY      0x101u
#define CAN_TX_TIMEOUT_MS   50u
#define CAN_RX_TIMEOUT_MS   2000u   /* board 1 gui moi 500 ms, qua 2 s la mat ket noi */

/* --- Cam bien: chan DO tren PTC14 (J8 chan ADC0 SEL12, dung nhu GPIO vao) ---
 * Muon doi chan khac thi sua 3 dong duoi, vi du ADC0 SEL13 = PORTC/PTC/15u,
 * GPIO1 = PORTD/PTD/17u */
#define SENSOR_PORT         PORTC
#define SENSOR_GPIO         PTC
#define SENSOR_PIN          14u
#define SENSOR_MASK         (1u << SENSOR_PIN)
/* Muc logic cua DO khi nhiet do VUOT nguong. Module LM393 thuong keo DO xuong 0
 * khi vuot nguong (den DO-LED sang). Neu LCD bao nguoc thi doi thanh 1u. */
#define SENSOR_HOT_LEVEL    0u
#define SAMPLE_PERIOD_MS    100u
#define SEND_EVERY_SAMPLES  5u      /* 5 x 100 ms = gui moi 500 ms */
#define DEBOUNCE_SAMPLES    3u      /* doc giong nhau 3 lan lien tiep moi doi trang thai */

#define BATTERY_NORMAL      0u
#define BATTERY_HOT         1u

/* --- LCD --- */
#define LCD_ADDR_PCF8574    0x27u
#define LCD_ADDR_PCF8574A   0x3Fu

/* LED02 tren board: PTD15 = do, PTD16 = xanh duong, sang khi chan o muc 0 */
#define LED_RED_MASK        (1u << 15u)
#define LED_BLUE_MASK       (1u << 16u)

/* Bien toan cuc de xem trong debugger */
volatile uint32_t g_txOk = 0u;
volatile uint32_t g_txErr = 0u;
volatile uint32_t g_rxOk = 0u;
volatile uint8_t  g_batteryState = BATTERY_NORMAL;
volatile uint8_t  g_sensorRaw = 0u;
flexcan_msgbuff_t g_rxMsg;

static const flexcan_data_info_t s_dataInfo = {
    .msg_id_type = FLEXCAN_MSG_ID_STD,
    .data_length = 8u,
    .fd_enable   = false,
    .fd_padding  = 0u,
    .enable_brs  = false,
    .is_remote   = false
};

static void led_init(void)
{
    PORTD->PCR[15] = PORT_PCR_MUX(1u);
    PORTD->PCR[16] = PORT_PCR_MUX(1u);
    PTD->PSOR = LED_RED_MASK | LED_BLUE_MASK;      /* muc 1 = tat */
    PTD->PDDR |= LED_RED_MASK | LED_BLUE_MASK;
}

/* ok = true: tat do, nhay xanh duong. ok = false: tat xanh duong, nhay do */
static void led_show(bool ok)
{
    if (ok)
    {
        PTD->PSOR = LED_RED_MASK;
        PTD->PTOR = LED_BLUE_MASK;
    }
    else
    {
        PTD->PSOR = LED_BLUE_MASK;
        PTD->PTOR = LED_RED_MASK;
    }
}

#if (BOARD_NUMBER == 1u)
/* ================== BOARD 1: NODE CAM BIEN ================== */
static void sensor_init(void)
{
    /* Chan DO la GPIO vao, bat dien tro keo len (module cung co san keo len) */
    SENSOR_PORT->PCR[SENSOR_PIN] = PORT_PCR_MUX(1u) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    SENSOR_GPIO->PDDR &= ~SENSOR_MASK;
}

static uint8_t sensor_read_raw(void)
{
    return ((SENSOR_GPIO->PDIR & SENSOR_MASK) != 0u) ? 1u : 0u;
}

static void node_sensor_run(void)
{
    uint8_t counter = 0u;
    uint8_t sameCount = 0u;
    uint8_t lastRaw;
    uint8_t tx[8] = {0u};
    bool txOk = false;

    sensor_init();
    lastRaw = sensor_read_raw();

    for (uint32_t tick = 0u; ; tick++)
    {
        /* Chong doi: chi doi trang thai khi DO giu nguyen DEBOUNCE_SAMPLES lan */
        g_sensorRaw = sensor_read_raw();
        if (g_sensorRaw == lastRaw)
        {
            if (sameCount < DEBOUNCE_SAMPLES)
            {
                sameCount++;
            }
        }
        else
        {
            lastRaw = g_sensorRaw;
            sameCount = 1u;
        }
        if (sameCount >= DEBOUNCE_SAMPLES)
        {
            g_batteryState = (lastRaw == SENSOR_HOT_LEVEL) ? BATTERY_HOT : BATTERY_NORMAL;
        }

        if ((tick % SEND_EVERY_SAMPLES) == 0u)
        {
            tx[0] = g_batteryState;
            tx[1] = counter++;
            tx[2] = g_sensorRaw;
            txOk = (FLEXCAN_DRV_SendBlocking(CAN_INST, CAN_TX_MB, &s_dataInfo, CAN_ID_BATTERY,
                                             tx, CAN_TX_TIMEOUT_MS) == STATUS_SUCCESS);
            if (txOk)
            {
                g_txOk++;
            }
            else
            {
                g_txErr++;
            }
            led_show(txOk);
        }

        OSIF_TimeDelay(SAMPLE_PERIOD_MS);
    }
}

#else
/* ================== BOARD 2: NODE HIEN THI LCD ================== */
#if (USE_LCD == 1u)
static bool s_lcdReady = false;

/* Thu 2 dia chi pho bien cua module PCF8574 */
static void lcd_try_init(void)
{
    s_lcdReady = (LCD1602_Init(LCD_ADDR_PCF8574) == STATUS_SUCCESS) ||
                 (LCD1602_Init(LCD_ADDR_PCF8574A) == STATUS_SUCCESS);
}

/* In 1 dong, them dau cach cho du 16 ky tu de xoa chu cu */
static void lcd_line(uint8_t row, const char *text)
{
    char buf[17];
    uint8_t i = 0u;

    if (!s_lcdReady)
    {
        return;
    }
    while ((text[i] != '\0') && (i < 16u))
    {
        buf[i] = text[i];
        i++;
    }
    while (i < 16u)
    {
        buf[i++] = ' ';
    }
    buf[16] = '\0';

    LCD1602_SetCursor(row, 0u);
    LCD1602_Print(buf);
}
#else
/* Khong dung LCD: cac ham LCD thanh rong, chi con LED02 bao trang thai */
static void lcd_try_init(void) {}
static void lcd_line(uint8_t row, const char *text) { (void)row; (void)text; }
#endif

/* Cac man hinh LCD */
#define SCREEN_NONE         0u
#define SCREEN_NORMAL       1u
#define SCREEN_HOT          2u
#define SCREEN_LOST         3u
/* Khoi tao lai LCD moi ~5 s (10 vong x 500 ms): neu 1 lan truyen I2C bi loi lam
 * LCD lech nhip 4 bit (hien ky tu rac) thi tu het loi sau toi da 5 s */
#define LCD_REFRESH_LOOPS   10u

static void screen_draw(uint8_t screen)
{
    switch (screen)
    {
        case SCREEN_NORMAL:
            lcd_line(0u, "PIN LAPTOP:");
            lcd_line(1u, "Binh thuong");
            break;
        case SCREEN_HOT:
            lcd_line(0u, "PIN LAPTOP:");
            lcd_line(1u, "QUA NONG!");
            break;
        case SCREEN_LOST:
            lcd_line(0u, "Mat ket noi CAN");
            lcd_line(1u, "Kiem tra CANH/L");
            break;
        default:
            lcd_line(0u, "CANH BAO PIN");
            lcd_line(1u, "Cho du lieu CAN");
            break;
    }
}

static void node_display_run(void)
{
    status_t st;
    uint8_t screen = SCREEN_NONE;
    uint8_t shown = SCREEN_NONE;
    bool redraw = false;
    uint32_t loops = 0u;

#if (USE_LCD == 1u)
    (void)I2C_MasterInit(&lpi2c0_instance, &i2c_pal_lpi2c0_MasterConfig0);
#endif
    lcd_try_init();
    screen_draw(SCREEN_NONE);

    (void)FLEXCAN_DRV_ConfigRxMb(CAN_INST, CAN_RX_MB, &s_dataInfo, CAN_ID_BATTERY);

    for (;;)
    {
        st = FLEXCAN_DRV_ReceiveBlocking(CAN_INST, CAN_RX_MB, &g_rxMsg, CAN_RX_TIMEOUT_MS);

        if ((st == STATUS_SUCCESS) && (g_rxMsg.msgId == CAN_ID_BATTERY))
        {
            g_rxOk++;
            g_batteryState = g_rxMsg.data[0];
            screen = (g_batteryState == BATTERY_HOT) ? SCREEN_HOT : SCREEN_NORMAL;
        }
        else if (st == STATUS_TIMEOUT)
        {
            /* 2 s khong co khung nao: board 1 tat, dut day CAN, sai toc do bit */
            screen = SCREEN_LOST;
        }
        else
        {
            /* Loi nhan khac: giu man hinh cu */
        }
        led_show(screen == SCREEN_NORMAL);

        /* Chi ghi LCD khi noi dung doi (it truyen I2C -> it loi),
         * va dinh ky khoi tao lai LCD de tu sua neu bi hien ky tu rac */
        loops++;
        if (loops >= LCD_REFRESH_LOOPS)
        {
            loops = 0u;
            lcd_try_init();
            redraw = true;
        }
        if (redraw || (screen != shown))
        {
            screen_draw(screen);
            shown = screen;
            redraw = false;
        }
    }
}
#endif

int main(void)
{
    CLOCK_DRV_Init(&clockMan1_InitConfig0);
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0, g_pin_mux_InitConfigArr0);
    led_init();
    (void)FLEXCAN_DRV_Init(CAN_INST, &flexcanState0, &flexcanInitConfig0);

#if (BOARD_NUMBER == 1u)
    node_sensor_run();
#else
    node_display_run();
#endif

    return 0;
}
