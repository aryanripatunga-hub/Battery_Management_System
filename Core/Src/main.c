#include "main.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ── Peripheral handles  */
ADC_HandleTypeDef  hadc1;
ADC_HandleTypeDef  hadc2;
ADC_HandleTypeDef  hadc3;
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart3;


/* ── VOLTAGE  */
volatile uint16_t raw_cell1, raw_cell2, raw_cell3;
volatile float    v_pin1,    v_pin2,    v_pin3;
volatile float    v_cell1,   v_cell2,   v_cell3;

/* ── TEMPERATURE  */
volatile uint16_t raw_ntc1, raw_ntc2, raw_ntc3;
volatile float    temp_c1,  temp_c2,  temp_c3;

/* ── CURRENT / POWER (INA219)  */
volatile uint8_t  ina219_ok;
volatile float    shunt_mV;
volatile float    bus_V;
volatile float    current_A;
volatile float    power_W;
volatile uint32_t i2c_error_count;

/* ── PROTECTION  */
volatile uint8_t  fault_uvp;
volatile uint8_t  fault_ocp;
volatile uint8_t  fault_otp;
volatile uint8_t  fault_any;
volatile uint8_t  fet_on;

/* ── BALANCING  */
volatile uint8_t  bal_request1, bal_request2, bal_request3;
volatile uint8_t  bal_active1,  bal_active2,  bal_active3;
volatile float    v_max, v_min, v_spread;

/* ── SOC  */
volatile float    soc_cell1, soc_cell2, soc_cell3, soc_pack;

/* ── SOH  */
volatile float    r_internal1, r_internal2, r_internal3;
volatile float    soh_cell1,   soh_cell2,   soh_cell3, soh_pack;

/* ── SELF-TEST  */
volatile uint8_t  test_mode;

/* ── CONSTANTS  */
#define VREF              3.3f
#define ADC_FS            4095.0f
#define RATIO             (330.0f / (100.0f + 330.0f))

#define NTC_BETA          3950.0f
#define NTC_NOM_R         10000.0f
#define NTC_NOM_T         298.15f
#define NTC_PULLUP        10000.0f

#define UVP_THRESH_V      2.0f
#define OCP_THRESH_A      3.0f
#define OTP_THRESH_C      55.0f
#define OTP_RECOV_C       45.0f
#define OCP_DEBOUNCE      2u

#define PROT_PORT         GPIOB
#define PROT_PIN          GPIO_PIN_1

#define BAL_START_V       0.050f
#define BAL_STOP_V        0.020f
#define BAL_SAFE_TEMP     50.0f
#define BAL1_PORT         GPIOC
#define BAL1_PIN          GPIO_PIN_4
#define BAL2_PORT         GPIOC
#define BAL2_PIN          GPIO_PIN_5
#define BAL3_PORT         GPIOC
#define BAL3_PIN          GPIO_PIN_6

#define INA219_ADDR       (0x40 << 1)
#define REG_CONFIG        0x00
#define REG_SHUNT         0x01
#define REG_BUS           0x02
#define REG_POWER         0x03
#define REG_CURRENT       0x04
#define REG_CALIB         0x05
#define CAL_VALUE         410
#define I2C_TIMEOUT       10u
#define I2C_ERR_LIMIT     3u

#define SOC_TABLE_SIZE    12
static const float SOC_OCV_V[SOC_TABLE_SIZE] = {
    3.00f, 3.20f, 3.40f, 3.50f, 3.60f, 3.65f,
    3.70f, 3.75f, 3.80f, 3.90f, 4.05f, 4.20f
};
static const float SOC_PCT[SOC_TABLE_SIZE] = {
      0.0f,  3.0f,  8.0f, 15.0f, 28.0f, 40.0f,
     52.0f, 62.0f, 72.0f, 84.0f, 95.0f,100.0f
};

#define RI_NEW_OHM        0.080f
#define RI_MIN_CURRENT    0.200f

#define JSON_BUF_SZ       320
static char    json_buf[JSON_BUF_SZ];
static uint8_t uart_rx_byte;

/* ── Prototypes  */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_Init(void);

static uint16_t          ADC1_ReadChannel(uint32_t ch);
static float             NTC_ToCelsius(uint16_t raw);
void                     BMS_ReadVoltages(void);
void                     BMS_ReadTemperatures(void);

static HAL_StatusTypeDef INA219_Write(uint8_t reg, uint16_t val);
static HAL_StatusTypeDef INA219_Read(uint8_t reg, int16_t *out);
static void              I2C1_RecoverBus(void);
void                     INA219_Init(void);
void                     INA219_ReadAll(void);

static void              FET_TurnOn(void);
static void              FET_TurnOff(void);
void                     BMS_ProtectionCheck(void);
void                     BMS_UpdateBalancing(void);

static float             OCV_ToSOC(float v_ocv);
void                     BMS_UpdateSOC(void);
void                     BMS_UpdateSOH(void);

void                     BMS_SelfTest_ProcessCommand(void);
static void              UART_Print(const char *s);
void                     BMS_SendJSON(void);

/* ============================================================================
   ADC + NTC
   ============================================================================ */

static uint16_t ADC1_ReadChannel(uint32_t ch)
{
    ADC_ChannelConfTypeDef s = {0};
    s.Channel      = ch;
    s.Rank         = 1;
    s.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &s);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t v = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return v;
}

static float NTC_ToCelsius(uint16_t raw)
{
    if (raw == 0)    return  125.0f;
    if (raw >= 4095) return  -40.0f;
    float v  = raw * (VREF / ADC_FS);
    float r  = NTC_PULLUP * v / (VREF - v);
    float tk = 1.0f / ((1.0f / NTC_NOM_T) +
               (1.0f / NTC_BETA) * logf(r / NTC_NOM_R));
    return tk - 273.15f;
}

void BMS_ReadVoltages(void)
{
    ADC_ChannelConfTypeDef s = {0};
    s.Rank         = 1;
    s.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    /* Cell 1 — ADC1 CH0 PA0 */
    s.Channel = ADC_CHANNEL_0;
    HAL_ADC_ConfigChannel(&hadc1, &s);
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        raw_cell1 = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    /* Cell 2 — ADC2 CH1 PA1  */
    s.Channel = ADC_CHANNEL_1;
    HAL_ADC_ConfigChannel(&hadc2, &s);
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
        raw_cell2 = (uint16_t)HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);

    /* Cell 3 — ADC3 CH10 PC0 */
    s.Channel = ADC_CHANNEL_10;
    HAL_ADC_ConfigChannel(&hadc3, &s);
    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK)
        raw_cell3 = (uint16_t)HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);

    v_pin1  = (raw_cell1 / ADC_FS) * VREF;
    v_pin2  = (raw_cell2 / ADC_FS) * VREF;
    v_pin3  = (raw_cell3 / ADC_FS) * VREF;
    v_cell1 = v_pin1 / RATIO;
    v_cell2 = v_pin2 / RATIO;
    v_cell3 = v_pin3 / RATIO;
}

void BMS_ReadTemperatures(void)
{
    raw_ntc1 = ADC1_ReadChannel(ADC_CHANNEL_11);  /* PC1 */
    raw_ntc2 = ADC1_ReadChannel(ADC_CHANNEL_8);   /* PB0 */
    raw_ntc3 = ADC1_ReadChannel(ADC_CHANNEL_13);  /* PC3 */
    temp_c1  = NTC_ToCelsius(raw_ntc1);
    temp_c2  = NTC_ToCelsius(raw_ntc2);
    temp_c3  = NTC_ToCelsius(raw_ntc3);
}

/* ============================================================================
   INA219
   ============================================================================ */

static HAL_StatusTypeDef INA219_Write(uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDR, buf, 3, I2C_TIMEOUT);
}

static HAL_StatusTypeDef INA219_Read(uint8_t reg, int16_t *out)
{
    uint8_t buf[2] = {0};
    HAL_StatusTypeDef st;
    st = HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDR, &reg, 1, I2C_TIMEOUT);
    if (st != HAL_OK) return st;
    st = HAL_I2C_Master_Receive(&hi2c1, INA219_ADDR, buf, 2, I2C_TIMEOUT);
    if (st != HAL_OK) return st;
    *out = (int16_t)((buf[0] << 8) | buf[1]);
    return HAL_OK;
}

static void I2C1_RecoverBus(void)
{
    GPIO_InitTypeDef g = {0};
    HAL_I2C_DeInit(&hi2c1);
    g.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1);
    for (int i = 0; i < 9; i++) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) break;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   HAL_Delay(1);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   HAL_Delay(1);
    MX_I2C1_Init();
    HAL_Delay(5);
}

void INA219_Init(void)
{
    if (INA219_Write(REG_CONFIG, 0x399F) != HAL_OK) { ina219_ok = 0; return; }
    if (INA219_Write(REG_CALIB,  CAL_VALUE) != HAL_OK) { ina219_ok = 0; return; }
    ina219_ok = 1;
}

void INA219_ReadAll(void)
{
    int16_t           raw  = 0;
    HAL_StatusTypeDef st;
    static uint32_t   errs = 0;

    st = INA219_Read(REG_SHUNT,   &raw);
    if (st == HAL_OK) shunt_mV  = raw * 0.01f;                       else { i2c_error_count++; errs++; }
    st = INA219_Read(REG_BUS,     &raw);
    if (st == HAL_OK) bus_V     = ((uint16_t)raw >> 3) * 0.004f;     else { i2c_error_count++; errs++; }
    st = INA219_Read(REG_CURRENT, &raw);
    if (st == HAL_OK) current_A = raw * 0.001f;                      else { i2c_error_count++; errs++; }
    st = INA219_Read(REG_POWER,   &raw);
    if (st == HAL_OK) power_W   = (uint16_t)raw * 0.02f;             else { i2c_error_count++; errs++; }

    if      (errs >= I2C_ERR_LIMIT) { errs = 0; I2C1_RecoverBus(); INA219_Init(); }
    else if (st == HAL_OK)          { errs = 0; }
}

/* ============================================================================
   PROTECTION FET
   ============================================================================ */

static void FET_TurnOn(void)
{
    HAL_GPIO_WritePin(PROT_PORT, PROT_PIN, GPIO_PIN_SET);
    fet_on = 1;
}

static void FET_TurnOff(void)
{
    HAL_GPIO_WritePin(PROT_PORT, PROT_PIN, GPIO_PIN_RESET);
    fet_on = 0;
}

void BMS_ProtectionCheck(void)
{
    /* Under-voltage (latching) */
    if (v_cell1 < UVP_THRESH_V || v_cell2 < UVP_THRESH_V || v_cell3 < UVP_THRESH_V)
        fault_uvp = 1;

    /* Over-current (debounced, latching) */
    static uint32_t ocp_cnt = 0;
    if (current_A > OCP_THRESH_A) {
        if (++ocp_cnt >= OCP_DEBOUNCE) { fault_ocp = 1; ocp_cnt = 0; }
    } else { ocp_cnt = 0; }

    /* Over-temperature (auto-recovers) */
    if (temp_c1 > OTP_THRESH_C || temp_c2 > OTP_THRESH_C || temp_c3 > OTP_THRESH_C)
        fault_otp = 1;
    else if (fault_otp &&
             temp_c1 < OTP_RECOV_C && temp_c2 < OTP_RECOV_C && temp_c3 < OTP_RECOV_C)
        fault_otp = 0;

    fault_any = (fault_uvp | fault_ocp | fault_otp) ? 1u : 0u;
    if (fault_any) FET_TurnOff(); else FET_TurnOn();
}

/* ============================================================================
   PASSIVE BALANCING
   ============================================================================ */

void BMS_UpdateBalancing(void)
{
    v_max = v_cell1;
    if (v_cell2 > v_max) v_max = v_cell2;
    if (v_cell3 > v_max) v_max = v_cell3;
    v_min = v_cell1;
    if (v_cell2 < v_min) v_min = v_cell2;
    if (v_cell3 < v_min) v_min = v_cell3;
    v_spread = v_max - v_min;

    /* Hysteresis */
    if (v_spread <= BAL_STOP_V) {
        bal_request1 = bal_request2 = bal_request3 = 0;
    } else if (v_spread >= BAL_START_V) {
        /* Discharge only the cell(s) sitting within 5 mV of the pack maximum */
        bal_request1 = (v_cell1 >= v_max - 0.005f) ? 1u : 0u;
        bal_request2 = (v_cell2 >= v_max - 0.005f) ? 1u : 0u;
        bal_request3 = (v_cell3 >= v_max - 0.005f) ? 1u : 0u;
    }

    if (fault_any)               { bal_request1 = bal_request2 = bal_request3 = 0; }
    if (temp_c1 > BAL_SAFE_TEMP) bal_request1 = 0;
    if (temp_c2 > BAL_SAFE_TEMP) bal_request2 = 0;
    if (temp_c3 > BAL_SAFE_TEMP) bal_request3 = 0;

    bal_active1 = bal_request1;
    bal_active2 = bal_request2;
    bal_active3 = bal_request3;
    HAL_GPIO_WritePin(BAL1_PORT, BAL1_PIN, bal_active1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BAL2_PORT, BAL2_PIN, bal_active2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BAL3_PORT, BAL3_PIN, bal_active3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ============================================================================
   SOC
   ============================================================================ */

static float OCV_ToSOC(float v_ocv)
{
    if (v_ocv <= SOC_OCV_V[0])                  return SOC_PCT[0];
    if (v_ocv >= SOC_OCV_V[SOC_TABLE_SIZE - 1]) return SOC_PCT[SOC_TABLE_SIZE - 1];
    for (int i = 0; i < SOC_TABLE_SIZE - 1; i++) {
        if (v_ocv >= SOC_OCV_V[i] && v_ocv <= SOC_OCV_V[i + 1]) {
            float slope = (SOC_PCT[i+1] - SOC_PCT[i]) /
                          (SOC_OCV_V[i+1] - SOC_OCV_V[i]);
            return SOC_PCT[i] + slope * (v_ocv - SOC_OCV_V[i]);
        }
    }
    return 0.0f;
}

void BMS_UpdateSOC(void)
{
    float ia   = (current_A < 0.0f) ? -current_A : current_A;
    float voc1 = v_cell1 + ia * r_internal1;
    float voc2 = v_cell2 + ia * r_internal2;
    float voc3 = v_cell3 + ia * r_internal3;
    if (voc1 > 4.25f) voc1 = 4.25f;
    if (voc2 > 4.25f) voc2 = 4.25f;
    if (voc3 > 4.25f) voc3 = 4.25f;

    soc_cell1 = OCV_ToSOC(voc1);
    soc_cell2 = OCV_ToSOC(voc2);
    soc_cell3 = OCV_ToSOC(voc3);

    soc_pack = soc_cell1;
    if (soc_cell2 < soc_pack) soc_pack = soc_cell2;
    if (soc_cell3 < soc_pack) soc_pack = soc_cell3;
}

void BMS_UpdateSOH(void)
{
    static float v_rest1 = 0.0f, v_rest2 = 0.0f, v_rest3 = 0.0f;
    float ia = (current_A < 0.0f) ? -current_A : current_A;

    if (ia < RI_MIN_CURRENT) {
        v_rest1 = v_cell1;
        v_rest2 = v_cell2;
        v_rest3 = v_cell3;
        return;
    }

#define UPDATE_RI(ri, vr, vc) do {                   \
    if ((vr) > 0.0f && (vr) > (vc)) {               \
        (ri) = ((vr) - (vc)) / ia;                  \
        if ((ri) < 0.01f) (ri) = 0.01f;             \
        if ((ri) > 1.00f) (ri) = 1.00f;             \
    }                                                \
} while(0)

#define UPDATE_SOH(soh, ri) do {                     \
    (soh) = 100.0f * (RI_NEW_OHM / (ri));           \
    if ((soh) > 100.0f) (soh) = 100.0f;             \
    if ((soh) <   0.0f) (soh) =   0.0f;             \
} while(0)

    UPDATE_RI(r_internal1, v_rest1, v_cell1);
    UPDATE_RI(r_internal2, v_rest2, v_cell2);
    UPDATE_RI(r_internal3, v_rest3, v_cell3);

    if (r_internal1 > 0.0f) UPDATE_SOH(soh_cell1, r_internal1);
    if (r_internal2 > 0.0f) UPDATE_SOH(soh_cell2, r_internal2);
    if (r_internal3 > 0.0f) UPDATE_SOH(soh_cell3, r_internal3);

    soh_pack = soh_cell1;
    if (soh_cell2 < soh_pack) soh_pack = soh_cell2;
    if (soh_cell3 < soh_pack) soh_pack = soh_cell3;
}

/* ============================================================================
   UART HELPERS
   ============================================================================ */

static void UART_Print(const char *s)
{
    HAL_UART_Transmit(&huart3
	, (const uint8_t *)s, (uint16_t)strlen(s), 200);
}

/* ============================================================================
   SELF-TEST
   ============================================================================ */

void BMS_SelfTest_ProcessCommand(void)
{
    if (HAL_UART_Receive(&huart3
			, &uart_rx_byte, 1, 0) != HAL_OK)
        return;

    char buf[256];

    switch (uart_rx_byte)
    {
        case 'U': case 'u':
            test_mode = 1; fault_uvp = 1; fault_any = 1; FET_TurnOff();
            UART_Print("[TEST] UVP injected. PB1=LOW. Send R to reset.\r\n");
            break;

        case 'O': case 'o':
            test_mode = 2; fault_ocp = 1; fault_any = 1; FET_TurnOff();
            UART_Print("[TEST] OCP injected. PB1=LOW. Send R to reset.\r\n");
            break;

        case 'T': case 't':
            test_mode = 3; fault_otp = 1; fault_any = 1; FET_TurnOff();
            UART_Print("[TEST] OTP injected. PB1=LOW. Send R to reset.\r\n");
            break;

        case 'R': case 'r':
            test_mode = 0;
            fault_uvp = 0; fault_ocp = 0; fault_otp = 0; fault_any = 0;
            FET_TurnOn();
            UART_Print("[TEST] All faults cleared. PB1=HIGH.\r\n");
            break;

        case 'S': case 's':
            snprintf(buf, sizeof(buf),
                "[SOC] C1=%.1f%% C2=%.1f%% C3=%.1f%% Pack=%.1f%%\r\n"
                "[SOH] C1=%.1f%% C2=%.1f%% C3=%.1f%% Pack=%.1f%%\r\n"
                "[Ri]  C1=%.4f C2=%.4f C3=%.4f ohm\r\n"
                "[V]   C1=%.3f C2=%.3f C3=%.3f V  I=%.3f A\r\n"
                "[T]   C1=%.1f C2=%.1f C3=%.1f degC\r\n",
                (double)soc_cell1,(double)soc_cell2,(double)soc_cell3,(double)soc_pack,
                (double)soh_cell1,(double)soh_cell2,(double)soh_cell3,(double)soh_pack,
                (double)r_internal1,(double)r_internal2,(double)r_internal3,
                (double)v_cell1,(double)v_cell2,(double)v_cell3,(double)current_A,
                (double)temp_c1,(double)temp_c2,(double)temp_c3);
            UART_Print(buf);
            break;

        default:
            break;
    }
}


void BMS_SendJSON(void)
{
    int len = snprintf(json_buf, JSON_BUF_SZ,
        "{"
        "\"v1\":%.3f,\"v2\":%.3f,\"v3\":%.3f,"
        "\"t1\":%.1f,\"t2\":%.1f,\"t3\":%.1f,"
        "\"i\":%.3f,\"pw\":%.2f,\"bv\":%.2f,"
        "\"s1\":%.1f,\"s2\":%.1f,\"s3\":%.1f,\"sp\":%.1f,"
        "\"h1\":%.1f,\"h2\":%.1f,\"h3\":%.1f,\"hp\":%.1f,"
        "\"r1\":%.4f,\"r2\":%.4f,\"r3\":%.4f,"
        "\"fuvp\":%u,\"focp\":%u,\"fotp\":%u,"
        "\"fa\":%u,\"fon\":%u,"
        "\"b1\":%u,\"b2\":%u,\"b3\":%u,"
        "\"tm\":%u"
        "}\n",
        (double)v_cell1,    (double)v_cell2,    (double)v_cell3,
        (double)temp_c1,    (double)temp_c2,    (double)temp_c3,
        (double)current_A,  (double)power_W,    (double)bus_V,
        (double)soc_cell1,  (double)soc_cell2,  (double)soc_cell3,  (double)soc_pack,
        (double)soh_cell1,  (double)soh_cell2,  (double)soh_cell3,  (double)soh_pack,
        (double)r_internal1,(double)r_internal2,(double)r_internal3,
        (unsigned)fault_uvp,(unsigned)fault_ocp,(unsigned)fault_otp,
        (unsigned)fault_any,(unsigned)fet_on,
        (unsigned)bal_active1,(unsigned)bal_active2,(unsigned)bal_active3,
        (unsigned)test_mode
    );

    if (len > 0 && len < JSON_BUF_SZ)
        HAL_UART_Transmit(&huart3
		, (uint8_t *)json_buf, (uint16_t)len, 300);
}

/* ============================================================================
   MAIN
   ============================================================================ */

int main(void)

{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_I2C1_Init();
    MX_USART3_Init();

    /* Safe startup */
    FET_TurnOff();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
    fault_uvp = 0; fault_ocp = 0; fault_otp = 0; fault_any = 0;
    test_mode = 0;

    /* FIX-2: seed Ri so SOH is valid from frame 1 */
    r_internal1 = RI_NEW_OHM;
    r_internal2 = RI_NEW_OHM;
    r_internal3 = RI_NEW_OHM;
    soh_cell1 = soh_cell2 = soh_cell3 = soh_pack = 100.0f;

    HAL_Delay(100);
    INA219_Init();

    UART_Print("{\"msg\":\"3S BMS online\"}\n");

    /* First measurement pass */
    BMS_ReadVoltages();
    BMS_ReadTemperatures();
    INA219_ReadAll();
    BMS_UpdateSOH();
    BMS_UpdateSOC();
    BMS_ProtectionCheck();

    /* Main loop */
    while (1)
    {
        BMS_SelfTest_ProcessCommand();

        BMS_ReadVoltages();
        BMS_ReadTemperatures();
        INA219_ReadAll();

        BMS_UpdateSOH();
        BMS_UpdateSOC();

        if (test_mode == 0)
            BMS_ProtectionCheck();

        BMS_UpdateBalancing();

        BMS_SendJSON();

        HAL_Delay(500);
    }
}

/* ============================================================================
   PERIPHERAL INIT — unchanged from original
   ============================================================================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 2;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef s = {0};
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);
    s.Channel = ADC_CHANNEL_0; s.Rank = 1; s.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &s);
}

static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef s = {0};
    hadc2.Instance                   = ADC2;
    hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode          = DISABLE;
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion       = 1;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc2);
    s.Channel = ADC_CHANNEL_1; s.Rank = 1; s.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&hadc2, &s);
}

static void MX_ADC3_Init(void)
{
    ADC_ChannelConfTypeDef s = {0};
    hadc3.Instance                   = ADC3;
    hadc3.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc3.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode          = DISABLE;
    hadc3.Init.ContinuousConvMode    = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion       = 1;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc3);
    s.Channel = ADC_CHANNEL_10; s.Rank = 1; s.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&hadc3, &s);
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void MX_USART3_Init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* ── Protection FET gate — PB1 — default OFF ── */
    HAL_GPIO_WritePin(PROT_PORT, PROT_PIN, GPIO_PIN_RESET);
    g.Pin       = PROT_PIN;
    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = 0;
    HAL_GPIO_Init(PROT_PORT, &g);

    /* ── Balancing FET gates — PC4/PC5/PC6 — default OFF ── */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
    g.Pin       = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = 0;
    HAL_GPIO_Init(GPIOC, &g);

    /* ── USART3 — PC10 TX, PC11 RX — AF7 ── */
    __HAL_RCC_USART3_CLK_ENABLE();
    g.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &g);
}

void Error_Handler(void)
{
    FET_TurnOff();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
    __disable_irq();
    while (1) {}
}