#include <ch32v00X.h>
#include <debug.h>
#include <string.h>

#include "protocol_v2.h"

#define AS5600_ADDR 0x36u
#define AS5600_STATUS 0x0Bu
#define AS5600_RAW_ANGLE 0x0Cu
#define TMAG_MANUFACTURER_LSB 0x0Eu
#define TMAG_SENSOR_CONFIG_1 0x02u
#define TMAG_DEVICE_CONFIG_2 0x01u
#define TMAG_X_RESULT 0x12u
#define CAP_AS5600 (1u << 0)
#define CAP_TMAG5273 (1u << 1)
#define CAP_JOYSTICK (1u << 2)
#define CAP_BUTTONS (1u << 3)
#define LOOP_US 50u
#define TICKS_PER_MS (1000u / LOOP_US)
#define SAMPLE_TICKS (2000u / LOOP_US)
#define CRC_LED_TICKS (2000000u / LOOP_US)
#define BUTTON_STAT_MIN_TICKS (200000u / LOOP_US)

static const uint8_t tmag_addresses[] = {0x35u, 0x22u, 0x78u, 0x44u};

static CctrlBusParser parser;
static uint8_t node_id = 0xFF;
static uint8_t node_type = CCTRL_NODE_ENCODER;
static uint8_t enumerated;
static uint8_t encoder_status = CCTRL_ENC_DATA_STALE;
static uint8_t handle_status = CCTRL_HANDLE_DATA_STALE;
static uint8_t handle_buttons;
static uint8_t handle_buttons_down;
static uint8_t handle_button_stat_active;
static uint8_t tmag_address;
static uint16_t raw_angle;
static uint16_t joystick_x;
static uint16_t joystick_y;
static int16_t magnetic_x;
static int16_t magnetic_y;
static int16_t magnetic_z;
static uint32_t ticks;
static uint32_t last_good_tick;
static uint32_t crc_error_until_tick;
static uint32_t handle_button_stat_until_tick;
static volatile uint8_t rx_ring[128];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;

static void gpio_init(void);
static void startup_led_test(void);
static void uart_init(void);
static void i2c_init(void);
static void adc_init(void);
static uint16_t adc_read(uint8_t channel);
static uint8_t i2c_read(uint8_t address, uint8_t reg, uint8_t *data,
                        uint8_t length);
static uint8_t i2c_write(uint8_t address, uint8_t reg, uint8_t value);
static void detect_node_type(void);
static void sample_node(void);
static void update_handle_button_stat(void);
static void update_leds(void);
static void process_frame(void);
static void uart_write(const uint8_t *data, uint8_t length);

int main(void) {
  SystemCoreClockUpdate();
  Delay_Init();
  gpio_init();
  startup_led_test();
  uart_init();
  i2c_init();
  adc_init();
  cctrl_bus_parser_reset(&parser);
  Delay_Ms(2);
  detect_node_type();
  sample_node();

  for (;;) {
    while (rx_head != rx_tail) {
      const uint8_t byte = rx_ring[rx_head];
      rx_head = (uint8_t)((rx_head + 1u) & 0x7Fu);
      const int result = cctrl_bus_parser_push(&parser, byte);
      if (result == 1) {
        process_frame();
        cctrl_bus_parser_reset(&parser);
      } else if (result == -1) {
        crc_error_until_tick = ticks + CRC_LED_TICKS;
        cctrl_bus_parser_reset(&parser);
      } else if (result == -2) {
        cctrl_bus_parser_reset(&parser);
      }
    }
    if ((ticks % SAMPLE_TICKS) == 0u) sample_node();
    update_leds();
    ++ticks;
    Delay_Us(LOOP_US);
  }
}

static void gpio_init(void) {
  GPIO_InitTypeDef gpio = {0};
  RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_GPIOD, ENABLE);
  gpio.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
  gpio.GPIO_Speed = GPIO_Speed_30MHz;
  gpio.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &gpio);
  GPIO_ResetBits(GPIOC, GPIO_Pin_3 | GPIO_Pin_4);

  gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  gpio.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &gpio);

  /* The fitted 5-pin joystick has VCC/GND reversed: S3 closes to 3V3. */
  gpio.GPIO_Pin = GPIO_Pin_5;
  gpio.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOC, &gpio);
}

static void startup_led_test(void) {
  for (uint8_t i = 0; i < 2; ++i) {
    GPIO_SetBits(GPIOC, GPIO_Pin_3 | GPIO_Pin_4);
    Delay_Ms(140);
    GPIO_ResetBits(GPIOC, GPIO_Pin_3 | GPIO_Pin_4);
    Delay_Ms(140);
  }
}

static void uart_init(void) {
  GPIO_InitTypeDef gpio = {0};
  USART_InitTypeDef uart = {0};
  NVIC_InitTypeDef irq = {0};
  RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_USART1, ENABLE);
  gpio.GPIO_Pin = GPIO_Pin_5;
  gpio.GPIO_Speed = GPIO_Speed_30MHz;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &gpio);
  gpio.GPIO_Pin = GPIO_Pin_6;
  gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &gpio);
  uart.USART_BaudRate = 250000;
  uart.USART_WordLength = USART_WordLength_8b;
  uart.USART_StopBits = USART_StopBits_1;
  uart.USART_Parity = USART_Parity_No;
  uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &uart);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  irq.NVIC_IRQChannel = USART1_IRQn;
  irq.NVIC_IRQChannelPreemptionPriority = 1;
  irq.NVIC_IRQChannelSubPriority = 0;
  irq.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&irq);
  USART_Cmd(USART1, ENABLE);
}

static void i2c_init(void) {
  GPIO_InitTypeDef gpio = {0};
  I2C_InitTypeDef i2c = {0};
  RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);
  RCC_PB1PeriphClockCmd(RCC_PB1Periph_I2C1, ENABLE);
  gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
  gpio.GPIO_Speed = GPIO_Speed_30MHz;
  gpio.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOC, &gpio);
  i2c.I2C_ClockSpeed = 400000;
  i2c.I2C_Mode = I2C_Mode_I2C;
  i2c.I2C_DutyCycle = I2C_DutyCycle_2;
  i2c.I2C_OwnAddress1 = 0;
  i2c.I2C_Ack = I2C_Ack_Enable;
  i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &i2c);
  I2C_Cmd(I2C1, ENABLE);
}

static void adc_init(void) {
  GPIO_InitTypeDef gpio = {0};
  ADC_InitTypeDef adc = {0};
  RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_ADC1, ENABLE);
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  gpio.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOD, &gpio);
  ADC_DeInit(ADC1);
  adc.ADC_Mode = ADC_Mode_Independent;
  adc.ADC_ScanConvMode = DISABLE;
  adc.ADC_ContinuousConvMode = DISABLE;
  adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  adc.ADC_DataAlign = ADC_DataAlign_Right;
  adc.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &adc);
  ADC_Cmd(ADC1, ENABLE);
  ADC_BufferCmd(ADC1, ENABLE);
}

static uint16_t adc_read(uint8_t channel) {
  ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_CyclesMode7);
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {}
  return ADC_GetConversionValue(ADC1) & 0x0FFFu;
}

static uint8_t i2c_wait(uint32_t event) {
  uint32_t timeout = 30000;
  while (I2C_CheckEvent(I2C1, event) != READY)
    if (--timeout == 0) return 0;
  return 1;
}

static uint8_t i2c_read(uint8_t address, uint8_t reg, uint8_t *data,
                        uint8_t length) {
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  I2C_GenerateSTART(I2C1, ENABLE);
  if (!i2c_wait(I2C_EVENT_MASTER_MODE_SELECT)) goto fail;
  I2C_Send7bitAddress(I2C1, address << 1, I2C_Direction_Transmitter);
  if (!i2c_wait(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto fail;
  I2C_SendData(I2C1, reg);
  if (!i2c_wait(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto fail;
  I2C_GenerateSTART(I2C1, ENABLE);
  if (!i2c_wait(I2C_EVENT_MASTER_MODE_SELECT)) goto fail;
  I2C_Send7bitAddress(I2C1, address << 1, I2C_Direction_Receiver);
  if (!i2c_wait(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto fail;
  for (uint8_t i = 0; i < length; ++i) {
    if (i + 1u == length) {
      I2C_AcknowledgeConfig(I2C1, DISABLE);
      I2C_GenerateSTOP(I2C1, ENABLE);
    }
    if (!i2c_wait(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto fail;
    data[i] = I2C_ReceiveData(I2C1);
  }
  I2C_AcknowledgeConfig(I2C1, ENABLE);
  return 1;
fail:
  I2C_GenerateSTOP(I2C1, ENABLE);
  I2C_SoftwareResetCmd(I2C1, ENABLE);
  I2C_SoftwareResetCmd(I2C1, DISABLE);
  i2c_init();
  return 0;
}

static uint8_t i2c_write(uint8_t address, uint8_t reg, uint8_t value) {
  I2C_GenerateSTART(I2C1, ENABLE);
  if (!i2c_wait(I2C_EVENT_MASTER_MODE_SELECT)) goto fail;
  I2C_Send7bitAddress(I2C1, address << 1, I2C_Direction_Transmitter);
  if (!i2c_wait(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto fail;
  I2C_SendData(I2C1, reg);
  if (!i2c_wait(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto fail;
  I2C_SendData(I2C1, value);
  if (!i2c_wait(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto fail;
  I2C_GenerateSTOP(I2C1, ENABLE);
  return 1;
fail:
  I2C_GenerateSTOP(I2C1, ENABLE);
  I2C_SoftwareResetCmd(I2C1, ENABLE);
  I2C_SoftwareResetCmd(I2C1, DISABLE);
  i2c_init();
  return 0;
}

static void detect_node_type(void) {
  uint8_t manufacturer[2];
  for (uint8_t i = 0; i < sizeof(tmag_addresses); ++i) {
    if (i2c_read(tmag_addresses[i], TMAG_MANUFACTURER_LSB, manufacturer, 2) &&
        manufacturer[0] == 0x49u && manufacturer[1] == 0x54u) {
      node_type = CCTRL_NODE_HANDLE;
      tmag_address = tmag_addresses[i];
      i2c_write(tmag_address, TMAG_SENSOR_CONFIG_1, 0x70u);
      i2c_write(tmag_address, TMAG_DEVICE_CONFIG_2, 0x02u);
      return;
    }
  }
  node_type = CCTRL_NODE_ENCODER;
}

static void sample_node(void) {
  if (node_type == CCTRL_NODE_HANDLE) {
    uint8_t xyz[6];
    uint8_t flags = 0;
    /* Reversed joystick supply reverses both potentiometer axes. */
    joystick_x = 4095u - adc_read(ADC_Channel_4); /* ADC1 net: PD3 */
    joystick_y = 4095u - adc_read(ADC_Channel_3); /* ADC2 net: PD2 */
    handle_buttons = 0;
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == Bit_RESET) handle_buttons |= 0x01u;
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6) == Bit_RESET) handle_buttons |= 0x02u;
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5) == Bit_SET) handle_buttons |= 0x04u;
    /* S4 (trigger at 80%) stays clear until the magnetic model is calibrated. */
    update_handle_button_stat();
    if (i2c_read(tmag_address, TMAG_X_RESULT, xyz, sizeof(xyz))) {
      flags = CCTRL_HANDLE_I2C_OK | CCTRL_HANDLE_DATA_READY;
      magnetic_x = (int16_t)(((uint16_t)xyz[0] << 8) | xyz[1]);
      magnetic_y = (int16_t)(((uint16_t)xyz[2] << 8) | xyz[3]);
      magnetic_z = (int16_t)(((uint16_t)xyz[4] << 8) | xyz[5]);
      last_good_tick = ticks;
    }
    if (ticks - last_good_tick > (50000u / LOOP_US))
      flags |= CCTRL_HANDLE_DATA_STALE;
    handle_status = flags;
    return;
  }

  uint8_t status = 0;
  uint8_t angle[2] = {0, 0};
  uint8_t flags = 0;
  if (i2c_read(AS5600_ADDR, AS5600_STATUS, &status, 1) &&
      i2c_read(AS5600_ADDR, AS5600_RAW_ANGLE, angle, 2)) {
    flags |= CCTRL_ENC_I2C_OK;
    if (status & 0x20u) flags |= CCTRL_ENC_MAGNET_DETECTED;
    if (status & 0x10u) flags |= CCTRL_ENC_MAGNET_WEAK;
    if (status & 0x08u) flags |= CCTRL_ENC_MAGNET_STRONG;
    if (status & 0x20u) {
      raw_angle = (uint16_t)(((uint16_t)(angle[0] & 0x0Fu) << 8) | angle[1]);
      last_good_tick = ticks;
    }
  }
  if (ticks - last_good_tick > (50000u / LOOP_US)) flags |= CCTRL_ENC_DATA_STALE;
  encoder_status = flags;
}

static void update_handle_button_stat(void) {
  if (handle_buttons != 0) {
    if (!handle_buttons_down) {
      handle_button_stat_active = 1;
      handle_button_stat_until_tick = ticks + BUTTON_STAT_MIN_TICKS;
    }
    handle_buttons_down = 1;
  } else {
    handle_buttons_down = 0;
    if (handle_button_stat_active &&
        (int32_t)(ticks - handle_button_stat_until_tick) >= 0)
      handle_button_stat_active = 0;
  }
}

static uint8_t pulse_train(uint32_t now_ms, uint8_t count, uint32_t cycle_ms) {
  const uint32_t phase = now_ms % cycle_ms;
  return phase < (uint32_t)count * 240u && (phase % 240u) < 120u;
}

static void update_leds(void) {
  const uint32_t now_ms = ticks / TICKS_PER_MS;
  if (node_type == CCTRL_NODE_HANDLE && handle_button_stat_active) {
    GPIO_ResetBits(GPIOC, GPIO_Pin_3);
    GPIO_SetBits(GPIOC, GPIO_Pin_4);
    return;
  }
  if (node_type == CCTRL_NODE_HANDLE && now_ms < 960u) {
    const uint8_t slot = (uint8_t)(now_ms / 120u);
    GPIO_WriteBit(GPIOC, GPIO_Pin_3,
                  (slot == 0u || slot == 4u) ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(GPIOC, GPIO_Pin_4,
                  (slot == 2u || slot == 6u) ? Bit_SET : Bit_RESET);
    return;
  }
  uint8_t err_count = 0;
  if ((int32_t)(crc_error_until_tick - ticks) > 0)
    err_count = 4;
  else if (node_type == CCTRL_NODE_HANDLE) {
    if ((handle_status & CCTRL_HANDLE_I2C_OK) == 0) err_count = 5;
  } else if ((encoder_status & CCTRL_ENC_I2C_OK) == 0)
    err_count = 5;
  else if ((encoder_status & CCTRL_ENC_MAGNET_DETECTED) == 0)
    err_count = 1;
  else if (encoder_status & CCTRL_ENC_MAGNET_WEAK)
    err_count = 2;
  else if (encoder_status & CCTRL_ENC_MAGNET_STRONG)
    err_count = 3;

  GPIO_WriteBit(GPIOC, GPIO_Pin_3,
                err_count && pulse_train(now_ms, err_count, 2000u) ? Bit_SET
                                                                   : Bit_RESET);
  uint8_t stat_on = 0;
  if (err_count == 0 && enumerated)
    stat_on = pulse_train(now_ms, (uint8_t)(node_id + 1u), 5000u);
  GPIO_WriteBit(GPIOC, GPIO_Pin_4, stat_on ? Bit_SET : Bit_RESET);
}

static void process_frame(void) {
  uint8_t *frame = parser.bytes;
  const uint8_t length = (uint8_t)cctrl_bus_size(frame);
  if (frame[2] != CCTRL_BUS_VERSION) {
    uart_write(frame, length);
    return;
  }
  if (frame[3] == CCTRL_MSG_ENUM_RESET) {
    CctrlEnumData data;
    enumerated = 0;
    node_id = frame[5];
    data.fw_major = 1;
    data.fw_minor = 1;
    data.capabilities = node_type == CCTRL_NODE_HANDLE
                            ? (CAP_TMAG5273 | CAP_JOYSTICK | CAP_BUTTONS)
                            : CAP_AS5600;
    if (cctrl_bus_append_record(frame, node_id, node_type,
                                CCTRL_PLATFORM_CH32V006, &data, sizeof(data)))
      enumerated = 1;
    uart_write(frame, (uint8_t)cctrl_bus_size(frame));
    return;
  }
  if (frame[3] == CCTRL_MSG_POLL && enumerated) {
    if (node_type == CCTRL_NODE_HANDLE) {
      CctrlHandleData data = {handle_status, joystick_x, joystick_y,
                              magnetic_x, magnetic_y, magnetic_z, handle_buttons};
      cctrl_bus_append_record(frame, node_id, CCTRL_NODE_HANDLE,
                              CCTRL_PLATFORM_CH32V006, &data, sizeof(data));
    } else {
      CctrlEncoderData data = {encoder_status, raw_angle};
      cctrl_bus_append_record(frame, node_id, CCTRL_NODE_ENCODER,
                              CCTRL_PLATFORM_CH32V006, &data, sizeof(data));
    }
  }
  uart_write(frame, (uint8_t)cctrl_bus_size(frame));
}

static void uart_write(const uint8_t *data, uint8_t length) {
  for (uint8_t i = 0; i < length; ++i) {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
    USART_SendData(USART1, data[i]);
  }
}

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void NMI_Handler(void) {}
void HardFault_Handler(void) { for (;;) {} }
void USART1_IRQHandler(void) {
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    const uint8_t byte = (uint8_t)USART_ReceiveData(USART1);
    const uint8_t next = (uint8_t)((rx_tail + 1u) & 0x7Fu);
    if (next != rx_head) {
      rx_ring[rx_tail] = byte;
      rx_tail = next;
    }
  }
}
