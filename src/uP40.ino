/*
 * uP40 - 40M Phasing Transceiver
 *
 * Copyright (C) 2025 Ian Mitchell VK7IAN
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
 *
 * Version 0.9 2025-04-07 initial release
 * Version 0.9 2025-05-06 fix tuning direction
 * Version 0.9 2025-05-06 fix TX sideband
 * Version 0.9 2025-05-06 fix auto sideband
 * Version 0.9 2025-05-10 mute changing mode
 * Version 0.9 2025-05-10 AGC TX/RX transition
 * Version 0.9 2025-05-10 mute Mic when RX
 * Version 0.9 2025-05-14 long press, change mode
 * Version 0.9 2025-05-14 announce mode change
 * Version 0.9 2025-05-14 announce step change
 * Version 0.9 2025-05-14 reduce CW signal strength
 * Version 0.9 2025-05-14 fix CW receive offset
 * Version 0.9 2025-05-14 fix CW paddle processing
 * Version 0.9 2025-05-16 fix step change button bounce
 * Version 1.0 2025-05-16 release version 1.0
 * Version 1.1 2025-05-24 fix mic level indication
 * Version 1.2 2025-05-25 remove unnecessary band code
 * Version 1.3 2025-05-25 adjust mic gain
 * Version 1.4 2025-05-27 direct CW (no phase errors)
 *
 * TODO:
 *
 * Build:
 *  Board: Pi Pico 2
 *  Flash 4MB (no FS)
 *  CPU Speed: 240Mhz
 *  Optimize: -O2
 *  USB Stack: No USB
 */

#include <Wire.h>
#include "si5351.h"
#include "Rotary.h"
#include "filter.h"
#include "dsp.h"
#include "cw.h"
#include "vfa.h"
#include "announce.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/vreg.h"

#define PIN_UNUSED0   0u // free pin
#define PIN_UNUSED1   1u // free pin
#define PIN_PTT       2u // Mic PTT (active low) and CW Paddle A
#define PIN_UNUSED3   3u // free pin
#define PIN_SDA       4u // I2C SDA
#define PIN_SCL       5u // I2C SCL
#define PIN_TX000     6u // TX PWM
#define PIN_TX180     7u // TX PWM
#define PIN_TX090     8u // TX PWM
#define PIN_TX270     9u // TX PWM
#define PIN_TXN      10u // Enable TX mixer (active low)
#define PIN_UNUSED11 11u // free pin
#define PIN_UNUSED12 12u // free pin
#define PIN_1LED     13u // PWM LED signal level
#define PIN_PADB     14u // CW Paddle B
#define PIN_VOL      15u // PWM volume control
#define PIN_AUDL     16u // PWM audio low bits
#define PIN_AUDH     17u // PWM audio high bits
#define PIN_ENCA     18u // Rotary encoder
#define PIN_ENCB     19u // Rotary encoder
#define PIN_ENCBUT   20u // Rotary encoder button
#define PIN_TXBIAS   21u // enable TX bias
#define PIN_RXN      22u // Enable RX mixer (active low)
#define PIN_REG      23u // Pico regulator
#define PIN_SIG_I    26u // GPIO 26
#define PIN_SIG_Q    27u // GPIO 27
#define PIN_MIC      28u // analog MIC

#define CW_TIMEOUT         800u
#define CW_SIDETONE        700u
#define CW_TIME            60u
#define DEFAULT_VOLUME     120u
#define DEFAULT_FREQUENCY  7100000ul
#define DEFAULT_STEP       1000ul
#define DEFAULT_MODE       MODE_LSB
#define DEFAULT_CW_MODE    CW_PADDLE
#define DEFAULT_AUTO_MODE  false
#define VOLUME_STEP        5u
#define LONG_PRESS_TIME    1000u
#define MIN_FREQUENCY      7000000ul
#define MAX_FREQUENCY      7300000ul
#define MIN_VOL            80ul
#define MAX_VOL            255ul
#define TCXO_FREQ          27000000ul
#define VFA_DELAY          2000ul
#define QUADRATURE_DIVISOR 88ul
#define MUTE               0u
#define CW_STRAIGHT        0u
#define CW_PADDLE          1u

#define TEST_5351         0
#define DEBUG_LED         0

#define SIG_MUX 0u
#if PIN_MIC == 26U
#define MIC_MUX 0U
#elif PIN_MIC == 27U
#define MIC_MUX 1U
#elif PIN_MIC == 28U
#define MIC_MUX 2U
#elif PIN_MIC == 29U
#define MIC_MUX 3U
#endif

enum radio_mode_t
{
  MODE_LSB,
  MODE_USB,
  MODE_CWL,
  MODE_CWU
};

volatile static struct
{
  uint32_t frequency;
  uint32_t tuning_step;
  uint32_t volume;
  radio_mode_t mode;
  uint8_t cw_mode;
  bool auto_mode;
  bool tx_enable;
  bool keydown;
}
radio =
{
  DEFAULT_FREQUENCY,
  DEFAULT_STEP,
  DEFAULT_VOLUME,
  DEFAULT_MODE,
  DEFAULT_CW_MODE,
  DEFAULT_AUTO_MODE,
  false,
  false
};

Si5351 si5351;
Rotary r = Rotary(PIN_ENCB,PIN_ENCA);

volatile static uint32_t mic_peak_level = 0;
volatile static uint32_t audio_pwm = 0;
volatile static uint32_t tx_i_pwm = 0;
volatile static uint32_t tx_q_pwm = 0;
volatile static int32_t dac_h = 0;
volatile static int32_t dac_l = 0;
volatile static int32_t dac_value_i_p = 0;
volatile static int32_t dac_value_i_n = 0;
volatile static int32_t dac_value_q_p = 0;
volatile static int32_t dac_value_q_n = 0;
volatile static int16_t adc_value = 0;
volatile static float adc_value_i = 0;
volatile static float adc_value_q = 0;
volatile static bool adc_value_ready = false;
volatile static bool setup_complete = false;
volatile static bool dit_latched = false;
volatile static bool dah_latched = false;

void setup()
{
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  const uint32_t clksys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
  // frequency_count_khz() isn't accurate!
  if (clksys < (240000ul - 5ul) || clksys > (240000ul + 5ul))
  {
    // trap the wrong system clock
    pinMode(LED_BUILTIN,OUTPUT);
    for (;;)
    {
      digitalWrite(LED_BUILTIN,HIGH);
      delay(30);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
    }
  }
#if defined DEBUG_LED && DEBUG_LED==1
  for (int i=0;i<5;i++)
  {
    delay(250);
    digitalWrite(LED_BUILTIN,HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN,LOW);
  }
  delay(5000);
#endif
  analogWriteResolution(256);
  analogWriteFreq(500000ul);

  pinMode(PIN_TXBIAS,OUTPUT);
  pinMode(PIN_TXN,OUTPUT);
  pinMode(PIN_RXN,OUTPUT);
  pinMode(PIN_PTT,INPUT);
  pinMode(PIN_PADB,INPUT);
  pinMode(PIN_UNUSED1,INPUT_PULLUP);
  pinMode(PIN_UNUSED3,INPUT_PULLUP);
  pinMode(PIN_UNUSED11,INPUT_PULLUP);
  pinMode(PIN_UNUSED12,INPUT_PULLUP);
  pinMode(PIN_REG,OUTPUT);
  pinMode(PIN_ENCBUT,INPUT_PULLUP);
  pinMode(PIN_ENCA,INPUT_PULLUP);
  pinMode(PIN_ENCB,INPUT_PULLUP);
  analogWrite(PIN_1LED,0);
  analogWrite(PIN_VOL,0);
  digitalWrite(PIN_REG,HIGH);
  digitalWrite(PIN_TXBIAS,LOW);
  digitalWrite(PIN_TXN,HIGH);
  digitalWrite(PIN_RXN,LOW);
  digitalWrite(LED_BUILTIN,LOW);

  // set TX pin function to PWM
  gpio_set_function(PIN_TX000,GPIO_FUNC_PWM); // 6  PWM
  gpio_set_function(PIN_TX180,GPIO_FUNC_PWM); // 7  PWM
  gpio_set_function(PIN_TX090,GPIO_FUNC_PWM); // 8  PWM
  gpio_set_function(PIN_TX270,GPIO_FUNC_PWM); // 9  PWM

  // get PWM slice connected to each pin pair
  tx_i_pwm = pwm_gpio_to_slice_num(PIN_TX000);
  tx_q_pwm = pwm_gpio_to_slice_num(PIN_TX090);
  
  // set period of 1024 cycles
  pwm_set_wrap(tx_i_pwm,1023);
  pwm_set_wrap(tx_q_pwm,1023);
  
  // initialise to zero (low)
  pwm_set_both_levels(tx_i_pwm,0,0);
  pwm_set_both_levels(tx_q_pwm,0,0);

  // set each PWM running
  pwm_set_enabled(tx_i_pwm,true);
  pwm_set_enabled(tx_q_pwm,true);

  // set up audio out PWM
  gpio_set_function(PIN_AUDH,GPIO_FUNC_PWM);
  gpio_set_function(PIN_AUDL,GPIO_FUNC_PWM);
  audio_pwm = pwm_gpio_to_slice_num(PIN_AUDL);
  pwm_set_wrap(audio_pwm,63u); // 240,000,000 / 64 = 3,750,000
  pwm_set_both_levels(audio_pwm,0u,31u);
  pwm_set_enabled(audio_pwm,true);

  // set up MS5351M
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  const bool si5351_found = si5351.init(SI5351_CRYSTAL_LOAD_0PF,TCXO_FREQ,0);
  if (!si5351_found)
  {
    for (;;)
    {
      digitalWrite(LED_BUILTIN,HIGH);
      delay(25);
      digitalWrite(LED_BUILTIN,LOW);
      delay(250);
    }
  }
  si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1,SI5351_DRIVE_8MA);
  const uint64_t f = radio.frequency*SI5351_FREQ_MULT;
  const uint64_t p = radio.frequency*QUADRATURE_DIVISOR*SI5351_FREQ_MULT;
  si5351.set_freq_manual(f,p,SI5351_CLK0);
  si5351.set_freq_manual(f,p,SI5351_CLK1);
  si5351.set_phase(SI5351_CLK0,0);
  si5351.set_phase(SI5351_CLK1,QUADRATURE_DIVISOR);
  si5351.pll_reset(SI5351_PLLA);

#if defined TEST_5351 && TEST_5351==1
  for (;;)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
  }
#endif

  // if button pressed at startup
  // then turn on auto mode
  if (digitalRead(PIN_ENCBUT)==LOW)
  {
    radio.auto_mode = true;
    delay(50);
    while (digitalRead(PIN_ENCBUT)==LOW)
    {
      delay(50);
    }
    delay(50);
  }

  // if PTT pressed at startup
  // then enable CW straight key
  if (digitalRead(PIN_PTT)==LOW)
  {
    radio.cw_mode = CW_STRAIGHT;
    delay(50);
    while (digitalRead(PIN_PTT)==LOW)
    {
      delay(50);
    }
    delay(50);
  }

  r.begin();
  init_adc();
  analogWrite(PIN_VOL,radio.volume);
  setup_complete = true;
}

void setup1()
{
  // wait for setup() to complete
  while (!setup_complete)
  {
    // setup() not done yet
    tight_loop_contents();
  }
#if defined DEBUG_LED && DEBUG_LED==1
  pinMode(LED_BUILTIN,OUTPUT);
  for (int i=0;i<5;i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
  }
#endif
}

void __not_in_flash_func(adc_interrupt_handler)(void)
{
  // note, FIFO depth of 8 caused random
  // channel swapping causing the radio
  // to switch between LSB and USB
  // this is probably due to buffer overflow
  // in the FIFO caused by background interrupts
  volatile static uint32_t counter = 0;
  volatile static uint32_t adc_raw = 0;
  volatile static float adc_raw_i = 0;
  volatile static float adc_raw_q = 0;
  if (adc_fifo_get_level()<4u)
  {
    return;
  }
  volatile const uint16_t adc0 = adc_fifo_get();
  volatile const uint16_t adc1 = adc_fifo_get();
  volatile const uint16_t adc2 = adc_fifo_get();
  volatile const uint16_t adc3 = adc_fifo_get();
  if (radio.tx_enable)
  {
    adc_raw += adc0;
    adc_raw += adc1;
    adc_raw += adc2;
    adc_raw += adc3;
    if (counter==4)
    {
      pwm_set_both_levels(audio_pwm,dac_l,dac_h);
      pwm_set_both_levels(tx_i_pwm,dac_value_i_p,dac_value_i_n);
      pwm_set_both_levels(tx_q_pwm,dac_value_q_p,dac_value_q_n);
      adc_value = ((int16_t)(adc_raw>>4))-2048;
      adc_value_ready = true;
      adc_raw = 0;
      counter = 0;
    }
  }
  else
  {
    adc_raw_i += FILTER::ma4fi(adc0);
    adc_raw_q += FILTER::ma4fq(adc1);
    adc_raw_i += FILTER::ma4fi(adc2);
    adc_raw_q += FILTER::ma4fq(adc3);
    if (counter==4)
    {
      pwm_set_both_levels(audio_pwm,dac_l,dac_h);
      // 8 times oversampling per channel
      adc_value_i = adc_raw_i / 8.0f;
      adc_value_q = adc_raw_q / 8.0f;
      adc_value_ready = true;
      adc_raw_i = 0;
      adc_raw_q = 0;
      counter = 0;
    }
  }
  counter++;
}

void init_adc(void)
{
  pinMode(PIN_MIC,OUTPUT);
  digitalWrite(PIN_MIC,LOW);
  adc_init();
  adc_gpio_init(PIN_SIG_I);
  adc_gpio_init(PIN_SIG_Q);
  adc_select_input(SIG_MUX);
  adc_set_round_robin(0b00000011);
  adc_fifo_setup(true, false, 4, false, false);
  adc_fifo_drain();
  adc_irq_set_enabled(true);
  irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_interrupt_handler);
  irq_set_priority(ADC_IRQ_FIFO, PICO_HIGHEST_IRQ_PRIORITY);
  irq_set_enabled(ADC_IRQ_FIFO, true);
  adc_run(true);
}

void __not_in_flash_func(reset_adc_rx)(void)
{
  pinMode(PIN_MIC,OUTPUT);
  digitalWrite(PIN_MIC,LOW);
  irq_set_enabled(ADC_IRQ_FIFO, false);
  adc_run(false);
  adc_fifo_drain();
  adc_set_round_robin(0b00000011);
  adc_select_input(SIG_MUX);
  irq_set_enabled(ADC_IRQ_FIFO, true);
  adc_run(true);
}

void __not_in_flash_func(reset_adc_tx)(void)
{
  irq_set_enabled(ADC_IRQ_FIFO, false);
  adc_run(false);
  adc_fifo_drain();
  adc_set_round_robin(0b00000000);
  adc_gpio_init(PIN_MIC);
  adc_select_input(MIC_MUX);
  irq_set_enabled(ADC_IRQ_FIFO, true);
  adc_run(true);
}

void __not_in_flash_func(loop)(void)
{
  // run DSP on core 0
  static bool tx = false;
  if (tx)
  {
    // TX, check if changed to RX
    if (radio.tx_enable)
    {
      if (adc_value_ready)
      {
        adc_value_ready = false;
        int16_t tx_i = 0;
        int16_t tx_q = 0;
        switch (radio.mode)
        {
          case MODE_LSB: DSP::process_mic(adc_value,tx_i,tx_q);   break;
          case MODE_USB: DSP::process_mic(adc_value,tx_q,tx_i);   break;
          case MODE_CWL: CW::process_cw(radio.keydown,tx_i,tx_q); break;
          case MODE_CWU: CW::process_cw(radio.keydown,tx_q,tx_i); break;
        }
        tx_i = constrain(tx_i,-512,+511);
        tx_q = constrain(tx_q,-512,+511);
        dac_value_i_p = 512+tx_i;
        dac_value_i_n = 511-tx_i;
        dac_value_q_p = 512+tx_q;
        dac_value_q_n = 511-tx_q;
        if (radio.mode==MODE_LSB || radio.mode==MODE_USB)
        {
          mic_peak_level = DSP::get_mic_peak_level(adc_value);
        }
        else if (radio.mode==MODE_CWL || radio.mode==MODE_CWU)
        {
          // generate the sidetone
          int32_t dac_audio = CW::sidetone(radio.keydown);
          dac_audio = constrain(dac_audio,-2048l,+2047l);
          dac_audio += 2048l;
          dac_h = dac_audio >> 6;
          dac_l = dac_audio & 0x3f;
        }
      }
    }
    else
    {
      // switched to RX
      reset_adc_rx();
      // set TX output to zero in receive mode
      dac_value_i_p = 0;
      dac_value_i_n = 0;
      dac_value_q_p = 0;
      dac_value_q_n = 0;
      tx = false;
    }
  }
  else
  {
    // RX, check if changed to TX
    if (radio.tx_enable)
    {
      // switch to TX
      reset_adc_tx();
      tx = true;
    }
    else
    {
      if (adc_value_ready)
      {
        adc_value_ready = false;
        int32_t rx_value = 0;
        switch (radio.mode)
        {
          case MODE_LSB: rx_value = (int32_t)DSP::process_ssb(adc_value_i,adc_value_q); break;
          case MODE_USB: rx_value = (int32_t)DSP::process_ssb(adc_value_q,adc_value_i); break;
          case MODE_CWL: rx_value = (int32_t)DSP::process_cw(adc_value_i,adc_value_q);  break;
          case MODE_CWU: rx_value = (int32_t)DSP::process_cw(adc_value_q,adc_value_i);  break;
        }
        if (VFA::active)
        {
          rx_value = (rx_value>>4) + VFA::announce();
        }
        else if (ANNOUNCE::active)
        {
          rx_value = (rx_value>>4) + ANNOUNCE::announce();
        }
        const int32_t dac_audio = constrain(rx_value,-2048l,+2047l)+2048l;
        dac_h = dac_audio >> 6;
        dac_l = dac_audio & 0x3f;
      }
    }
  }
}

static void process_ssb_tx(void)
{
  // 1. mute the receiver
  // 2. set TX/RX relay to TX
  // 3. enable MIC (DSP)
  // 4. enable TX bias

  // indicate PTT pressed
  digitalWrite(LED_BUILTIN,HIGH);

  // mute the receiver
  analogWrite(PIN_VOL,MUTE);
  delay(10);

  // disable QSD
  digitalWrite(PIN_RXN,HIGH);

  // enable MIC processing
  radio.tx_enable = true;
  delay(10);

  // enable QSE and TX bias
  digitalWrite(PIN_TXN,LOW);
  digitalWrite(PIN_TXBIAS,HIGH);
  delay(50);

  // wait for PTT release
  uint32_t tx_LED_update = 0;
  while (digitalRead(PIN_PTT)==LOW)
  {
    const uint32_t now = millis();
    if (now>tx_LED_update)
    {
      tx_LED_update = now + 50ul;
      analogWrite(PIN_1LED,mic_peak_level>>2);
    }
  }
}

static void cw_dit_delay(const uint32_t ms,const uint32_t level)
{
  // delay here for dit and check for dah
  const uint32_t delay_time = millis()+ms;
  analogWrite(PIN_1LED,level);
  while (delay_time>millis())
  {
    // check for dah
    if (digitalRead(PIN_PADB)==LOW)
    {
      dah_latched = true;
    }
  }
}

static void cw_dah_delay(const uint32_t ms,const uint32_t level)
{
  // delay here for dah and check for dit
  const uint32_t delay_time = millis()+ms;
  analogWrite(PIN_1LED,level);
  while (delay_time>millis())
  {
    // check for dit
    if (digitalRead(PIN_PTT)==LOW)
    {
      dit_latched = true;
    }
  }
}

static void process_key(void)
{
  // disable QSD
  digitalWrite(PIN_RXN,HIGH);
  delay(10);

  // enable TX processing
  radio.keydown = false;
  radio.tx_enable = true;

  // enable QSE and TX bias
  digitalWrite(PIN_TXN,LOW);
  digitalWrite(PIN_TXBIAS,HIGH);
  delay(10);

  // stay here until timeout after key up (PTT released)
  uint32_t cw_timeout = millis() + CW_TIMEOUT;
  if (radio.cw_mode==CW_STRAIGHT)
  {
    for (;;)
    {
      if (digitalRead(PIN_PTT)==LOW)
      {
        // indicate PTT pressed
        digitalWrite(LED_BUILTIN,HIGH);
        radio.keydown = true;
        cw_timeout = millis() + CW_TIMEOUT;
        analogWrite(PIN_1LED,255u);
        delay(20);
      }
      else
      {
        // indicate PTT released
        digitalWrite(LED_BUILTIN,LOW);
        radio.keydown = false;
        analogWrite(PIN_1LED,0u);
        delay(20);
        if (millis()>cw_timeout)
        {
          break;
        }
      }
    }
  }
  else
  {
    // paddle
    for (;;)
    {
      if (digitalRead(PIN_PTT)==LOW || dit_latched)
      {
        // dit
        dit_latched = false;
        cw_dit_delay(CW_TIME,0u);
        radio.keydown = true;
        digitalWrite(LED_BUILTIN,HIGH);
        cw_dit_delay(CW_TIME,255u);
        radio.keydown = false;
        digitalWrite(LED_BUILTIN,LOW);
        cw_timeout = millis() + CW_TIMEOUT;
      }
      if (digitalRead(PIN_PADB)==LOW || dah_latched)
      {
        // dah
        dah_latched = false;
        cw_dah_delay(CW_TIME,0u);
        radio.keydown = true;
        digitalWrite(LED_BUILTIN,HIGH);
        cw_dah_delay(CW_TIME*3,255u);
        radio.keydown = false;
        digitalWrite(LED_BUILTIN,LOW);
        cw_timeout = millis() + CW_TIMEOUT;
      }
      if (millis()>cw_timeout)
      {
        break;
      }
    }
  }

  // mute during transition back to receive
  analogWrite(PIN_VOL,MUTE);
  delay(50);
}

void __not_in_flash_func(loop1)(void)
{
  // remember the current frequency and button state
  static uint32_t current_frequency = 0;
  static uint32_t button_start_time = 0;
  static uint32_t vfa_announce_time = 0;
  static uint32_t current_vfa_frequency = 0;
  static uint32_t new_vfa_frequency = 0;
  static bool vfa_announce = false;

  // update volume and LED smeter
  analogWrite(PIN_VOL,radio.volume);
  analogWrite(PIN_1LED,DSP::smeter());
    
  // what's the rotary encoder doing?
  const uint8_t rotary = r.process();

  volatile static enum
  {
    STATE_TUNING,
    STATE_BUTTON_PRESS,
    STATE_VOLUME,
    STATE_WAIT_RELEASE
  } state = STATE_TUNING;

  switch (state)
  {
    case STATE_TUNING:
    {
      // tuning
      switch (rotary)
      {
        case DIR_CW:
        {
          radio.frequency += radio.tuning_step - radio.frequency%radio.tuning_step;
          radio.frequency = constrain(radio.frequency,MIN_FREQUENCY,MAX_FREQUENCY);
          break;
        }
        case DIR_CCW:
        {
          const uint32_t modula = radio.frequency%radio.tuning_step;
          radio.frequency -= (modula==0)?radio.tuning_step:modula;
          radio.frequency = constrain(radio.frequency,MIN_FREQUENCY,MAX_FREQUENCY);
          break;
        }
      }
      if (digitalRead(PIN_ENCBUT)==LOW)
      {
        button_start_time = millis();
        state = STATE_BUTTON_PRESS;
      }
      break;
    }
    case STATE_BUTTON_PRESS:
    {
      const uint32_t press_time = millis()-button_start_time;
      if (press_time>LONG_PRESS_TIME)
      {
        // change mode
        DSP::mute();
        switch (radio.mode)
        {
          case MODE_LSB: radio.mode = MODE_USB; break;
          case MODE_USB: radio.mode = MODE_CWL; break;
          case MODE_CWL: radio.mode = MODE_CWU; break;
          case MODE_CWU: radio.mode = MODE_LSB; break;
        }
        ANNOUNCE::setMode(radio.mode);
        state = STATE_WAIT_RELEASE;
        break;
      }
      switch (rotary)
      {
        case DIR_CW:  radio.volume += VOLUME_STEP; state = STATE_VOLUME; break;
        case DIR_CCW: radio.volume -= VOLUME_STEP; state = STATE_VOLUME; break;
      }
      if (digitalRead(PIN_ENCBUT)==HIGH)
      {
        // change step
        if (press_time<50)
        {
          // avoid bounce
          break;
        }
        switch (radio.tuning_step)
        {
          case 1000: radio.tuning_step = 100;  break;
          case 100:  radio.tuning_step = 10;   break;
          case 10:   radio.tuning_step = 1000; break;
        }
        ANNOUNCE::setStep(radio.tuning_step);
        state = STATE_WAIT_RELEASE;
      }
      break;
    }
    case STATE_VOLUME:
    {
      switch (rotary)
      {
        case DIR_CW:  radio.volume += VOLUME_STEP; break;
        case DIR_CCW: radio.volume -= VOLUME_STEP; break;
      }
      radio.volume = constrain(radio.volume,MIN_VOL,MAX_VOL);
      if (digitalRead(PIN_ENCBUT)==HIGH)
      {
        state = STATE_WAIT_RELEASE;
      }
      break;
    }
    case STATE_WAIT_RELEASE:
    {
      if (digitalRead(PIN_ENCBUT)==HIGH)
      {
        // debounce
        button_start_time = 0;
        state = STATE_TUNING;
        delay(50);
      }
    }
  }

  // frequency changed?
  if (radio.frequency != current_frequency)
  {
    // update the frequency
    current_frequency = radio.frequency;
    new_vfa_frequency = radio.frequency / 1000ul;
    const uint32_t correct4cw = radio.mode==MODE_CWL?+CW_SIDETONE:radio.mode==MODE_CWU?-CW_SIDETONE:0u;
    const uint64_t f = (current_frequency + correct4cw) * SI5351_FREQ_MULT;
    const uint64_t p = (current_frequency + correct4cw) * QUADRATURE_DIVISOR * SI5351_FREQ_MULT;
    si5351.set_freq_manual(f,p,SI5351_CLK0);
    si5351.set_freq_manual(f,p,SI5351_CLK1);

    // reset announce time for any change
    vfa_announce_time = millis() + VFA_DELAY;

    // update the mode
    if (radio.auto_mode)
    {
      volatile radio_mode_t new_mode = MODE_LSB;
      if (radio.frequency == 7074000ul)
      {
        new_mode = MODE_USB;
      }
      else if (radio.frequency >= 7000000ul && radio.frequency <= 7060000ul)
      {
        new_mode = MODE_CWL;
      }
      if (radio.mode != new_mode)
      {
        // only update the mode if it has changed
        DSP::mute();
        ANNOUNCE::setMode(new_mode);
        radio.mode = new_mode;
        if (new_mode==MODE_LSB || new_mode==MODE_USB)
        {
          // changed to SSB
          radio.tuning_step = 1000u;
        }
        else if (radio.tuning_step==1000u)
        {
          // changed to CW
          radio.tuning_step = 100u;
        }
      }
    }
  }

  // VFA processing
  if (current_vfa_frequency != new_vfa_frequency)
  {
    current_vfa_frequency = new_vfa_frequency;
    vfa_announce = true;
  }

  // announce frequency
  if (vfa_announce)
  {
    if (millis() > vfa_announce_time)
    {
      vfa_announce = false;
      VFA::setFreq(radio.frequency);
    }
  }

  // check for PTT
  const bool b_PTT = (digitalRead(PIN_PTT)==LOW);
  const bool b_PADB = (digitalRead(PIN_PADB)==LOW);
  if (b_PTT || b_PADB)
  {
    bool back_to_receive = false;
    const float saved_agc = DSP::agc_peak;
    if (radio.mode==MODE_CWL || radio.mode==MODE_CWU)
    {
      process_key();
      back_to_receive = true;
    }
    else if (b_PTT)
    {
      process_ssb_tx();
      back_to_receive = true;
    }
    if (back_to_receive)
    {
      // back to receive
      digitalWrite(PIN_TXBIAS,LOW);
      digitalWrite(PIN_TXN,HIGH);
      radio.tx_enable = false;
      delay(10);
      digitalWrite(PIN_RXN,LOW);
      digitalWrite(LED_BUILTIN,LOW);
      delay(50);
      DSP::agc_peak = saved_agc;
    }
  }
}