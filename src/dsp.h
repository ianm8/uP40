/*
 * uPDCR - Direct Conversion Receiver mk III
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

#ifndef DSP_H
#define DSP_H

#include "filter.h"

namespace DSP
{
  volatile static float agc_peak = 0.0f;

  static void __not_in_flash_func(mute)(void)
  {
    // set AGC to high value so that audio is temporarily muted
    static const float mute_value = 8192.0f;
    agc_peak = mute_value;
  }

  static const int16_t __not_in_flash_func(agc)(const float in)
  {
    // limit gain to max of 40 (32db)
    static const float max_gain = 40.0f;
    // about 10dB per second
    static const float k = 0.99996f;

    const float magnitude = fabsf(in);
    if (magnitude > agc_peak)
    {
      agc_peak = magnitude;
    }
    else
    {
      agc_peak *= k;
    }

    // trap issues with low values
    if (agc_peak<1.0f) return (int16_t)(in * max_gain);

    // set maximum gain possible for 12 bit DAC
    const float m = 2047.0f / agc_peak;
    return (int16_t)(in * fminf(m, max_gain));
  }

  static const uint32_t __not_in_flash_func(map)(const uint32_t x,const uint32_t in_min, const uint32_t in_max,const uint32_t out_min, const float out_max)
  {
    // unsigned map
    if (x<in_min)
    {
      return out_min;
    }
    if (x>in_max)
    {
      return out_max;
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  static const uint32_t __not_in_flash_func(smeter)(void)
  {
    // assume S9 = 86 in 14 bits (35mv PP)
    volatile static uint32_t s = 0;
    volatile static uint32_t agc_update = 0;
    static const float S0_sig = 30.0f;
    static const float S9_sig = 120.0f;
    static const uint32_t sig_min = (uint32_t)(log10f(S0_sig) * 1024.0f);
    static const uint32_t sig_max = (uint32_t)(log10f(S9_sig) * 1024.0f);
    static const uint32_t led_min = 0ul;
    static const uint32_t led_max = 255ul;
    const uint32_t now = millis();
    if (now>agc_update)
    {
      agc_update = now + 20ul;
      s = 0;
      const float peak = agc_peak;
      if (peak>1.0f)
      {
        const uint32_t log_peak = (uint32_t)(log10f(peak) * 1024.0f);
        s = map(log_peak,sig_min,sig_max,led_min,led_max);
      }
    }
    return s;
  }

  static const int16_t __not_in_flash_func(process_ssb)(const float in_i,const float in_q)
  {
    const float ii = FILTER::dc1(in_i);
    const float qq = FILTER::dc2(in_q);

    // phase shift IQ +/- 45
    const float p45 = FILTER::ap1(ii);
    const float n45 = FILTER::ap2(qq);

    // reject image
    const float ssb = p45 - n45;

    // LPF
    const float audio = FILTER::lpf_2400(ssb);

    // AGC returns 12 bit value
    return agc(audio * 8192.0f);
  }

  static const int16_t __not_in_flash_func(process_cw)(const float in_i,const float in_q)
  {
    const float ii = FILTER::dc1(in_i);
    const float qq = FILTER::dc2(in_q);

    // phase shift IQ +/- 45
    const float p45 = FILTER::ap1(ii);
    const float n45 = FILTER::ap2(qq);

    // reject image
    const float ssb = p45 - n45;

    // BPF for CW
    const float audio = FILTER::bpf_700(ssb);

    // AGC returns 12 bit value
    return agc(audio * 8192.0f);
  }

  static const uint32_t __not_in_flash_func(get_mic_peak_level)(const int16_t mic_in)
  {
    static const uint32_t MIC_LEVEL_DECAY_RATE = 50ul;
    static const uint32_t MIC_LEVEL_HANG_TIME = 500ul;
    static uint32_t mic_peak_level = 0;
    static uint32_t mic_level_update = 0;
    static uint32_t mic_hangtime_update = 0;
    const uint32_t now = millis();
    const uint32_t mic_level = abs(mic_in);
    if (mic_level>mic_peak_level)
    {
      mic_peak_level = mic_level;
      mic_level_update = now + MIC_LEVEL_DECAY_RATE;
      mic_hangtime_update = now + MIC_LEVEL_HANG_TIME;
    }
    else
    {
      if (now>mic_hangtime_update)
      {
        if (now>mic_level_update)
        {
          if (mic_peak_level) mic_peak_level--;
          mic_level_update = now + MIC_LEVEL_DECAY_RATE;
        }
      }
    }
    return mic_peak_level;
  }

  const void __not_in_flash_func(process_mic)(const int16_t s,int16_t &out_i,int16_t &out_q)
  {
    static const float mic_gain = 4.0f;
    // input is 12 bits
    // convert to float
    // remove Mic DC
    // 2400 LPF 
    // phase shift I
    // phase shift Q
    // first order CESSB
    // convert to int
    // output is 10 bits
    const float ac_sig = FILTER::dcf(((float)s)*(1.0f/2048.0f));
    const float mic_sig = FILTER::lpf_2400f_tx(ac_sig * mic_gain);
    float ii = FILTER::ap1(mic_sig);
    float qq = FILTER::ap2(mic_sig);
    const float mag_raw = sqrtf(ii*ii + qq*qq);
    const float mag_max = fmaxf(mag_raw, 1.0f);
    ii = FILTER::lpf_2400if_tx(ii / mag_max);
    qq = FILTER::lpf_2400qf_tx(qq / mag_max);
    out_i = (int16_t)(ii * 512.0f);
    out_q = (int16_t)(qq * 512.0f);
  }
}

#endif