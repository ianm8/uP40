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

#ifndef FILTER_H
#define FILTER_H

#define FIR_LENGTH 256
#define __mac_tap(_h) acc += (_h)*x[i++]
#define __mac_zap(_h) i++
#define MA_FILTER_LENGTH 32u
#define MA_FILTER_MASK (MA_FILTER_LENGTH-1u)

namespace FILTER
{
  static const float __not_in_flash_func(ma4fi)(const int16_t s)
  {
    // I channel
    // 4th order 32 pole moving average
    // (notch at 250,000/32 = 7812.5Hz)
    static float delay1[MA_FILTER_LENGTH] = { 0 };
    static float delay2[MA_FILTER_LENGTH] = { 0 };
    static float delay3[MA_FILTER_LENGTH] = { 0 };
    static float delay4[MA_FILTER_LENGTH] = { 0 };
    static float sum1 = 0;
    static float sum2 = 0;
    static float sum3 = 0;
    static float sum4 = 0;
    static uint8_t p = 0;
    const float v1 = (float)(s - 2048) / 2048.0f;
    sum1 = sum1 - delay1[p] + v1;
    delay1[p] = v1;
    const float v2 = sum1 / (float)MA_FILTER_LENGTH;
    sum2 = sum2 - delay2[p] + v2;
    delay2[p] = v2;
    const float v3 = sum2 / (float)MA_FILTER_LENGTH;
    sum3 = sum3 - delay3[p] + v3;
    delay3[p] = v3;
    const float v4 = sum3 / (float)MA_FILTER_LENGTH;
    sum4 = sum4 - delay4[p] + v4;
    delay4[p] = v4;
    p++;
    p &= MA_FILTER_MASK;
    return sum4 / (float)MA_FILTER_LENGTH;
  }

  static const float __not_in_flash_func(ma4fq)(const int16_t s)
  {
    // Q channel
    // 4th order 32 pole moving average
    // (notch at 250,000/32 = 7812.5Hz)
    static float delay1[MA_FILTER_LENGTH] = { 0 };
    static float delay2[MA_FILTER_LENGTH] = { 0 };
    static float delay3[MA_FILTER_LENGTH] = { 0 };
    static float delay4[MA_FILTER_LENGTH] = { 0 };
    static float sum1 = 0;
    static float sum2 = 0;
    static float sum3 = 0;
    static float sum4 = 0;
    static uint8_t p = 0;
    const float v1 = (float)(s - 2048) / 2048.0f;
    sum1 = sum1 - delay1[p] + v1;
    delay1[p] = v1;
    const float v2 = sum1 / (float)MA_FILTER_LENGTH;
    sum2 = sum2 - delay2[p] + v2;
    delay2[p] = v2;
    const float v3 = sum2 / (float)MA_FILTER_LENGTH;
    sum3 = sum3 - delay3[p] + v3;
    delay3[p] = v3;
    const float v4 = sum3 / (float)MA_FILTER_LENGTH;
    sum4 = sum4 - delay4[p] + v4;
    delay4[p] = v4;
    p++;
    p &= MA_FILTER_MASK;
    return sum4 / (float)MA_FILTER_LENGTH;
  }

  static const int16_t __not_in_flash_func(dc)(const int16_t in)
  {
    static int32_t s = 0;
    static int32_t x1 = 0;
    static int16_t y1 = 0;
    s -= x1;
    x1 = (int32_t)in << 16;
    s += x1 - ((int32_t)y1 << 8);
    return (y1 = s >> 16);
  }

  static float __not_in_flash_func(dcf)(const float in)
  {
    // single pole IIR high-pass filter
    //static const float k = 0.004f; // <100Hz
    //static const float k = 0.01f; // ~100Hz
    //static const float k = 0.1f; // ~300Hz
    static const float k = 0.05f; // ~200Hz
    static float s = 0;
    static float x1 = 0;
    static float y1 = 0;
    s -= x1;
    x1 = in;
    s += x1 - y1 * k;
    return (y1 = s);
  }

  static float __not_in_flash_func(dc1)(const float in)
  {
    // single pole IIR high-pass filter
    //static const float k = 0.004f; // <100Hz
    //static const float k = 0.01f; // ~100Hz
    //static const float k = 0.1f; // ~300Hz
    static const float k = 0.05f; // ~200Hz
    static float s = 0;
    static float x1 = 0;
    static float y1 = 0;
    s -= x1;
    x1 = in;
    s += x1 - y1 * k;
    return (y1 = s);
  }

  static float __not_in_flash_func(dc2)(const float in)
  {
    // single pole IIR high-pass filter
    //static const float k = 0.004f; // <100Hz
    //static const float k = 0.01f; // ~100Hz
    //static const float k = 0.1f; // ~300Hz
    static const float k = 0.05f; // ~200Hz
    static float s = 0;
    static float x1 = 0;
    static float y1 = 0;
    s -= x1;
    x1 = in;
    s += x1 - y1 * k;
    return (y1 = s);
  }

  static const float __not_in_flash_func(ap1)(const float s)
  {
    // all pass 84 @ 31250
    // all pass 607 @ 31250
    // all pass 2539 @ 31250
    static const float k1 = 0.98325f;
    static const float k2 = 0.88497f;
    static const float k3 = 0.59331f;
    static float x1 = 0.0f;
    static float y1 = 0.0f;
    static float x2 = 0.0f;
    static float y2 = 0.0f;
    static float x3 = 0.0f;
    static float y3 = 0.0f;
    y1 = (k1 * (s + y1)) - x1;
    x1 = s;
    y2 = (k2 * (y1 + y2)) - x2;
    x2 = y1;
    y3 = (k3 * (y2 + y3)) - x3;
    x3 = y2;
    return y3;
  }

  static const float __not_in_flash_func(ap2)(const float s)
  {
    // all pass 8628 @ 31250
    // all pass 1200 @ 31250
    // all pass 287 @ 31250
    static const float k1 = 0.07102f;
    static const float k2 = 0.78470f;
    static const float k3 = 0.94391f;
    static float x1 = 0.0f;
    static float y1 = 0.0f;
    static float x2 = 0.0f;
    static float y2 = 0.0f;
    static float x3 = 0.0f;
    static float y3 = 0.0f;
    y1 = (k1 * (s + y1)) - x1;
    x1 = s;
    y2 = (k2 * (y1 + y2)) - x2;
    x2 = y1;
    y3 = (k3 * (y2 + y3)) - x3;
    x3 = y2;
    return y3;
  }

  static const float __not_in_flash_func(lpf_2600)(const float sample)
  {
    // 31250
    // 2600 Hz
    // att: 60dB
    // 255 taps
    static float x[FIR_LENGTH] = { 0.0f };
    static uint8_t sample_index = 0;
    uint8_t i = sample_index;
    x[sample_index--] = sample;
    float acc = 0;
    __mac_tap(-0.000021f);
    __mac_tap(0.000006f);
    __mac_tap(0.000039f);
    __mac_tap(0.000067f);
    __mac_tap(0.000082f);
    __mac_tap(0.000074f);
    __mac_tap(0.000041f);
    __mac_tap(-0.000011f);
    __mac_tap(-0.000071f);
    __mac_tap(-0.000122f);
    __mac_tap(-0.000145f);
    __mac_tap(-0.000129f);
    __mac_tap(-0.000071f);
    __mac_tap(0.000018f);
    __mac_tap(0.000116f);
    __mac_tap(0.000197f);
    __mac_tap(0.000232f);
    __mac_tap(0.000204f);
    __mac_tap(0.000112f);
    __mac_tap(-0.000026f);
    __mac_tap(-0.000177f);
    __mac_tap(-0.000297f);
    __mac_tap(-0.000348f);
    __mac_tap(-0.000305f);
    __mac_tap(-0.000168f);
    __mac_tap(0.000036f);
    __mac_tap(0.000255f);
    __mac_tap(0.000428f);
    __mac_tap(0.000499f);
    __mac_tap(0.000435f);
    __mac_tap(0.00024f);
    __mac_tap(-0.000048f);
    __mac_tap(-0.000354f);
    __mac_tap(-0.000594f);
    __mac_tap(-0.000691f);
    __mac_tap(-0.000602f);
    __mac_tap(-0.000332f);
    __mac_tap(0.000061f);
    __mac_tap(0.000477f);
    __mac_tap(0.000802f);
    __mac_tap(0.000932f);
    __mac_tap(0.000811f);
    __mac_tap(0.000449f);
    __mac_tap(-0.000076f);
    __mac_tap(-0.000629f);
    __mac_tap(-0.001058f);
    __mac_tap(-0.001229f);
    __mac_tap(-0.00107f);
    __mac_tap(-0.000594f);
    __mac_tap(0.000092f);
    __mac_tap(0.000814f);
    __mac_tap(0.001373f);
    __mac_tap(0.001595f);
    __mac_tap(0.001389f);
    __mac_tap(0.000775f);
    __mac_tap(-0.000109f);
    __mac_tap(-0.001038f);
    __mac_tap(-0.001755f);
    __mac_tap(-0.00204f);
    __mac_tap(-0.001778f);
    __mac_tap(-0.000997f);
    __mac_tap(0.000127f);
    __mac_tap(0.001308f);
    __mac_tap(0.002219f);
    __mac_tap(0.002583f);
    __mac_tap(0.002255f);
    __mac_tap(0.001271f);
    __mac_tap(-0.000146f);
    __mac_tap(-0.001633f);
    __mac_tap(-0.002782f);
    __mac_tap(-0.003245f);
    __mac_tap(-0.002839f);
    __mac_tap(-0.001609f);
    __mac_tap(0.000164f);
    __mac_tap(0.002029f);
    __mac_tap(0.003472f);
    __mac_tap(0.004059f);
    __mac_tap(0.00356f);
    __mac_tap(0.002031f);
    __mac_tap(-0.000183f);
    __mac_tap(-0.002516f);
    __mac_tap(-0.004328f);
    __mac_tap(-0.005075f);
    __mac_tap(-0.004466f);
    __mac_tap(-0.002565f);
    __mac_tap(0.0002f);
    __mac_tap(0.003127f);
    __mac_tap(0.005413f);
    __mac_tap(0.006372f);
    __mac_tap(0.005631f);
    __mac_tap(0.003259f);
    __mac_tap(-0.000216f);
    __mac_tap(-0.003918f);
    __mac_tap(-0.006835f);
    __mac_tap(-0.008088f);
    __mac_tap(-0.007188f);
    __mac_tap(-0.004199f);
    __mac_tap(0.000231f);
    __mac_tap(0.004994f);
    __mac_tap(0.008795f);
    __mac_tap(0.010485f);
    __mac_tap(0.009391f);
    __mac_tap(0.005549f);
    __mac_tap(-0.000243f);
    __mac_tap(-0.006569f);
    __mac_tap(-0.011718f);
    __mac_tap(-0.014124f);
    __mac_tap(-0.012803f);
    __mac_tap(-0.007688f);
    __mac_tap(0.000253f);
    __mac_tap(0.00916f);
    __mac_tap(0.016666f);
    __mac_tap(0.020472f);
    __mac_tap(0.018954f);
    __mac_tap(0.011692f);
    __mac_tap(-0.000261f);
    __mac_tap(-0.014419f);
    __mac_tap(-0.027262f);
    __mac_tap(-0.034914f);
    __mac_tap(-0.033976f);
    __mac_tap(-0.02233f);
    __mac_tap(0.000265f);
    __mac_tap(0.031935f);
    __mac_tap(0.068874f);
    __mac_tap(0.105951f);
    __mac_tap(0.137611f);
    __mac_tap(0.158899f);
    __mac_tap(0.1664f);
    __mac_tap(0.158899f);
    __mac_tap(0.137611f);
    __mac_tap(0.105951f);
    __mac_tap(0.068874f);
    __mac_tap(0.031935f);
    __mac_tap(0.000265f);
    __mac_tap(-0.02233f);
    __mac_tap(-0.033976f);
    __mac_tap(-0.034914f);
    __mac_tap(-0.027262f);
    __mac_tap(-0.014419f);
    __mac_tap(-0.000261f);
    __mac_tap(0.011692f);
    __mac_tap(0.018954f);
    __mac_tap(0.020472f);
    __mac_tap(0.016666f);
    __mac_tap(0.00916f);
    __mac_tap(0.000253f);
    __mac_tap(-0.007688f);
    __mac_tap(-0.012803f);
    __mac_tap(-0.014124f);
    __mac_tap(-0.011718f);
    __mac_tap(-0.006569f);
    __mac_tap(-0.000243f);
    __mac_tap(0.005549f);
    __mac_tap(0.009391f);
    __mac_tap(0.010485f);
    __mac_tap(0.008795f);
    __mac_tap(0.004994f);
    __mac_tap(0.000231f);
    __mac_tap(-0.004199f);
    __mac_tap(-0.007188f);
    __mac_tap(-0.008088f);
    __mac_tap(-0.006835f);
    __mac_tap(-0.003918f);
    __mac_tap(-0.000216f);
    __mac_tap(0.003259f);
    __mac_tap(0.005631f);
    __mac_tap(0.006372f);
    __mac_tap(0.005413f);
    __mac_tap(0.003127f);
    __mac_tap(0.0002f);
    __mac_tap(-0.002565f);
    __mac_tap(-0.004466f);
    __mac_tap(-0.005075f);
    __mac_tap(-0.004328f);
    __mac_tap(-0.002516f);
    __mac_tap(-0.000183f);
    __mac_tap(0.002031f);
    __mac_tap(0.00356f);
    __mac_tap(0.004059f);
    __mac_tap(0.003472f);
    __mac_tap(0.002029f);
    __mac_tap(0.000164f);
    __mac_tap(-0.001609f);
    __mac_tap(-0.002839f);
    __mac_tap(-0.003245f);
    __mac_tap(-0.002782f);
    __mac_tap(-0.001633f);
    __mac_tap(-0.000146f);
    __mac_tap(0.001271f);
    __mac_tap(0.002255f);
    __mac_tap(0.002583f);
    __mac_tap(0.002219f);
    __mac_tap(0.001308f);
    __mac_tap(0.000127f);
    __mac_tap(-0.000997f);
    __mac_tap(-0.001778f);
    __mac_tap(-0.00204f);
    __mac_tap(-0.001755f);
    __mac_tap(-0.001038f);
    __mac_tap(-0.000109f);
    __mac_tap(0.000775f);
    __mac_tap(0.001389f);
    __mac_tap(0.001595f);
    __mac_tap(0.001373f);
    __mac_tap(0.000814f);
    __mac_tap(0.000092f);
    __mac_tap(-0.000594f);
    __mac_tap(-0.00107f);
    __mac_tap(-0.001229f);
    __mac_tap(-0.001058f);
    __mac_tap(-0.000629f);
    __mac_tap(-0.000076f);
    __mac_tap(0.000449f);
    __mac_tap(0.000811f);
    __mac_tap(0.000932f);
    __mac_tap(0.000802f);
    __mac_tap(0.000477f);
    __mac_tap(0.000061f);
    __mac_tap(-0.000332f);
    __mac_tap(-0.000602f);
    __mac_tap(-0.000691f);
    __mac_tap(-0.000594f);
    __mac_tap(-0.000354f);
    __mac_tap(-0.000048f);
    __mac_tap(0.00024f);
    __mac_tap(0.000435f);
    __mac_tap(0.000499f);
    __mac_tap(0.000428f);
    __mac_tap(0.000255f);
    __mac_tap(0.000036f);
    __mac_tap(-0.000168f);
    __mac_tap(-0.000305f);
    __mac_tap(-0.000348f);
    __mac_tap(-0.000297f);
    __mac_tap(-0.000177f);
    __mac_tap(-0.000026f);
    __mac_tap(0.000112f);
    __mac_tap(0.000204f);
    __mac_tap(0.000232f);
    __mac_tap(0.000197f);
    __mac_tap(0.000116f);
    __mac_tap(0.000018f);
    __mac_tap(-0.000071f);
    __mac_tap(-0.000129f);
    __mac_tap(-0.000145f);
    __mac_tap(-0.000122f);
    __mac_tap(-0.000071f);
    __mac_tap(-0.000011f);
    __mac_tap(0.000041f);
    __mac_tap(0.000074f);
    __mac_tap(0.000082f);
    __mac_tap(0.000067f);
    __mac_tap(0.000039f);
    __mac_tap(0.000006f);
    __mac_tap(-0.000021f);
    return acc;
  }

  static const float __not_in_flash_func(bpf_700)(const float sample)
  {
    // 31250
    // att: 60dB
    // Lo: 600
    // Hi: 800
    // 255 taps
    static float x[FIR_LENGTH] = { 0.0f };
    static uint8_t sample_index = 0;
    uint8_t i = sample_index;
    x[sample_index--] = sample;
    float acc = 0;
    __mac_tap(0.000032f);
    __mac_tap(0.000029f);
    __mac_tap(0.000024f);
    __mac_tap(0.000015f);
    __mac_tap(0.000003f);
    __mac_tap(-0.000013f);
    __mac_tap(-0.000032f);
    __mac_tap(-0.000056f);
    __mac_tap(-0.000084f);
    __mac_tap(-0.000116f);
    __mac_tap(-0.00015f);
    __mac_tap(-0.000187f);
    __mac_tap(-0.000225f);
    __mac_tap(-0.000264f);
    __mac_tap(-0.000301f);
    __mac_tap(-0.000336f);
    __mac_tap(-0.000367f);
    __mac_tap(-0.000391f);
    __mac_tap(-0.000408f);
    __mac_tap(-0.000414f);
    __mac_tap(-0.000409f);
    __mac_tap(-0.000391f);
    __mac_tap(-0.000358f);
    __mac_tap(-0.00031f);
    __mac_tap(-0.000244f);
    __mac_tap(-0.000162f);
    __mac_tap(-0.000062f);
    __mac_tap(0.000054f);
    __mac_tap(0.000185f);
    __mac_tap(0.000329f);
    __mac_tap(0.000485f);
    __mac_tap(0.000648f);
    __mac_tap(0.000816f);
    __mac_tap(0.000984f);
    __mac_tap(0.001148f);
    __mac_tap(0.001302f);
    __mac_tap(0.001441f);
    __mac_tap(0.00156f);
    __mac_tap(0.001654f);
    __mac_tap(0.001717f);
    __mac_tap(0.001744f);
    __mac_tap(0.001731f);
    __mac_tap(0.001675f);
    __mac_tap(0.001572f);
    __mac_tap(0.00142f);
    __mac_tap(0.001219f);
    __mac_tap(0.000969f);
    __mac_tap(0.000672f);
    __mac_tap(0.000331f);
    __mac_tap(-0.000049f);
    __mac_tap(-0.000463f);
    __mac_tap(-0.000902f);
    __mac_tap(-0.001358f);
    __mac_tap(-0.001822f);
    __mac_tap(-0.002282f);
    __mac_tap(-0.002727f);
    __mac_tap(-0.003146f);
    __mac_tap(-0.003526f);
    __mac_tap(-0.003855f);
    __mac_tap(-0.004122f);
    __mac_tap(-0.004315f);
    __mac_tap(-0.004426f);
    __mac_tap(-0.004446f);
    __mac_tap(-0.004368f);
    __mac_tap(-0.004187f);
    __mac_tap(-0.003901f);
    __mac_tap(-0.003511f);
    __mac_tap(-0.003019f);
    __mac_tap(-0.002429f);
    __mac_tap(-0.00175f);
    __mac_tap(-0.000992f);
    __mac_tap(-0.000168f);
    __mac_tap(0.000706f);
    __mac_tap(0.001615f);
    __mac_tap(0.002538f);
    __mac_tap(0.003456f);
    __mac_tap(0.004347f);
    __mac_tap(0.005191f);
    __mac_tap(0.005965f);
    __mac_tap(0.006649f);
    __mac_tap(0.007224f);
    __mac_tap(0.007672f);
    __mac_tap(0.007977f);
    __mac_tap(0.008126f);
    __mac_tap(0.008109f);
    __mac_tap(0.00792f);
    __mac_tap(0.007556f);
    __mac_tap(0.007018f);
    __mac_tap(0.006311f);
    __mac_tap(0.005446f);
    __mac_tap(0.004435f);
    __mac_tap(0.003295f);
    __mac_tap(0.002047f);
    __mac_tap(0.000715f);
    __mac_tap(-0.000676f);
    __mac_tap(-0.002096f);
    __mac_tap(-0.003515f);
    __mac_tap(-0.004902f);
    __mac_tap(-0.006227f);
    __mac_tap(-0.00746f);
    __mac_tap(-0.00857f);
    __mac_tap(-0.009531f);
    __mac_tap(-0.010319f);
    __mac_tap(-0.010912f);
    __mac_tap(-0.011294f);
    __mac_tap(-0.011452f);
    __mac_tap(-0.011378f);
    __mac_tap(-0.011068f);
    __mac_tap(-0.010526f);
    __mac_tap(-0.009759f);
    __mac_tap(-0.008779f);
    __mac_tap(-0.007604f);
    __mac_tap(-0.006257f);
    __mac_tap(-0.004763f);
    __mac_tap(-0.003153f);
    __mac_tap(-0.00146f);
    __mac_tap(0.000282f);
    __mac_tap(0.002035f);
    __mac_tap(0.003763f);
    __mac_tap(0.005429f);
    __mac_tap(0.006996f);
    __mac_tap(0.008432f);
    __mac_tap(0.009704f);
    __mac_tap(0.010785f);
    __mac_tap(0.011652f);
    __mac_tap(0.012285f);
    __mac_tap(0.012671f);
    __mac_tap(0.0128f);
    __mac_tap(0.012671f);
    __mac_tap(0.012285f);
    __mac_tap(0.011652f);
    __mac_tap(0.010785f);
    __mac_tap(0.009704f);
    __mac_tap(0.008432f);
    __mac_tap(0.006996f);
    __mac_tap(0.005429f);
    __mac_tap(0.003763f);
    __mac_tap(0.002035f);
    __mac_tap(0.000282f);
    __mac_tap(-0.00146f);
    __mac_tap(-0.003153f);
    __mac_tap(-0.004763f);
    __mac_tap(-0.006257f);
    __mac_tap(-0.007604f);
    __mac_tap(-0.008779f);
    __mac_tap(-0.009759f);
    __mac_tap(-0.010526f);
    __mac_tap(-0.011068f);
    __mac_tap(-0.011378f);
    __mac_tap(-0.011452f);
    __mac_tap(-0.011294f);
    __mac_tap(-0.010912f);
    __mac_tap(-0.010319f);
    __mac_tap(-0.009531f);
    __mac_tap(-0.00857f);
    __mac_tap(-0.00746f);
    __mac_tap(-0.006227f);
    __mac_tap(-0.004902f);
    __mac_tap(-0.003515f);
    __mac_tap(-0.002096f);
    __mac_tap(-0.000676f);
    __mac_tap(0.000715f);
    __mac_tap(0.002047f);
    __mac_tap(0.003295f);
    __mac_tap(0.004435f);
    __mac_tap(0.005446f);
    __mac_tap(0.006311f);
    __mac_tap(0.007018f);
    __mac_tap(0.007556f);
    __mac_tap(0.00792f);
    __mac_tap(0.008109f);
    __mac_tap(0.008126f);
    __mac_tap(0.007977f);
    __mac_tap(0.007672f);
    __mac_tap(0.007224f);
    __mac_tap(0.006649f);
    __mac_tap(0.005965f);
    __mac_tap(0.005191f);
    __mac_tap(0.004347f);
    __mac_tap(0.003456f);
    __mac_tap(0.002538f);
    __mac_tap(0.001615f);
    __mac_tap(0.000706f);
    __mac_tap(-0.000168f);
    __mac_tap(-0.000992f);
    __mac_tap(-0.00175f);
    __mac_tap(-0.002429f);
    __mac_tap(-0.003019f);
    __mac_tap(-0.003511f);
    __mac_tap(-0.003901f);
    __mac_tap(-0.004187f);
    __mac_tap(-0.004368f);
    __mac_tap(-0.004446f);
    __mac_tap(-0.004426f);
    __mac_tap(-0.004315f);
    __mac_tap(-0.004122f);
    __mac_tap(-0.003855f);
    __mac_tap(-0.003526f);
    __mac_tap(-0.003146f);
    __mac_tap(-0.002727f);
    __mac_tap(-0.002282f);
    __mac_tap(-0.001822f);
    __mac_tap(-0.001358f);
    __mac_tap(-0.000902f);
    __mac_tap(-0.000463f);
    __mac_tap(-0.000049f);
    __mac_tap(0.000331f);
    __mac_tap(0.000672f);
    __mac_tap(0.000969f);
    __mac_tap(0.001219f);
    __mac_tap(0.00142f);
    __mac_tap(0.001572f);
    __mac_tap(0.001675f);
    __mac_tap(0.001731f);
    __mac_tap(0.001744f);
    __mac_tap(0.001717f);
    __mac_tap(0.001654f);
    __mac_tap(0.00156f);
    __mac_tap(0.001441f);
    __mac_tap(0.001302f);
    __mac_tap(0.001148f);
    __mac_tap(0.000984f);
    __mac_tap(0.000816f);
    __mac_tap(0.000648f);
    __mac_tap(0.000485f);
    __mac_tap(0.000329f);
    __mac_tap(0.000185f);
    __mac_tap(0.000054f);
    __mac_tap(-0.000062f);
    __mac_tap(-0.000162f);
    __mac_tap(-0.000244f);
    __mac_tap(-0.00031f);
    __mac_tap(-0.000358f);
    __mac_tap(-0.000391f);
    __mac_tap(-0.000409f);
    __mac_tap(-0.000414f);
    __mac_tap(-0.000408f);
    __mac_tap(-0.000391f);
    __mac_tap(-0.000367f);
    __mac_tap(-0.000336f);
    __mac_tap(-0.000301f);
    __mac_tap(-0.000264f);
    __mac_tap(-0.000225f);
    __mac_tap(-0.000187f);
    __mac_tap(-0.00015f);
    __mac_tap(-0.000116f);
    __mac_tap(-0.000084f);
    __mac_tap(-0.000056f);
    __mac_tap(-0.000032f);
    __mac_tap(-0.000013f);
    __mac_tap(0.000003f);
    __mac_tap(0.000015f);
    __mac_tap(0.000024f);
    __mac_tap(0.000029f);
    __mac_tap(0.000032f);
    return acc;
  }

  static const float __not_in_flash_func(lpf_2600f_tx)(const float sample)
  {
    // 31250
    // att: 60dB
    static float x[FIR_LENGTH] = { 0.0f };
    static uint8_t sample_index = 0;
    uint8_t i = sample_index;
    x[sample_index--] = sample;
    float acc = 0;
    __mac_tap(-0.000021f);
    __mac_tap(0.000006f);
    __mac_tap(0.000039f);
    __mac_tap(0.000067f);
    __mac_tap(0.000082f);
    __mac_tap(0.000074f);
    __mac_tap(0.000041f);
    __mac_tap(-0.000011f);
    __mac_tap(-0.000071f);
    __mac_tap(-0.000122f);
    __mac_tap(-0.000145f);
    __mac_tap(-0.000129f);
    __mac_tap(-0.000071f);
    __mac_tap(0.000018f);
    __mac_tap(0.000116f);
    __mac_tap(0.000197f);
    __mac_tap(0.000232f);
    __mac_tap(0.000204f);
    __mac_tap(0.000112f);
    __mac_tap(-0.000026f);
    __mac_tap(-0.000177f);
    __mac_tap(-0.000297f);
    __mac_tap(-0.000348f);
    __mac_tap(-0.000305f);
    __mac_tap(-0.000168f);
    __mac_tap(0.000036f);
    __mac_tap(0.000255f);
    __mac_tap(0.000428f);
    __mac_tap(0.000499f);
    __mac_tap(0.000435f);
    __mac_tap(0.00024f);
    __mac_tap(-0.000048f);
    __mac_tap(-0.000354f);
    __mac_tap(-0.000594f);
    __mac_tap(-0.000691f);
    __mac_tap(-0.000602f);
    __mac_tap(-0.000332f);
    __mac_tap(0.000061f);
    __mac_tap(0.000477f);
    __mac_tap(0.000802f);
    __mac_tap(0.000932f);
    __mac_tap(0.000811f);
    __mac_tap(0.000449f);
    __mac_tap(-0.000076f);
    __mac_tap(-0.000629f);
    __mac_tap(-0.001058f);
    __mac_tap(-0.001229f);
    __mac_tap(-0.00107f);
    __mac_tap(-0.000594f);
    __mac_tap(0.000092f);
    __mac_tap(0.000814f);
    __mac_tap(0.001373f);
    __mac_tap(0.001595f);
    __mac_tap(0.001389f);
    __mac_tap(0.000775f);
    __mac_tap(-0.000109f);
    __mac_tap(-0.001038f);
    __mac_tap(-0.001755f);
    __mac_tap(-0.00204f);
    __mac_tap(-0.001778f);
    __mac_tap(-0.000997f);
    __mac_tap(0.000127f);
    __mac_tap(0.001308f);
    __mac_tap(0.002219f);
    __mac_tap(0.002583f);
    __mac_tap(0.002255f);
    __mac_tap(0.001271f);
    __mac_tap(-0.000146f);
    __mac_tap(-0.001633f);
    __mac_tap(-0.002782f);
    __mac_tap(-0.003245f);
    __mac_tap(-0.002839f);
    __mac_tap(-0.001609f);
    __mac_tap(0.000164f);
    __mac_tap(0.002029f);
    __mac_tap(0.003472f);
    __mac_tap(0.004059f);
    __mac_tap(0.00356f);
    __mac_tap(0.002031f);
    __mac_tap(-0.000183f);
    __mac_tap(-0.002516f);
    __mac_tap(-0.004328f);
    __mac_tap(-0.005075f);
    __mac_tap(-0.004466f);
    __mac_tap(-0.002565f);
    __mac_tap(0.0002f);
    __mac_tap(0.003127f);
    __mac_tap(0.005413f);
    __mac_tap(0.006372f);
    __mac_tap(0.005631f);
    __mac_tap(0.003259f);
    __mac_tap(-0.000216f);
    __mac_tap(-0.003918f);
    __mac_tap(-0.006835f);
    __mac_tap(-0.008088f);
    __mac_tap(-0.007188f);
    __mac_tap(-0.004199f);
    __mac_tap(0.000231f);
    __mac_tap(0.004994f);
    __mac_tap(0.008795f);
    __mac_tap(0.010485f);
    __mac_tap(0.009391f);
    __mac_tap(0.005549f);
    __mac_tap(-0.000243f);
    __mac_tap(-0.006569f);
    __mac_tap(-0.011718f);
    __mac_tap(-0.014124f);
    __mac_tap(-0.012803f);
    __mac_tap(-0.007688f);
    __mac_tap(0.000253f);
    __mac_tap(0.00916f);
    __mac_tap(0.016666f);
    __mac_tap(0.020472f);
    __mac_tap(0.018954f);
    __mac_tap(0.011692f);
    __mac_tap(-0.000261f);
    __mac_tap(-0.014419f);
    __mac_tap(-0.027262f);
    __mac_tap(-0.034914f);
    __mac_tap(-0.033976f);
    __mac_tap(-0.02233f);
    __mac_tap(0.000265f);
    __mac_tap(0.031935f);
    __mac_tap(0.068874f);
    __mac_tap(0.105951f);
    __mac_tap(0.137611f);
    __mac_tap(0.158899f);
    __mac_tap(0.1664f);
    __mac_tap(0.158899f);
    __mac_tap(0.137611f);
    __mac_tap(0.105951f);
    __mac_tap(0.068874f);
    __mac_tap(0.031935f);
    __mac_tap(0.000265f);
    __mac_tap(-0.02233f);
    __mac_tap(-0.033976f);
    __mac_tap(-0.034914f);
    __mac_tap(-0.027262f);
    __mac_tap(-0.014419f);
    __mac_tap(-0.000261f);
    __mac_tap(0.011692f);
    __mac_tap(0.018954f);
    __mac_tap(0.020472f);
    __mac_tap(0.016666f);
    __mac_tap(0.00916f);
    __mac_tap(0.000253f);
    __mac_tap(-0.007688f);
    __mac_tap(-0.012803f);
    __mac_tap(-0.014124f);
    __mac_tap(-0.011718f);
    __mac_tap(-0.006569f);
    __mac_tap(-0.000243f);
    __mac_tap(0.005549f);
    __mac_tap(0.009391f);
    __mac_tap(0.010485f);
    __mac_tap(0.008795f);
    __mac_tap(0.004994f);
    __mac_tap(0.000231f);
    __mac_tap(-0.004199f);
    __mac_tap(-0.007188f);
    __mac_tap(-0.008088f);
    __mac_tap(-0.006835f);
    __mac_tap(-0.003918f);
    __mac_tap(-0.000216f);
    __mac_tap(0.003259f);
    __mac_tap(0.005631f);
    __mac_tap(0.006372f);
    __mac_tap(0.005413f);
    __mac_tap(0.003127f);
    __mac_tap(0.0002f);
    __mac_tap(-0.002565f);
    __mac_tap(-0.004466f);
    __mac_tap(-0.005075f);
    __mac_tap(-0.004328f);
    __mac_tap(-0.002516f);
    __mac_tap(-0.000183f);
    __mac_tap(0.002031f);
    __mac_tap(0.00356f);
    __mac_tap(0.004059f);
    __mac_tap(0.003472f);
    __mac_tap(0.002029f);
    __mac_tap(0.000164f);
    __mac_tap(-0.001609f);
    __mac_tap(-0.002839f);
    __mac_tap(-0.003245f);
    __mac_tap(-0.002782f);
    __mac_tap(-0.001633f);
    __mac_tap(-0.000146f);
    __mac_tap(0.001271f);
    __mac_tap(0.002255f);
    __mac_tap(0.002583f);
    __mac_tap(0.002219f);
    __mac_tap(0.001308f);
    __mac_tap(0.000127f);
    __mac_tap(-0.000997f);
    __mac_tap(-0.001778f);
    __mac_tap(-0.00204f);
    __mac_tap(-0.001755f);
    __mac_tap(-0.001038f);
    __mac_tap(-0.000109f);
    __mac_tap(0.000775f);
    __mac_tap(0.001389f);
    __mac_tap(0.001595f);
    __mac_tap(0.001373f);
    __mac_tap(0.000814f);
    __mac_tap(0.000092f);
    __mac_tap(-0.000594f);
    __mac_tap(-0.00107f);
    __mac_tap(-0.001229f);
    __mac_tap(-0.001058f);
    __mac_tap(-0.000629f);
    __mac_tap(-0.000076f);
    __mac_tap(0.000449f);
    __mac_tap(0.000811f);
    __mac_tap(0.000932f);
    __mac_tap(0.000802f);
    __mac_tap(0.000477f);
    __mac_tap(0.000061f);
    __mac_tap(-0.000332f);
    __mac_tap(-0.000602f);
    __mac_tap(-0.000691f);
    __mac_tap(-0.000594f);
    __mac_tap(-0.000354f);
    __mac_tap(-0.000048f);
    __mac_tap(0.00024f);
    __mac_tap(0.000435f);
    __mac_tap(0.000499f);
    __mac_tap(0.000428f);
    __mac_tap(0.000255f);
    __mac_tap(0.000036f);
    __mac_tap(-0.000168f);
    __mac_tap(-0.000305f);
    __mac_tap(-0.000348f);
    __mac_tap(-0.000297f);
    __mac_tap(-0.000177f);
    __mac_tap(-0.000026f);
    __mac_tap(0.000112f);
    __mac_tap(0.000204f);
    __mac_tap(0.000232f);
    __mac_tap(0.000197f);
    __mac_tap(0.000116f);
    __mac_tap(0.000018f);
    __mac_tap(-0.000071f);
    __mac_tap(-0.000129f);
    __mac_tap(-0.000145f);
    __mac_tap(-0.000122f);
    __mac_tap(-0.000071f);
    __mac_tap(-0.000011f);
    __mac_tap(0.000041f);
    __mac_tap(0.000074f);
    __mac_tap(0.000082f);
    __mac_tap(0.000067f);
    __mac_tap(0.000039f);
    __mac_tap(0.000006f);
    __mac_tap(-0.000021f);
    return acc;
  }

  static const float __not_in_flash_func(lpf_2600if_tx)(const float sample)
  {
    // 31250
    // 2600 Hz
    // att: 60dB
    // 125 taps
    static float x[FIR_LENGTH] = { 0.0f };
    static uint8_t sample_index = 0;
    uint8_t i = sample_index;
    x[sample_index--] = sample;
    float acc = 0;
    __mac_tap(0.000088f);
    __mac_tap(0.000062f);
    __mac_tap(-0.000009f);
    __mac_tap(-0.000114f);
    __mac_tap(-0.000226f);
    __mac_tap(-0.000304f);
    __mac_tap(-0.000303f);
    __mac_tap(-0.000194f);
    __mac_tap(0.000022f);
    __mac_tap(0.000305f);
    __mac_tap(0.000576f);
    __mac_tap(0.00074f);
    __mac_tap(0.000709f);
    __mac_tap(0.00044f);
    __mac_tap(-0.000043f);
    __mac_tap(-0.000637f);
    __mac_tap(-0.001178f);
    __mac_tap(-0.00148f);
    __mac_tap(-0.001391f);
    __mac_tap(-0.000851f);
    __mac_tap(0.00007f);
    __mac_tap(0.001167f);
    __mac_tap(0.002135f);
    __mac_tap(0.00265f);
    __mac_tap(0.002464f);
    __mac_tap(0.001497f);
    __mac_tap(-0.000104f);
    __mac_tap(-0.001971f);
    __mac_tap(-0.003589f);
    __mac_tap(-0.004425f);
    __mac_tap(-0.00409f);
    __mac_tap(-0.002481f);
    __mac_tap(0.000141f);
    __mac_tap(0.003166f);
    __mac_tap(0.005762f);
    __mac_tap(0.007088f);
    __mac_tap(0.006543f);
    __mac_tap(0.003978f);
    __mac_tap(-0.000179f);
    __mac_tap(-0.004967f);
    __mac_tap(-0.009082f);
    __mac_tap(-0.011206f);
    __mac_tap(-0.010386f);
    __mac_tap(-0.006369f);
    __mac_tap(0.000214f);
    __mac_tap(0.007885f);
    __mac_tap(0.014599f);
    __mac_tap(0.018227f);
    __mac_tap(0.017134f);
    __mac_tap(0.010719f);
    __mac_tap(-0.000242f);
    __mac_tap(-0.013552f);
    __mac_tap(-0.025902f);
    __mac_tap(-0.033499f);
    __mac_tap(-0.032885f);
    __mac_tap(-0.02178f);
    __mac_tap(0.00026f);
    __mac_tap(0.031531f);
    __mac_tap(0.068316f);
    __mac_tap(0.105468f);
    __mac_tap(0.137332f);
    __mac_tap(0.158818f);
    __mac_tap(0.1664f);
    __mac_tap(0.158818f);
    __mac_tap(0.137332f);
    __mac_tap(0.105468f);
    __mac_tap(0.068316f);
    __mac_tap(0.031531f);
    __mac_tap(0.00026f);
    __mac_tap(-0.02178f);
    __mac_tap(-0.032885f);
    __mac_tap(-0.033499f);
    __mac_tap(-0.025902f);
    __mac_tap(-0.013552f);
    __mac_tap(-0.000242f);
    __mac_tap(0.010719f);
    __mac_tap(0.017134f);
    __mac_tap(0.018227f);
    __mac_tap(0.014599f);
    __mac_tap(0.007885f);
    __mac_tap(0.000214f);
    __mac_tap(-0.006369f);
    __mac_tap(-0.010386f);
    __mac_tap(-0.011206f);
    __mac_tap(-0.009082f);
    __mac_tap(-0.004967f);
    __mac_tap(-0.000179f);
    __mac_tap(0.003978f);
    __mac_tap(0.006543f);
    __mac_tap(0.007088f);
    __mac_tap(0.005762f);
    __mac_tap(0.003166f);
    __mac_tap(0.000141f);
    __mac_tap(-0.002481f);
    __mac_tap(-0.00409f);
    __mac_tap(-0.004425f);
    __mac_tap(-0.003589f);
    __mac_tap(-0.001971f);
    __mac_tap(-0.000104f);
    __mac_tap(0.001497f);
    __mac_tap(0.002464f);
    __mac_tap(0.00265f);
    __mac_tap(0.002135f);
    __mac_tap(0.001167f);
    __mac_tap(0.00007f);
    __mac_tap(-0.000851f);
    __mac_tap(-0.001391f);
    __mac_tap(-0.00148f);
    __mac_tap(-0.001178f);
    __mac_tap(-0.000637f);
    __mac_tap(-0.000043f);
    __mac_tap(0.00044f);
    __mac_tap(0.000709f);
    __mac_tap(0.00074f);
    __mac_tap(0.000576f);
    __mac_tap(0.000305f);
    __mac_tap(0.000022f);
    __mac_tap(-0.000194f);
    __mac_tap(-0.000303f);
    __mac_tap(-0.000304f);
    __mac_tap(-0.000226f);
    __mac_tap(-0.000114f);
    __mac_tap(-0.000009f);
    __mac_tap(0.000062f);
    __mac_tap(0.000088f); 
    return acc;
  }

  static const float __not_in_flash_func(lpf_2600qf_tx)(const float sample)
  {
    // 31250
    // 2600 Hz
    // att: 60dB
    // 125 taps
    static float x[FIR_LENGTH] = { 0.0f };
    static uint8_t sample_index = 0;
    uint8_t i = sample_index;
    x[sample_index--] = sample;
    float acc = 0;
    __mac_tap(0.000088f);
    __mac_tap(0.000062f);
    __mac_tap(-0.000009f);
    __mac_tap(-0.000114f);
    __mac_tap(-0.000226f);
    __mac_tap(-0.000304f);
    __mac_tap(-0.000303f);
    __mac_tap(-0.000194f);
    __mac_tap(0.000022f);
    __mac_tap(0.000305f);
    __mac_tap(0.000576f);
    __mac_tap(0.00074f);
    __mac_tap(0.000709f);
    __mac_tap(0.00044f);
    __mac_tap(-0.000043f);
    __mac_tap(-0.000637f);
    __mac_tap(-0.001178f);
    __mac_tap(-0.00148f);
    __mac_tap(-0.001391f);
    __mac_tap(-0.000851f);
    __mac_tap(0.00007f);
    __mac_tap(0.001167f);
    __mac_tap(0.002135f);
    __mac_tap(0.00265f);
    __mac_tap(0.002464f);
    __mac_tap(0.001497f);
    __mac_tap(-0.000104f);
    __mac_tap(-0.001971f);
    __mac_tap(-0.003589f);
    __mac_tap(-0.004425f);
    __mac_tap(-0.00409f);
    __mac_tap(-0.002481f);
    __mac_tap(0.000141f);
    __mac_tap(0.003166f);
    __mac_tap(0.005762f);
    __mac_tap(0.007088f);
    __mac_tap(0.006543f);
    __mac_tap(0.003978f);
    __mac_tap(-0.000179f);
    __mac_tap(-0.004967f);
    __mac_tap(-0.009082f);
    __mac_tap(-0.011206f);
    __mac_tap(-0.010386f);
    __mac_tap(-0.006369f);
    __mac_tap(0.000214f);
    __mac_tap(0.007885f);
    __mac_tap(0.014599f);
    __mac_tap(0.018227f);
    __mac_tap(0.017134f);
    __mac_tap(0.010719f);
    __mac_tap(-0.000242f);
    __mac_tap(-0.013552f);
    __mac_tap(-0.025902f);
    __mac_tap(-0.033499f);
    __mac_tap(-0.032885f);
    __mac_tap(-0.02178f);
    __mac_tap(0.00026f);
    __mac_tap(0.031531f);
    __mac_tap(0.068316f);
    __mac_tap(0.105468f);
    __mac_tap(0.137332f);
    __mac_tap(0.158818f);
    __mac_tap(0.1664f);
    __mac_tap(0.158818f);
    __mac_tap(0.137332f);
    __mac_tap(0.105468f);
    __mac_tap(0.068316f);
    __mac_tap(0.031531f);
    __mac_tap(0.00026f);
    __mac_tap(-0.02178f);
    __mac_tap(-0.032885f);
    __mac_tap(-0.033499f);
    __mac_tap(-0.025902f);
    __mac_tap(-0.013552f);
    __mac_tap(-0.000242f);
    __mac_tap(0.010719f);
    __mac_tap(0.017134f);
    __mac_tap(0.018227f);
    __mac_tap(0.014599f);
    __mac_tap(0.007885f);
    __mac_tap(0.000214f);
    __mac_tap(-0.006369f);
    __mac_tap(-0.010386f);
    __mac_tap(-0.011206f);
    __mac_tap(-0.009082f);
    __mac_tap(-0.004967f);
    __mac_tap(-0.000179f);
    __mac_tap(0.003978f);
    __mac_tap(0.006543f);
    __mac_tap(0.007088f);
    __mac_tap(0.005762f);
    __mac_tap(0.003166f);
    __mac_tap(0.000141f);
    __mac_tap(-0.002481f);
    __mac_tap(-0.00409f);
    __mac_tap(-0.004425f);
    __mac_tap(-0.003589f);
    __mac_tap(-0.001971f);
    __mac_tap(-0.000104f);
    __mac_tap(0.001497f);
    __mac_tap(0.002464f);
    __mac_tap(0.00265f);
    __mac_tap(0.002135f);
    __mac_tap(0.001167f);
    __mac_tap(0.00007f);
    __mac_tap(-0.000851f);
    __mac_tap(-0.001391f);
    __mac_tap(-0.00148f);
    __mac_tap(-0.001178f);
    __mac_tap(-0.000637f);
    __mac_tap(-0.000043f);
    __mac_tap(0.00044f);
    __mac_tap(0.000709f);
    __mac_tap(0.00074f);
    __mac_tap(0.000576f);
    __mac_tap(0.000305f);
    __mac_tap(0.000022f);
    __mac_tap(-0.000194f);
    __mac_tap(-0.000303f);
    __mac_tap(-0.000304f);
    __mac_tap(-0.000226f);
    __mac_tap(-0.000114f);
    __mac_tap(-0.000009f);
    __mac_tap(0.000062f);
    __mac_tap(0.000088f);
    return acc;
  }

}

#endif