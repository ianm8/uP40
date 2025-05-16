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

// Voice Announce

#ifndef ANNOUNCE_H
#define ANNOUNCE_H

#define ANNOUNCE_MODE_LSB 0
#define ANNOUNCE_MODE_USB 1
#define ANNOUNCE_MODE_CWL 2
#define ANNOUNCE_MODE_CWU 3

namespace ANNOUNCE
{
  #include "audiodata.h"

  static const uint32_t lsb_data_size = sizeof(LSB_data);
  static const uint32_t usb_data_size = sizeof(USB_data);
  static const uint32_t cwl_data_size = sizeof(CWL_data);
  static const uint32_t cwu_data_size = sizeof(CWU_data);
  static const uint32_t step10_data_size = sizeof(Step10_data);
  static const uint32_t step100_data_size = sizeof(Step100_data);
  static const uint32_t step1000_data_size = sizeof(Step1000_data);

  volatile static const uint8_t *data = NULL;
  volatile static uint32_t data_size = 0;
  volatile static uint32_t p_sample = 0;
  volatile static bool active = false;

  static void __not_in_flash_func(setMode)(const uint32_t the_mode)
  {
    if (active)
    {
      return;
    }
    switch (the_mode)
    {
      case ANNOUNCE_MODE_LSB:
      {
        data = LSB_data;
        data_size = lsb_data_size;
        p_sample = 16 * 3;
        active = true;
        break;
      }
      case ANNOUNCE_MODE_USB:
      {
        data = USB_data;
        data_size = usb_data_size;
        p_sample = 16 * 3;
        active = true;
        break;
      }
      case ANNOUNCE_MODE_CWL:
      {
        data = CWL_data;
        data_size = cwl_data_size;
        p_sample = 16 * 3;
        active = true;
        break;
      }
      case ANNOUNCE_MODE_CWU:
      {
        data = CWU_data;
        data_size = cwu_data_size;
        p_sample = 16 * 3;
        active = true;
        break;
      }
    }
  }

  static void __not_in_flash_func(setStep)(const uint32_t step)
  {
    if (active)
    {
      return;
    }
    switch (step)
    {
      case 10:
      {
        data = Step10_data;
        data_size = step10_data_size;
        p_sample = 16 * 3;
        active = true;
        break;
      }
      case 100:
      {
        data = Step100_data;
        data_size = step100_data_size;
        p_sample = 16 * 3;
        active = true;
        break;
      }
      case 1000:
      {
        data = Step1000_data;
        data_size = step1000_data_size;
        p_sample = 16 * 3;
        active = true;
        break;
      }
    }
  }

  static const int16_t __not_in_flash_func(announce)(void)
  {
    if (!active)
    {
      return 0;
    }
    if (p_sample>=data_size)
    {
      active = false;
      return 0;
    }
    const int16_t sample = (int16_t)data[p_sample++] - 128;
    return sample<<4;
  }
}

#endif