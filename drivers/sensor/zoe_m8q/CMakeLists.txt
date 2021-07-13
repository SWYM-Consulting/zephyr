# SPDX-License-Identifier: Apache-2.0

zephyr_library()

if (CONFIG_ZOEM8Q)
  zephyr_library_sources(zoe_m8q.c)
  zephyr_library_sources(nmea.c)
endif()
