
#FIXME - remove features and add header


# SPDX-License-Identifier: Apache-2.0

zephyr_library()

if (CONFIG_ZOEM8Q)
  zephyr_library_sources(zoe_m8q.c)
#  zephyr_library_sources(icm42605_setup.c)
#  zephyr_library_sources_ifdef(CONFIG_SPI icm42605_spi.c)
#  zephyr_library_sources_ifdef(CONFIG_ICM42605_TRIGGER icm42605_trigger.c)
endif()
