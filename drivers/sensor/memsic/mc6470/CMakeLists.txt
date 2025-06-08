# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2025 Michael Estes

zephyr_library()

zephyr_library_sources(mc6470_accel.c)
zephyr_library_sources_ifdef(CONFIG_MC6470_TRIGGER mc6470_accel_trigger.c)