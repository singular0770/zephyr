# Copyright Michael Estes
# SPDX-License-Identifier: Apache-2.0

# Suppress "unique_unit_address_if_enabled" to handle the following overlaps:
# - axi_io@80010000 & can@80010000
# The memory region is needed to mark the AXI peripheral as IO for the MMU
list(APPEND EXTRA_DTC_FLAGS "-Wno-unique_unit_address_if_enabled")