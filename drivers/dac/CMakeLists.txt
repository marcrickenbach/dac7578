#
# Copyright (c) 2024 FKMG Circuits
#

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_DAC7578 dac7578.c)