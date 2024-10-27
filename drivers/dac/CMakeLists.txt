#
# Copyright (c) 2022 Circuit Dojo LLC
#

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_DAC7578 dac7578.c)