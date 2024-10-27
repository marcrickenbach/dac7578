#
# Copyright (c) 2024 FKMG Circuits
#

zephyr_include_directories(include)
add_subdirectory_ifdef(CONFIG_DAC7578 drivers/dac)