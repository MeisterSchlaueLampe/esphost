import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_FREQUENCY,
    CONF_ID,
    CONF_SCAN,
    PLATFORM_HOST,
)

CODEOWNERS = ["@your_username"]  # Replace with your GitHub username
DEPENDENCIES = ["host"]  # Only works on host platform

# Define the namespace for our component
ch341_i2c_ns = cg.esphome_ns.namespace("ch341_i2c")

# Define our custom I2C bus class
CH341I2CBus = ch341_i2c_ns.class_("CH341I2CBus", i2c.I2CBus, cg.Component)

# Supported frequencies (from CH341 datasheet)
SUPPORTED_FREQUENCIES = {
    20000: 0,    # 20kHz
    100000: 1,   # 100kHz (default)
    400000: 2,   # 400kHz
    750000: 3,   # 750kHz
}

def validate_frequency(value):
    """Validate and convert frequency to CH341 speed setting."""
    value = cv.frequency(value)
    if value not in SUPPORTED_FREQUENCIES:
        raise cv.Invalid(
            f"Unsupported frequency {value}Hz. Supported frequencies are: "
            f"{', '.join(f'{f}Hz' for f in SUPPORTED_FREQUENCIES.keys())}"
        )
    return value

# Configuration schema
CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(CH341I2CBus),
        cv.Optional(CONF_FREQUENCY, default=100000): validate_frequency,
        cv.Optional(CONF_SCAN, default=True): cv.boolean,
    }).extend(cv.COMPONENT_SCHEMA),
    cv.only_on( PLATFORM_HOST ),  # Only allow on host platform
)

async def to_code(config):
    """Generate C++ code for the component."""
    # Create the component instance
    var = cg.new_Pvariable(config[CONF_ID])
    
    # Register as both a component and I2C bus
    await cg.register_component(var, config)
#    await i2c.register_i2c_bus(var, config)
    
    # Set frequency
    cg.add(var.set_frequency(config[CONF_FREQUENCY]))
    
    # Set scan flag
    cg.add(var.set_scan(config[CONF_SCAN]))
    
    # Add required libraries for compilation
    cg.add_library("libusb-1.0", None)
    
    # Add build flags to link libusb
    cg.add_build_flag("-lusb-1.0")
    
    # Add include directories if needed
    # cg.add_build_flag("-I/usr/include/libusb-1.0")

