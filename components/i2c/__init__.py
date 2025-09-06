import logging

from esphome import pins
import esphome.codegen as cg
from esphome.components import esp32
from esphome.config_helpers import filter_source_files_from_platform
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_FREQUENCY,
    CONF_I2C_ID,
    CONF_ID,
    CONF_SCAN,
    CONF_SCL,
    CONF_SDA,
    CONF_TIMEOUT,
    KEY_CORE,
    KEY_FRAMEWORK_VERSION,
    PLATFORM_ESP32,
    PLATFORM_ESP8266,
    PLATFORM_RP2040,
    PLATFORM_HOST,
    PlatformFramework,
)

from esphome.core import CORE

CODEOWNERS = ["@esphome/core"]
DEPENDENCIES = []
MULTI_CONF = True

i2c_ns = cg.esphome_ns.namespace("i2c")
I2CBus = i2c_ns.class_("I2CBus", cg.Component)
ArduinoI2CBus = i2c_ns.class_("ArduinoI2CBus", I2CBus)
IDFI2CBus = i2c_ns.class_("IDFI2CBus", I2CBus)
LinuxI2CBus = i2c_ns.class_("LinuxI2CBus", I2CBus)

I2CDevice = i2c_ns.class_("I2CDevice")

CONF_LINUX_DEVICE = "linux_device"
CONF_SCAN = "scan"

MULTI_CONF = True


def _bus_declare_type(value):
    if CORE.is_esp32:
        if CORE.using_arduino:
            return cv.declare_id(ArduinoI2CBus)(value)
        if CORE.using_esp_idf:
            return cv.declare_id(IDFI2CBus)(value)
    elif CORE.is_esp8266 and CORE.using_arduino:
        return cv.declare_id(ArduinoI2CBus)(value)
    elif CORE.is_rp2040 and CORE.using_arduino:
        return cv.declare_id(ArduinoI2CBus)(value)
    elif CORE.is_bk72xx and CORE.using_libretiny:
        return cv.declare_id(ArduinoI2CBus)(value)
    elif CORE.is_rtl87xx and CORE.using_libretiny:
        return cv.declare_id(ArduinoI2CBus)(value)
    elif CORE.is_host:
        return cv.declare_id(LinuxI2CBus)(value)
    raise NotImplementedError


def _validate_pin_config(value):
    if CORE.is_host:
        if CONF_SDA in value or CONF_SCL in value:
            raise cv.Invalid("SDA/SCL pins are not used on host platform. Use 'linux_device' instead.")
        if CONF_LINUX_DEVICE not in value:
            value[CONF_LINUX_DEVICE] = "/dev/i2c-1"  # Default device
    else:
        if CONF_SDA not in value or CONF_SCL not in value:
            raise cv.Invalid("SDA and SCL pins are required for non-host platforms")
        if CONF_LINUX_DEVICE in value:
            raise cv.Invalid("linux_device is only valid on host platform")
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): _bus_declare_type,
            cv.Optional(CONF_SDA): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_SCL): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_FREQUENCY, default="50kHz"): cv.All(
                cv.frequency, cv.Range(min=1, max=1000000)
            ),
            cv.Optional(CONF_LINUX_DEVICE): cv.string,
            cv.Optional(CONF_SCAN, default=True): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_pin_config,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CORE.is_host:
        # Linux I2C configuration
        if CONF_LINUX_DEVICE in config:
            cg.add(var.set_linux_device(config[CONF_LINUX_DEVICE]))
    else:
        # Standard GPIO-based I2C configuration
        cg.add_library("Wire", None)
        cg.add(var.set_sda_pin(config[CONF_SDA]))
        cg.add(var.set_scl_pin(config[CONF_SCL]))
        cg.add(var.set_frequency(config[CONF_FREQUENCY]))
        cg.add(var.set_scan(config[CONF_SCAN]))

    # Platform-specific includes
    if CORE.is_esp32:
        if CORE.using_arduino:
            cg.add_library("Wire", None)
        elif CORE.using_esp_idf:
            cg.add_define("USE_ESP_IDF")
    elif CORE.is_esp8266 and CORE.using_arduino:
        cg.add_library("Wire", None)
    elif CORE.is_rp2040 and CORE.using_arduino:
        cg.add_library("Wire", None)
    elif (CORE.is_bk72xx or CORE.is_rtl87xx) and CORE.using_libretiny:
        cg.add_library("Wire", None)
    elif CORE.is_host:
        # No additional libraries needed for Linux I2C
        pass


# I2C Device Schema and Helper Functions
CONF_I2C_ID = "i2c_id"


def i2c_device_schema(default_address):
    """Create a schema for I2C devices."""
    schema = {
        cv.GenerateID(CONF_I2C_ID): cv.use_id(I2CBus),
    }
    if default_address is None:
        schema[cv.Required("address")] = cv.i2c_address
    else:
        schema[cv.Optional("address", default=default_address)] = cv.i2c_address
    return cv.Schema(schema)


async def register_i2c_device(var, config):
    """Register an I2C device with the bus."""
    parent = await cg.get_variable(config[CONF_I2C_ID])
    cg.add(var.set_i2c_bus(parent))
    cg.add(var.set_i2c_address(config["address"]))


# Validation functions
def final_validate_device_schema(
    name: str, *, address_key: str = "address", min_frequency: str = None
):
    """Validate I2C device configuration against bus settings."""
    def validate_frequency(fconf, address, id):
        if min_frequency is None:
            return fconf
        if fconf[CONF_FREQUENCY] < cv.frequency(min_frequency):
            raise cv.Invalid(
                f"Component {name} requires a minimum I2C frequency of {min_frequency} for address {address:#04X}"
            )
        return fconf

    return cv.Schema(
        {
            cv.Required(CONF_I2C_ID): cv.use_variable_id(I2CBus),
            cv.Optional(address_key, default=0x00): cv.i2c_address,
        },
        extra=cv.ALLOW_EXTRA,
    ).add_extra(validate_frequency)

