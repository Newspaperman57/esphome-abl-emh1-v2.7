import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_EMPTY,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    ICON_COUNTER,
    ICON_EMPTY,
    ICON_TIMER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_EMPTY,
    UNIT_WATT,
)

from . import CONF_ABL_EMH1_ID, ABLeMH1

DEPENDENCIES = ["abl_emh1"]

CONF_MODE = "mode"
CONF_CURRENT = "current"
CONF_MAX_CURRENT = "max_current"
CONF_CHARGING_ENABLED = "charging_enabled"

ICON_MODE = "mdi:information"

SENSORS = [
    CONF_MODE,
	CONF_CURRENT,
    CONF_MAX_CURRENT,
    CONF_CHARGING_ENABLED,
]

# pylint: disable=too-many-function-args
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ABL_EMH1_ID): cv.use_id(ABLeMH1),
        cv.Optional(CONF_MODE): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_MODE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_MAX_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CHARGING_ENABLED): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_MODE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_ABL_EMH1_ID])
    for key in SENSORS:
        if key in config:
            conf = config[key]
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))
