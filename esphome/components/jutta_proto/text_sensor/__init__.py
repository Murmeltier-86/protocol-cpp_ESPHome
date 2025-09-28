import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from .. import JURA_COMPONENT_IDS, JuraComponent

CONF_JUTTA_PROTO_ID = "jutta_proto_id"


CONFIG_SCHEMA = text_sensor.text_sensor_schema().extend(
    {cv.Optional(CONF_JUTTA_PROTO_ID): cv.use_id(JuraComponent)}
)


async def to_code(config):
    parent_id = config.get(CONF_JUTTA_PROTO_ID)

    if parent_id is not None:
        parent = await cg.get_variable(parent_id)
    else:
        if not JURA_COMPONENT_IDS:
            raise cv.Invalid("No jutta_proto component configured")
        if len(JURA_COMPONENT_IDS) > 1:
            raise cv.Invalid(
                "Multiple jutta_proto components configured, please set 'jutta_proto_id'"
            )
        parent = await cg.get_variable(JURA_COMPONENT_IDS[0])

    sensor_config = dict(config)
    sensor_config.pop(CONF_JUTTA_PROTO_ID, None)
    sensor = await text_sensor.new_text_sensor(sensor_config)
    cg.add(parent.set_machine_data_sensor(sensor))
