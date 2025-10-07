import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from .. import JURA_COMPONENT_IDS, JuraComponent

CONF_JUTTA_ID = "jutta_id"


CONFIG_SCHEMA = binary_sensor.binary_sensor_schema().extend(
    {cv.Optional(CONF_JUTTA_ID): cv.use_id(JuraComponent)}
)


async def _resolve_parent(config):
    if CONF_JUTTA_ID in config:
        return await cg.get_variable(config[CONF_JUTTA_ID])
    if not JURA_COMPONENT_IDS:
        raise cv.Invalid("Es ist kein jutta_proto-Component konfiguriert")
    if len(JURA_COMPONENT_IDS) > 1:
        raise cv.Invalid("Mehrere jutta_proto-Instanzen gefunden, bitte 'jutta_id' setzen")
    return await cg.get_variable(JURA_COMPONENT_IDS[0])


async def to_code(config):
    parent = await _resolve_parent(config)
    sens = await binary_sensor.new_binary_sensor(config)

    id_obj = config.get(cv.CONF_ID)
    if id_obj is None:
        raise cv.Invalid("Binary Sensor benötigt ein id-Attribut")
    identifier = id_obj.id

    if identifier == "jura_has_error":
        cg.add(parent.set_error_active_sensor(sens))
        return

    raise cv.Invalid("Unbekannter jutta_proto Binary Sensor '{}'.".format(identifier))

