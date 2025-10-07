import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

from .. import JURA_COMPONENT_IDS, JuraComponent


def _setting_id_from_sensor_id(id_obj):
    if id_obj is None:
        return None
    name = id_obj.id
    prefix = "jura_setting_"
    if name.startswith(prefix):
        return name[len(prefix) :]
    return None


CONF_JUTTA_ID = "jutta_id"


CONFIG_SCHEMA = sensor.sensor_schema().extend(
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
    sens = await sensor.new_sensor(config)

    id_obj = config.get(cv.CONF_ID)
    if id_obj is None:
        raise cv.Invalid("Sensor requires an id")
    identifier = id_obj.id
    setting_id = _setting_id_from_sensor_id(id_obj)

    if setting_id is not None:
        cg.add(parent.register_setting_sensor(cg.std_string(setting_id), sens))
        return

    if identifier == "jura_error_code":
        cg.add(parent.set_error_code_sensor(sens))
        return

    raise cv.Invalid(
        f"Unbekannter jutta_proto Sensor '{identifier}'. Erwartet 'jura_error_code' oder 'jura_setting_*'."
    )

