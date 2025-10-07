import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from .. import JURA_COMPONENT_IDS, JuraComponent


def _setting_id_from_text_id(id_obj):
    if id_obj is None:
        return None
    name = id_obj.id
    prefix = "jura_setting_"
    if name.startswith(prefix):
        return name[len(prefix) :]
    return None


CONF_JUTTA_ID = "jutta_id"


CONFIG_SCHEMA = text_sensor.text_sensor_schema().extend(
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
    sens = await text_sensor.new_text_sensor(config)

    id_obj = config.get(cv.CONF_ID)
    if id_obj is None:
        raise cv.Invalid("Text-Sensor benötigt ein id-Attribut")
    identifier = id_obj.id
    setting_id = _setting_id_from_text_id(id_obj)

    if setting_id is not None:
        cg.add(parent.register_setting_text_sensor(cg.std_string(setting_id), sens))
        return

    if identifier == "jura_error_text":
        cg.add(parent.set_error_text_sensor(sens))
        return

    if identifier == "jura_error_severity":
        cg.add(parent.set_error_severity_sensor(sens))
        return

    raise cv.Invalid(
        f"Unbekannter jutta_proto Text-Sensor '{identifier}'. Erwartet Fehlertext/-severity oder jura_setting_*."
    )

