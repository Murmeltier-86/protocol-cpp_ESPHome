import os
import re
import xml.etree.ElementTree as ET

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import sensor, text_sensor, uart
from esphome.const import CONF_ID, CONF_NAME

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["uart", "sensor"]

CONF_COFFEE = "coffee"
CONF_GRIND_DURATION = "grind_duration"
CONF_WATER_DURATION = "water_duration"
CONF_PAGE = "page"
CONF_SEQUENCE = "sequence"
CONF_COMMAND = "command"
CONF_RAW = "raw"
CONF_SLEEP = "sleep"
CONF_DELAY = "delay"
CONF_TIMEOUT = "timeout"
CONF_DESCRIPTION = "description"
CONF_MACHINE_DATA = "machine_data"
CONF_MACHINE_SETTINGS = "machine_settings"
CONF_XML = "xml"
CONF_ENABLE_XML_POLL = "enable_xml_poll"
CONF_XML_MAPPING_PATH = "xml_mapping_path"
CONF_XML_POLL_INTERVAL_MS = "xml_poll_interval_ms"
CONF_XML_SENSORS = "xml_sensors"
CONF_FIELD = "field"
CONF_MULTIPLIER = "multiplier"
CONF_OFFSET = "offset"

DEFAULT_XML_POLL_INTERVAL = cv.TimePeriod(milliseconds=60000)

jutta_component_ns = cg.esphome_ns.namespace("jutta_component")
jutta_proto_ns = cg.global_ns.namespace("jutta_proto")

JuraComponent = jutta_component_ns.class_(
    "JuraComponent", cg.Component, uart.UARTDevice
)
CoffeeType = jutta_proto_ns.enum("CoffeeMaker::coffee_t")

StartBrewAction = jutta_component_ns.class_("StartBrewAction", automation.Action)
CustomBrewAction = jutta_component_ns.class_("CustomBrewAction", automation.Action)
CancelCustomBrewAction = jutta_component_ns.class_(
    "CancelCustomBrewAction", automation.Action
)
SwitchPageAction = jutta_component_ns.class_("SwitchPageAction", automation.Action)
RunSequenceAction = jutta_component_ns.class_("RunSequenceAction", automation.Action)
RequestMachineSettingsAction = jutta_component_ns.class_(
    "RequestMachineSettingsAction", automation.Action
)
WriteMachineSettingsAction = jutta_component_ns.class_(
    "WriteMachineSettingsAction", automation.Action
)

COFFEE_TYPES = {
    "espresso": CoffeeType.ESPRESSO,
    "coffee": CoffeeType.COFFEE,
    "cappuccino": CoffeeType.CAPPUCCINO,
    "milk_foam": CoffeeType.MILK_FOAM,
    "hot_water": CoffeeType.HOT_WATER,
    "hotwater": CoffeeType.HOT_WATER,
    "caffe_barista": CoffeeType.CAFFE_BARISTA,
    "lungo_barista": CoffeeType.LUNGO_BARISTA,
    "espresso_doppio": CoffeeType.ESPRESSO_DOPPIO,
    "macchiato": CoffeeType.MACCHIATO,
    "two_espresso": CoffeeType.TWO_ESPRESSO,
    "two_espressi": CoffeeType.TWO_ESPRESSO,
    "two_coffee": CoffeeType.TWO_COFFEE,
    "two_coffees": CoffeeType.TWO_COFFEE,
}

DEFAULT_GRIND_DURATION = cv.TimePeriod(milliseconds=3600)
DEFAULT_WATER_DURATION = cv.TimePeriod(milliseconds=40000)
DEFAULT_COMMAND_TIMEOUT = cv.TimePeriod(milliseconds=5000)


def _slugify(value):
    value = value.lower()
    value = (
        value.replace("ä", "ae")
        .replace("ö", "oe")
        .replace("ü", "ue")
        .replace("ß", "ss")
    )
    value = re.sub(r"[^0-9a-z]+", "_", value)
    return value.strip("_")


def _load_xml_mapping(path):
    try:
        tree = ET.parse(path)
    except (ET.ParseError, OSError) as err:
        raise cv.Invalid(f"Fehler beim Einlesen der XML-Datei '{path}': {err}") from err

    root = tree.getroot()
    namespace = ""
    if root.tag.startswith("{") and "}" in root.tag:
        namespace = root.tag[1 : root.tag.find("}")]
    ns = {"joe": namespace} if namespace else {}

    mapping = {}

    def register(field, command, key, default_name=None):
        field_key = field.lower()
        if field_key not in mapping:
            mapping[field_key] = {
                "command": command,
                "key": key.upper(),
                "default_name": default_name,
                "field": field,
            }

    bank_command = "TR:32"
    total_counter = root.findall(".//joe:TOTALCOUNTER", ns)
    for counter in total_counter:
        code = counter.get("Code")
        name = counter.get("Name", code)
        if code:
            field = f"tr32_{_slugify(name)}"
            register(field, bank_command, code, name)

    for product in root.findall(".//joe:PRODUCT", ns):
        code = product.get("Code")
        name = product.get("Name", code)
        if not code:
            continue
        slug = _slugify(name)
        if slug == "2_espressi":
            aliases = ["two_espresso", "two_espressi"]
        elif slug == "2_coffee":
            aliases = ["two_coffee"]
        else:
            aliases = []
        field = f"tr32_{slug}"
        register(field, bank_command, code, name)
        for alias in aliases:
            register(f"tr32_{alias}", bank_command, code, name)

    for bank in root.findall(".//joe:STATISTIC/joe:MAINTENANCEPAGE/joe:BANK", ns):
        command_attr = bank.get("Command")
        if not command_attr or not command_attr.startswith("@TG:"):
            continue
        command = command_attr[1:]
        code = command.split(":", 1)[-1].lower()
        for item in bank.findall("joe:TEXTITEM", ns):
            text_code = item.get("Text")
            item_type = item.get("Type", text_code or "")
            if not text_code:
                continue
            field = f"tg{code}_{text_code.lower()}"
            register(field, command, text_code, item_type)
            if item_type:
                register(f"tg{code}_{_slugify(item_type)}", command, text_code, item_type)

    return mapping

SEQUENCE_COMMAND_KEYS = {
    "grinder_on": "grinder_on",
    "grinder_off": "grinder_off",
    "brew_group_to_brewing_position": "brew_group_to_brewing_position",
    "brew_group_reset": "brew_group_reset",
    "coffee_press_on": "coffee_press_on",
    "coffee_press_off": "coffee_press_off",
    "water_heater_on": "water_heater_on",
    "water_heater_off": "water_heater_off",
    "water_pump_on": "water_pump_on",
    "water_pump_off": "water_pump_off",
}

SEQUENCE_COMMAND_EXPRESSIONS = {
    "grinder_on": cg.RawExpression("::jutta_proto::JUTTA_GRINDER_ON"),
    "grinder_off": cg.RawExpression("::jutta_proto::JUTTA_GRINDER_OFF"),
    "brew_group_to_brewing_position": cg.RawExpression("::jutta_proto::JUTTA_BREW_GROUP_TO_BREWING_POSITION"),
    "brew_group_reset": cg.RawExpression("::jutta_proto::JUTTA_BREW_GROUP_RESET"),
    "coffee_press_on": cg.RawExpression("::jutta_proto::JUTTA_COFFEE_PRESS_ON"),
    "coffee_press_off": cg.RawExpression("::jutta_proto::JUTTA_COFFEE_PRESS_OFF"),
    "water_heater_on": cg.RawExpression("::jutta_proto::JUTTA_COFFEE_WATER_HEATER_ON"),
    "water_heater_off": cg.RawExpression("::jutta_proto::JUTTA_COFFEE_WATER_HEATER_OFF"),
    "water_pump_on": cg.RawExpression("::jutta_proto::JUTTA_COFFEE_WATER_PUMP_ON"),
    "water_pump_off": cg.RawExpression("::jutta_proto::JUTTA_COFFEE_WATER_PUMP_OFF"),
}

JURA_COMPONENT_IDS = []


XML_SENSOR_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.Required(CONF_FIELD): cv.string,
        cv.Optional(CONF_MULTIPLIER, default=1.0): cv.float_,
        cv.Optional(CONF_OFFSET, default=0.0): cv.float_,
    }
)


def _validate_xml_config(value):
    sensors_conf = value.get(CONF_XML_SENSORS, [])
    if sensors_conf and CONF_XML_MAPPING_PATH not in value:
        raise cv.Invalid("'xml_mapping_path' muss gesetzt sein, wenn 'xml_sensors' verwendet werden")
    if value.get(CONF_ENABLE_XML_POLL) and not sensors_conf:
        raise cv.Invalid("'enable_xml_poll' benötigt mindestens einen Eintrag in 'xml_sensors'")
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(JuraComponent),
            cv.Optional(CONF_MACHINE_DATA): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_MACHINE_SETTINGS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_ENABLE_XML_POLL, default=False): cv.boolean,
            cv.Optional(CONF_XML_MAPPING_PATH): cv.file_,
            cv.Optional(CONF_XML_POLL_INTERVAL_MS, default=DEFAULT_XML_POLL_INTERVAL): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_XML_SENSORS, default=[]): cv.ensure_list(XML_SENSOR_SCHEMA),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    _validate_xml_config,
)


def _normalize_start_brew(value):
    if isinstance(value, str):
        value = {CONF_COFFEE: value}
    return cv.Schema(
        {
            cv.Optional(CONF_ID): cv.use_id(JuraComponent),
            cv.Required(CONF_COFFEE): cv.enum(COFFEE_TYPES, lower=True),
        }
    )(value)


def _normalize_custom_brew(value):
    if isinstance(value, str):
        raise cv.Invalid("Custom brew action requires a dictionary of options")
    return cv.Schema(
        {
            cv.Optional(CONF_ID): cv.use_id(JuraComponent),
            cv.Optional(CONF_GRIND_DURATION, default=DEFAULT_GRIND_DURATION): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_WATER_DURATION, default=DEFAULT_WATER_DURATION): cv.positive_time_period_milliseconds,
        }
    )(value)


def _normalize_cancel(value):
    if value is None:
        value = {}
    if isinstance(value, str):
        value = {CONF_ID: value}
    return cv.Schema({cv.Optional(CONF_ID): cv.use_id(JuraComponent)})(value)


def _normalize_switch_page(value):
    if isinstance(value, int):
        value = {CONF_PAGE: value}
    return cv.Schema(
        {
            cv.Optional(CONF_ID): cv.use_id(JuraComponent),
            cv.Required(CONF_PAGE): cv.int_range(min=0),
        }
    )(value)


def _validate_sequence_step(value):
    if isinstance(value, str):
        value = {CONF_COMMAND: value}

    if not isinstance(value, dict):
        raise cv.Invalid("Sequence step must be a dictionary or string")

    step = value.copy()
    description = step.pop(CONF_DESCRIPTION, "")
    description = cv.string(description)

    if CONF_SLEEP in step or CONF_DELAY in step:
        if CONF_COMMAND in step or CONF_RAW in step:
            raise cv.Invalid("Delay step cannot include a command")
        duration = step.pop(CONF_SLEEP, step.pop(CONF_DELAY, None))
        if duration is None:
            raise cv.Invalid("Delay step requires 'sleep' or 'delay'")
        duration = cv.positive_time_period_milliseconds(duration)
        if step:
            raise cv.Invalid("Unknown keys in delay step: {}".format(", ".join(step.keys())))
        return {
            "type": "delay",
            CONF_DELAY: duration,
            CONF_DESCRIPTION: description,
        }

    if CONF_COMMAND in step or CONF_RAW in step:
        command_value = step.pop(CONF_COMMAND, None)
        raw_value = step.pop(CONF_RAW, None)
        if command_value is not None and raw_value is not None:
            raise cv.Invalid("Specify either 'command' or 'raw'")

        delay = cv.time_period(step.pop(CONF_DELAY, cv.TimePeriod(milliseconds=0)))
        timeout = cv.positive_time_period_milliseconds(step.pop(CONF_TIMEOUT, DEFAULT_COMMAND_TIMEOUT))

        if step:
            raise cv.Invalid("Unknown keys in command step: {}".format(", ".join(step.keys())))

        if command_value is not None:
            command_key = cv.enum(SEQUENCE_COMMAND_KEYS, lower=True)(command_value)
            return {
                "type": "command",
                CONF_COMMAND: command_key,
                CONF_DELAY: delay,
                CONF_TIMEOUT: timeout,
                CONF_DESCRIPTION: description,
            }

        if raw_value is None:
            raise cv.Invalid("Command step requires either 'command' or 'raw'")
        raw_value = cv.string(raw_value)
        return {
            "type": "command",
            CONF_RAW: raw_value,
            CONF_DELAY: delay,
            CONF_TIMEOUT: timeout,
            CONF_DESCRIPTION: description,
        }

    raise cv.Invalid("Sequence step must define a command/raw or delay/sleep")


def _normalize_sequence(value):
    if isinstance(value, list):
        value = {CONF_SEQUENCE: value}
    elif isinstance(value, dict):
        base = {}
        if CONF_ID in value:
            base[CONF_ID] = value[CONF_ID]
        if CONF_SEQUENCE in value:
            base[CONF_SEQUENCE] = value[CONF_SEQUENCE]
        else:
            step = value.copy()
            step.pop(CONF_ID, None)
            base[CONF_SEQUENCE] = [step]
        value = base
    else:
        raise cv.Invalid("Sequence action requires a list of steps")

    return cv.Schema(
        {
            cv.Optional(CONF_ID): cv.use_id(JuraComponent),
            cv.Required(CONF_SEQUENCE): cv.All(cv.ensure_list(_validate_sequence_step)),
        }
    )(value)


def _normalize_request_machine_settings(value):
    if value is None:
        value = {}
    if isinstance(value, str):
        value = {CONF_ID: value}
    return cv.Schema({cv.Optional(CONF_ID): cv.use_id(JuraComponent)})(value)


def _normalize_write_machine_settings(value):
    if isinstance(value, str):
        value = {CONF_XML: value}
    return cv.Schema(
        {
            cv.Optional(CONF_ID): cv.use_id(JuraComponent),
            cv.Required(CONF_XML): cv.string,
        }
    )(value)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    JURA_COMPONENT_IDS.append(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_MACHINE_DATA in config:
        sensor = await text_sensor.new_text_sensor(config[CONF_MACHINE_DATA])
        cg.add(var.set_machine_data_sensor(sensor))

    if CONF_MACHINE_SETTINGS in config:
        sensor = await text_sensor.new_text_sensor(config[CONF_MACHINE_SETTINGS])
        cg.add(var.set_machine_settings_sensor(sensor))

    xml_sensors = config.get(CONF_XML_SENSORS, [])

    if config.get(CONF_ENABLE_XML_POLL):
        cg.add(var.set_xml_poll_enabled(True))

    if CONF_XML_POLL_INTERVAL_MS in config:
        interval = config[CONF_XML_POLL_INTERVAL_MS]
        cg.add(var.set_xml_poll_interval(interval.total_milliseconds))

    mapping = {}
    if xml_sensors:
        mapping_path = config[CONF_XML_MAPPING_PATH]
        mapping_path = os.path.expanduser(mapping_path)
        if not os.path.isabs(mapping_path):
            mapping_path = os.path.abspath(mapping_path)
        mapping = _load_xml_mapping(mapping_path)

    for sensor_conf in xml_sensors:
        sensor_conf = sensor_conf.copy()
        field = sensor_conf.pop(CONF_FIELD)
        multiplier = sensor_conf.pop(CONF_MULTIPLIER)
        offset = sensor_conf.pop(CONF_OFFSET)

        info = mapping.get(field.lower())
        if info is None:
            raise cv.Invalid(f"Unbekanntes XML-Feld '{field}'. Bitte prüfe die Mapping-Datei")

        if CONF_NAME not in sensor_conf and info.get("default_name"):
            sensor_conf[CONF_NAME] = info["default_name"]

        sens = await sensor.new_sensor(sensor_conf)
        cg.add(
            var.register_xml_sensor(
                cg.std_string(info["field"]),
                cg.std_string(info["command"]),
                cg.std_string(info["key"]),
                multiplier,
                offset,
                sens,
            )
        )


async def _get_parent(config):
    if CONF_ID in config:
        return await cg.get_variable(config[CONF_ID])
    if not JURA_COMPONENT_IDS:
        raise cv.Invalid("No jutta_proto component configured")
    if len(JURA_COMPONENT_IDS) > 1:
        raise cv.Invalid("Multiple jutta_proto components configured, please set 'id'")
    return await cg.get_variable(JURA_COMPONENT_IDS[0])


@automation.register_action("jutta_proto.start_brew", StartBrewAction, _normalize_start_brew)
async def start_brew_action_to_code(config, action_id, template_args, args):
    _ = args
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_coffee(config[CONF_COFFEE]))
    return var


@automation.register_action("jutta_proto.custom_brew", CustomBrewAction, _normalize_custom_brew)
async def custom_brew_action_to_code(config, action_id, template_args, args):
    _ = args
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    grind = config[CONF_GRIND_DURATION]
    water = config[CONF_WATER_DURATION]
    cg.add(var.set_grind_duration(grind.total_milliseconds))
    cg.add(var.set_water_duration(water.total_milliseconds))
    return var


@automation.register_action("jutta_proto.cancel_custom_brew", CancelCustomBrewAction, _normalize_cancel)
async def cancel_brew_action_to_code(config, action_id, template_args, args):
    _ = args
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    return var


@automation.register_action("jutta_proto.switch_page", SwitchPageAction, _normalize_switch_page)
async def switch_page_action_to_code(config, action_id, template_args, args):
    _ = args
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_page(config[CONF_PAGE]))
    return var


@automation.register_action("jutta_proto.run_sequence", RunSequenceAction, _normalize_sequence)
async def run_sequence_action_to_code(config, action_id, template_args, args):
    _ = args
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)

    for step in config[CONF_SEQUENCE]:
        description = cg.std_string(step.get(CONF_DESCRIPTION, ""))
        if step["type"] == "command":
            delay_ms = step[CONF_DELAY].total_milliseconds
            timeout_ms = step[CONF_TIMEOUT].total_milliseconds
            if CONF_COMMAND in step:
                command_expr = SEQUENCE_COMMAND_EXPRESSIONS[step[CONF_COMMAND]]
                cg.add(var.add_command_step(command_expr, delay_ms, timeout_ms, description))
            else:
                raw_command = step[CONF_RAW]
                if not raw_command.endswith("\r\n"):
                    raw_command = f"{raw_command}\r\n"
                cg.add(var.add_command_step(cg.std_string(raw_command), delay_ms, timeout_ms, description))
        else:
            delay_ms = step[CONF_DELAY].total_milliseconds
            cg.add(var.add_delay_step(delay_ms, description))

    return var


@automation.register_action(
    "jutta_proto.request_machine_settings", RequestMachineSettingsAction, _normalize_request_machine_settings
)
async def request_machine_settings_action_to_code(config, action_id, template_args, args):
    _ = args
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    return var


@automation.register_action(
    "jutta_proto.write_machine_settings", WriteMachineSettingsAction, _normalize_write_machine_settings
)
async def write_machine_settings_action_to_code(config, action_id, template_args, args):
    _ = args
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_xml(cg.std_string(config[CONF_XML])))
    return var

