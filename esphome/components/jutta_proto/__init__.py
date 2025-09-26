import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["uart"]

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


CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(JuraComponent)})
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
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


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    JURA_COMPONENT_IDS.append(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)


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

