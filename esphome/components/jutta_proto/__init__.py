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

jutta_component_ns = cg.esphome_ns.namespace("esphome").namespace("jutta_proto")
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

COFFEE_TYPES = {
    "espresso": CoffeeType.ESPRESSO,
    "coffee": CoffeeType.COFFEE,
    "cappuccino": CoffeeType.CAPPUCCINO,
    "milk_foam": CoffeeType.MILK_FOAM,
    "caffe_barista": CoffeeType.CAFFE_BARISTA,
    "lungo_barista": CoffeeType.LUNGO_BARISTA,
    "espresso_doppio": CoffeeType.ESPRESSO_DOPPIO,
    "macchiato": CoffeeType.MACCHIATO,
}

DEFAULT_GRIND_DURATION = cv.TimePeriod(milliseconds=3600)
DEFAULT_WATER_DURATION = cv.TimePeriod(milliseconds=40000)

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
async def start_brew_action_to_code(config, action_id, template_args):
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_coffee(config[CONF_COFFEE]))
    return var


@automation.register_action("jutta_proto.custom_brew", CustomBrewAction, _normalize_custom_brew)
async def custom_brew_action_to_code(config, action_id, template_args):
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    grind = config[CONF_GRIND_DURATION]
    water = config[CONF_WATER_DURATION]
    cg.add(var.set_grind_duration(grind.total_milliseconds))
    cg.add(var.set_water_duration(water.total_milliseconds))
    return var


@automation.register_action("jutta_proto.cancel_custom_brew", CancelCustomBrewAction, _normalize_cancel)
async def cancel_brew_action_to_code(config, action_id, template_args):
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    return var


@automation.register_action("jutta_proto.switch_page", SwitchPageAction, _normalize_switch_page)
async def switch_page_action_to_code(config, action_id, template_args):
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_page(config[CONF_PAGE]))
    return var

