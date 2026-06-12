import os

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import binary_sensor, sensor, text_sensor, uart
from esphome.const import CONF_ID
from esphome.core import CORE

COMPONENT_DIR = os.path.dirname(__file__)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["uart", "sensor", "text_sensor", "binary_sensor"]

CONF_COFFEE = "coffee"
CONF_GRIND_DURATION = "grind_duration"
CONF_WATER_DURATION = "water_duration"
CONF_PAGE = "page"
CONF_SEQUENCE = "sequence"
CONF_COMMAND = "command"
CONF_PROBE = "probe"
CONF_RAW = "raw"
CONF_SLEEP = "sleep"
CONF_DELAY = "delay"
CONF_TIMEOUT = "timeout"
CONF_DESCRIPTION = "description"
CONF_MACHINE_DATA = "machine_data"
CONF_RAW_RX = "raw_rx"
CONF_LAST_COMMAND_RESULT = "last_command_result"
CONF_MACHINE_TYPE = "machine_type"
CONF_MACHINE_STATUS = "machine_status"
CONF_MACHINE_DISPLAY_STATUS = "machine_display_status"
CONF_MACHINE_WARNING = "machine_warning"
CONF_ACTIVE_ALERTS = "active_alerts"
CONF_STATUS_PROBE_LAST_RESPONSE = "status_probe_last_response"
CONF_LAST_T2_STATUS_RAW = "last_t2_status_raw"
CONF_LAST_T2_STATUS_DECODED = "last_t2_status_decoded"
CONF_MACHINE_ONLINE = "machine_online"
CONF_MACHINE_READY = "machine_ready"
CONF_FILL_WATER_REQUIRED = "fill_water_required"
CONF_TF_WELCOME = "tf_welcome"
CONF_TF_COFFEE_READY = "tf_coffee_ready"
CONF_TF_ENERGY_SAFE = "tf_energy_safe"
CONF_TF_ACTIVE_RF_FILTER = "tf_active_rf_filter"
CONF_TF_STATUS_BITS = "tf_status_bits"
CONF_ENABLE_MACHINE_XML_POLL = "enable_machine_xml_poll"
CONF_ENABLE_XML_POLL = "enable_xml_poll"
CONF_XML_MAPPING_PATH = "xml_mapping_path"
CONF_XML_POLL_INTERVAL_MS = "xml_poll_interval_ms"
CONF_XML_STARTUP_DELAY_MS = "xml_startup_delay_ms"
CONF_XML_PUBLISH_UNSTABLE = "xml_publish_unstable"
CONF_XML_WAIT_FOR_TS_ACK = "xml_wait_for_ts_ack"
CONF_XML_STATS_USE_TS_LOCK = "xml_stats_use_ts_lock"
CONF_XML_DEBUG_COMPACT = "xml_debug_compact"
CONF_XML_DECODE_INNER_TRANSPORT = "xml_decode_inner_transport"
CONF_XML_INNER_DECODE_TRACE = "xml_inner_decode_trace"
CONF_XML_BINARY_PROBE = "xml_binary_probe"
CONF_XML_KEY_PROBE = "xml_key_probe"
CONF_XML_DEEP_DEBUG = "xml_deep_debug"
CONF_XML_TRANSPORT_SELFTEST = "xml_transport_selftest"
CONF_XML_COMMAND_PROBE = "xml_command_probe"
CONF_XML_COMMAND_PROBE_WITH_TS_LOCK = "xml_command_probe_with_ts_lock"
CONF_XML_SESSION_PROBE = "xml_session_probe"
CONF_XML_SESSION_PROBE_VARIANT = "xml_session_probe_variant"
CONF_XML_DONGLE_STARTUP = "xml_dongle_startup"
CONF_XML_DONGLE_STARTUP_DEBUG = "xml_dongle_startup_debug"
CONF_XML_DONGLE_STARTUP_MODE = "xml_dongle_startup_mode"
CONF_XML_DONGLE_WAIT_T0_AFTER_T3 = "xml_dongle_wait_t0_after_t3"
CONF_XML_DONGLE_INNER_TX_DEBUG = "xml_dongle_inner_tx_debug"
CONF_XML_RUN_TABLET_START_SEQUENCE = "xml_run_tablet_start_sequence"
CONF_XML_TABLET_SEQUENCE_MODE = "xml_tablet_sequence_mode"
CONF_XML_COUNTER_MAX = "xml_counter_max"
CONF_STATUS_DEBUG = "status_debug"
CONF_STATUS_PROBE_ENABLED = "status_probe_enabled"
CONF_STATUS_PROBE_INTERVAL_MS = "status_probe_interval_ms"
CONF_XML_SENSORS = "xml_sensors"
CONF_FIELD = "field"
CONF_LOG_DECODED_TX = "log_decoded_tx"
CONF_LOG_ENCODED_UART = "log_encoded_uart"

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
ManualStatusProbeAction = jutta_component_ns.class_(
    "ManualStatusProbeAction", automation.Action
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

STATUS_PROBE_COMMANDS = {
    "hf": "@hf",
    "ha_0": "@ha:03,20",
    "ha_1": "@ha:03,21",
    "ha_2": "@ha:03,22",
    "ha_3": "@ha:03,23",
}

JURA_COMPONENT_IDS = []


XML_SENSOR_SCHEMA = sensor.sensor_schema().extend({cv.Required(CONF_FIELD): cv.string})


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(JuraComponent),
            cv.Optional(CONF_MACHINE_DATA): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_RAW_RX): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_LAST_COMMAND_RESULT): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_MACHINE_TYPE): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_MACHINE_STATUS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_MACHINE_DISPLAY_STATUS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_MACHINE_WARNING): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_ACTIVE_ALERTS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_STATUS_PROBE_LAST_RESPONSE): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_LAST_T2_STATUS_RAW): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_LAST_T2_STATUS_DECODED): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_MACHINE_ONLINE): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_MACHINE_READY): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_FILL_WATER_REQUIRED): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_TF_WELCOME): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_TF_COFFEE_READY): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_TF_ENERGY_SAFE): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_TF_ACTIVE_RF_FILTER): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_TF_STATUS_BITS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_ENABLE_MACHINE_XML_POLL): cv.boolean,
            cv.Optional(CONF_ENABLE_XML_POLL, default=False): cv.boolean,
            cv.Optional(CONF_XML_MAPPING_PATH, default="embedded"): cv.string,
            cv.Optional(CONF_XML_POLL_INTERVAL_MS, default=30000): cv.All(
                cv.positive_int, cv.Range(min=25000)
            ),
            cv.Optional(CONF_XML_STARTUP_DELAY_MS, default=10000): cv.All(
                cv.positive_int, cv.Range(min=10000)
            ),
            cv.Optional(CONF_XML_PUBLISH_UNSTABLE, default=False): cv.boolean,
            cv.Optional(CONF_XML_WAIT_FOR_TS_ACK, default=False): cv.boolean,
            cv.Optional(CONF_XML_STATS_USE_TS_LOCK, default=False): cv.boolean,
            cv.Optional(CONF_XML_DEBUG_COMPACT, default=True): cv.boolean,
            cv.Optional(CONF_XML_DECODE_INNER_TRANSPORT, default=True): cv.boolean,
            cv.Optional(CONF_XML_INNER_DECODE_TRACE, default=False): cv.boolean,
            cv.Optional(CONF_XML_BINARY_PROBE, default=False): cv.boolean,
            cv.Optional(CONF_XML_KEY_PROBE, default=False): cv.boolean,
            cv.Optional(CONF_XML_DEEP_DEBUG, default=False): cv.boolean,
            cv.Optional(CONF_XML_TRANSPORT_SELFTEST, default=False): cv.boolean,
            cv.Optional(CONF_XML_COMMAND_PROBE, default=False): cv.boolean,
            cv.Optional(CONF_XML_COMMAND_PROBE_WITH_TS_LOCK, default=True): cv.boolean,
            cv.Optional(CONF_XML_SESSION_PROBE, default=False): cv.boolean,
            cv.Optional(CONF_XML_SESSION_PROBE_VARIANT, default="minimal"): cv.one_of(
                "minimal", "dongle_full", "no_d1", lower=True
            ),
            cv.Optional(CONF_XML_DONGLE_STARTUP, default=False): cv.boolean,
            cv.Optional(CONF_XML_DONGLE_STARTUP_DEBUG, default=False): cv.boolean,
            cv.Optional(CONF_XML_DONGLE_STARTUP_MODE, default="full"): cv.one_of(
                "full", "gate_only", lower=True
            ),
            cv.Optional(CONF_XML_DONGLE_WAIT_T0_AFTER_T3, default=False): cv.boolean,
            cv.Optional(CONF_XML_DONGLE_INNER_TX_DEBUG, default=False): cv.boolean,
            cv.Optional(CONF_XML_RUN_TABLET_START_SEQUENCE, default=False): cv.boolean,
            cv.Optional(CONF_XML_TABLET_SEQUENCE_MODE, default="minimal"): cv.one_of(
                "minimal", "minimal_tr37", lower=True
            ),
            cv.Optional(CONF_XML_COUNTER_MAX, default=20000): cv.positive_int,
            cv.Optional(CONF_STATUS_DEBUG, default=False): cv.boolean,
            cv.Optional(CONF_STATUS_PROBE_ENABLED, default=False): cv.boolean,
            cv.Optional(CONF_STATUS_PROBE_INTERVAL_MS, default=300000): cv.All(
                cv.positive_int, cv.Range(min=10000)
            ),
            cv.Optional(CONF_XML_SENSORS, default=[]): cv.ensure_list(XML_SENSOR_SCHEMA),
            cv.Optional(CONF_LOG_DECODED_TX, default=True): cv.boolean,
            cv.Optional(CONF_LOG_ENCODED_UART, default=False): cv.boolean,
        }
    )
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


def _normalize_status_probe(value):
    if isinstance(value, str):
        value = {CONF_PROBE: value}
    if value is None:
        value = {}
    return cv.Schema(
        {
            cv.Optional(CONF_ID): cv.use_id(JuraComponent),
            cv.Required(CONF_PROBE): cv.enum(STATUS_PROBE_COMMANDS, lower=True),
        }
    )(value)


def _make_raw_string_literal(text):
    delimiter = "JUTTA_XML"
    while f"){delimiter}\"" in text:
        delimiter += "_X"
    return 'R"' + delimiter + '(' + text + ')' + delimiter + '"'


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    JURA_COMPONENT_IDS.append(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_log_decoded_tx(config[CONF_LOG_DECODED_TX]))
    cg.add(var.set_log_encoded_uart(config[CONF_LOG_ENCODED_UART]))
    cg.add(var.set_enable_xml_poll(config[CONF_ENABLE_XML_POLL]))
    machine_xml_default = not config[CONF_ENABLE_XML_POLL]
    cg.add(var.set_enable_machine_xml_poll(config.get(CONF_ENABLE_MACHINE_XML_POLL, machine_xml_default)))
    cg.add(var.set_xml_publish_unstable(config[CONF_XML_PUBLISH_UNSTABLE]))
    cg.add(var.set_xml_wait_for_ts_ack(config[CONF_XML_WAIT_FOR_TS_ACK]))
    cg.add(var.set_xml_stats_use_ts_lock(config[CONF_XML_STATS_USE_TS_LOCK]))
    cg.add(var.set_xml_debug_compact(config[CONF_XML_DEBUG_COMPACT]))
    cg.add(var.set_xml_decode_inner_transport(config[CONF_XML_DECODE_INNER_TRANSPORT]))
    cg.add(var.set_xml_inner_decode_trace(config[CONF_XML_INNER_DECODE_TRACE]))
    cg.add(var.set_xml_binary_probe(config[CONF_XML_BINARY_PROBE]))
    cg.add(var.set_xml_key_probe(config[CONF_XML_KEY_PROBE]))
    cg.add(var.set_xml_deep_debug(config[CONF_XML_DEEP_DEBUG]))
    cg.add(var.set_xml_transport_selftest(config[CONF_XML_TRANSPORT_SELFTEST]))
    cg.add(var.set_xml_command_probe(config[CONF_XML_COMMAND_PROBE]))
    cg.add(var.set_xml_command_probe_with_ts_lock(config[CONF_XML_COMMAND_PROBE_WITH_TS_LOCK]))
    cg.add(var.set_xml_session_probe(config[CONF_XML_SESSION_PROBE]))
    cg.add(var.set_xml_session_probe_variant(config[CONF_XML_SESSION_PROBE_VARIANT]))
    cg.add(var.set_xml_dongle_startup(config[CONF_XML_DONGLE_STARTUP]))
    cg.add(var.set_xml_dongle_startup_debug(config[CONF_XML_DONGLE_STARTUP_DEBUG]))
    cg.add(var.set_xml_dongle_startup_mode(config[CONF_XML_DONGLE_STARTUP_MODE]))
    cg.add(var.set_xml_dongle_wait_t0_after_t3(config[CONF_XML_DONGLE_WAIT_T0_AFTER_T3]))
    cg.add(var.set_xml_dongle_inner_tx_debug(config[CONF_XML_DONGLE_INNER_TX_DEBUG]))
    cg.add(var.set_xml_run_tablet_start_sequence(config[CONF_XML_RUN_TABLET_START_SEQUENCE]))
    cg.add(var.set_xml_tablet_sequence_mode(config[CONF_XML_TABLET_SEQUENCE_MODE]))
    cg.add(var.set_xml_counter_max(config[CONF_XML_COUNTER_MAX]))
    cg.add(var.set_status_debug(config[CONF_STATUS_DEBUG]))
    cg.add(var.set_status_probe_enabled(config[CONF_STATUS_PROBE_ENABLED]))
    cg.add(var.set_status_probe_interval(config[CONF_STATUS_PROBE_INTERVAL_MS]))
    mapping_path = config[CONF_XML_MAPPING_PATH]
    if mapping_path == "embedded":
        resolved_path = os.path.join(COMPONENT_DIR, "jura_mapping_embed.xml")
    else:
        resolved_path = mapping_path
        if not os.path.isabs(resolved_path):
            resolved_path = os.path.join(CORE.relative_config_path, resolved_path)

    if not os.path.exists(resolved_path):
        raise cv.Invalid(
            "Die angegebene XML-Datei '{}' (aufgelöst zu '{}') wurde nicht gefunden".format(
                mapping_path, resolved_path
            )
        )

    try:
        with open(resolved_path, "r", encoding="utf-8") as xml_file:
            xml_content = xml_file.read()
    except OSError as err:
        raise cv.Invalid(
            "XML-Datei '{}' konnte nicht gelesen werden: {}".format(
                resolved_path, err
            )
        ) from err

    cg.add(var.set_xml_mapping_path(cg.std_string(resolved_path)))

    component_index = len(JURA_COMPONENT_IDS)
    symbol_base = f"jutta_proto_xml_blob_{component_index}"
    raw_literal = _make_raw_string_literal(xml_content)
    cg.add_global(
        cg.RawExpression(
            "namespace esphome {\nnamespace jutta_component {\n"
            f"constexpr char {symbol_base}[] = {raw_literal};\n"
            f"constexpr size_t {symbol_base}_len = sizeof({symbol_base}) - 1;\n"
            "}\n}\n"
        )
    )
    cg.add(
        var.set_xml_mapping_source(
            cg.RawExpression(f"::esphome::jutta_component::{symbol_base}"),
            cg.RawExpression(f"::esphome::jutta_component::{symbol_base}_len")
        )
    )
    cg.add(var.set_xml_poll_interval(config[CONF_XML_POLL_INTERVAL_MS]))
    cg.add(var.set_xml_startup_delay(config[CONF_XML_STARTUP_DELAY_MS]))

    if CONF_MACHINE_DATA in config:
        machine_sensor = await text_sensor.new_text_sensor(config[CONF_MACHINE_DATA])
        cg.add(var.set_machine_data_sensor(machine_sensor))

    if CONF_RAW_RX in config:
        raw_rx_sensor = await text_sensor.new_text_sensor(config[CONF_RAW_RX])
        cg.add(var.set_raw_rx_sensor(raw_rx_sensor))

    if CONF_LAST_COMMAND_RESULT in config:
        result_sensor = await text_sensor.new_text_sensor(config[CONF_LAST_COMMAND_RESULT])
        cg.add(var.set_last_command_result_sensor(result_sensor))

    if CONF_MACHINE_TYPE in config:
        machine_type_sensor = await text_sensor.new_text_sensor(config[CONF_MACHINE_TYPE])
        cg.add(var.set_machine_type_sensor(machine_type_sensor))

    if CONF_MACHINE_STATUS in config:
        machine_status_sensor = await text_sensor.new_text_sensor(config[CONF_MACHINE_STATUS])
        cg.add(var.set_machine_status_sensor(machine_status_sensor))

    if CONF_MACHINE_DISPLAY_STATUS in config:
        machine_display_status_sensor = await text_sensor.new_text_sensor(config[CONF_MACHINE_DISPLAY_STATUS])
        cg.add(var.set_machine_display_status_sensor(machine_display_status_sensor))

    if CONF_MACHINE_WARNING in config:
        machine_warning_sensor = await text_sensor.new_text_sensor(config[CONF_MACHINE_WARNING])
        cg.add(var.set_machine_warning_sensor(machine_warning_sensor))

    if CONF_ACTIVE_ALERTS in config:
        active_alerts_sensor = await text_sensor.new_text_sensor(config[CONF_ACTIVE_ALERTS])
        cg.add(var.set_active_alerts_sensor(active_alerts_sensor))

    if CONF_STATUS_PROBE_LAST_RESPONSE in config:
        status_probe_sensor = await text_sensor.new_text_sensor(config[CONF_STATUS_PROBE_LAST_RESPONSE])
        cg.add(var.set_status_probe_last_response_sensor(status_probe_sensor))

    if CONF_LAST_T2_STATUS_RAW in config:
        last_t2_raw_sensor = await text_sensor.new_text_sensor(config[CONF_LAST_T2_STATUS_RAW])
        cg.add(var.set_last_t2_status_raw_sensor(last_t2_raw_sensor))

    if CONF_LAST_T2_STATUS_DECODED in config:
        last_t2_decoded_sensor = await text_sensor.new_text_sensor(config[CONF_LAST_T2_STATUS_DECODED])
        cg.add(var.set_last_t2_status_decoded_sensor(last_t2_decoded_sensor))

    if CONF_MACHINE_ONLINE in config:
        machine_online_sensor = await binary_sensor.new_binary_sensor(config[CONF_MACHINE_ONLINE])
        cg.add(var.set_machine_online_sensor(machine_online_sensor))

    if CONF_MACHINE_READY in config:
        machine_ready_sensor = await binary_sensor.new_binary_sensor(config[CONF_MACHINE_READY])
        cg.add(var.set_machine_ready_sensor(machine_ready_sensor))

    if CONF_FILL_WATER_REQUIRED in config:
        fill_water_required_sensor = await binary_sensor.new_binary_sensor(config[CONF_FILL_WATER_REQUIRED])
        cg.add(var.set_fill_water_required_sensor(fill_water_required_sensor))

    if CONF_TF_WELCOME in config:
        tf_welcome_sensor = await binary_sensor.new_binary_sensor(config[CONF_TF_WELCOME])
        cg.add(var.set_tf_welcome_sensor(tf_welcome_sensor))

    if CONF_TF_COFFEE_READY in config:
        tf_coffee_ready_sensor = await binary_sensor.new_binary_sensor(config[CONF_TF_COFFEE_READY])
        cg.add(var.set_tf_coffee_ready_sensor(tf_coffee_ready_sensor))

    if CONF_TF_ENERGY_SAFE in config:
        tf_energy_safe_sensor = await binary_sensor.new_binary_sensor(config[CONF_TF_ENERGY_SAFE])
        cg.add(var.set_tf_energy_safe_sensor(tf_energy_safe_sensor))

    if CONF_TF_ACTIVE_RF_FILTER in config:
        tf_active_rf_filter_sensor = await binary_sensor.new_binary_sensor(config[CONF_TF_ACTIVE_RF_FILTER])
        cg.add(var.set_tf_active_rf_filter_sensor(tf_active_rf_filter_sensor))

    if CONF_TF_STATUS_BITS in config:
        tf_status_bits_sensor = await text_sensor.new_text_sensor(config[CONF_TF_STATUS_BITS])
        cg.add(var.set_tf_status_bits_sensor(tf_status_bits_sensor))

    for xml_sensor in config.get(CONF_XML_SENSORS, []):
        sensor_conf = xml_sensor.copy()
        field_name = sensor_conf.pop(CONF_FIELD)
        sens = await sensor.new_sensor(sensor_conf)
        cg.add(var.add_configured_xml_sensor(cg.std_string(field_name), sens))


async def _get_parent(config):
    if CONF_ID in config:
        return await cg.get_variable(config[CONF_ID])
    if not JURA_COMPONENT_IDS:
        raise cv.Invalid("No jutta_proto component configured")
    if len(JURA_COMPONENT_IDS) > 1:
        raise cv.Invalid("Multiple jutta_proto components configured, please set 'id'")
    return await cg.get_variable(JURA_COMPONENT_IDS[0])


@automation.register_action("jutta_proto.start_brew", StartBrewAction, _normalize_start_brew, synchronous=False)
async def start_brew_action_to_code(config, action_id, template_args, args):
    _ = args
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_coffee(config[CONF_COFFEE]))
    return var


@automation.register_action("jutta_proto.custom_brew", CustomBrewAction, _normalize_custom_brew, synchronous=False)
async def custom_brew_action_to_code(config, action_id, template_args, args):
    _ = args
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    grind = config[CONF_GRIND_DURATION]
    water = config[CONF_WATER_DURATION]
    cg.add(var.set_grind_duration(grind.total_milliseconds))
    cg.add(var.set_water_duration(water.total_milliseconds))
    return var


@automation.register_action("jutta_proto.cancel_custom_brew", CancelCustomBrewAction, _normalize_cancel, synchronous=False)
async def cancel_brew_action_to_code(config, action_id, template_args, args):
    _ = args
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    return var


@automation.register_action("jutta_proto.switch_page", SwitchPageAction, _normalize_switch_page, synchronous=False)
async def switch_page_action_to_code(config, action_id, template_args, args):
    _ = args
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_page(config[CONF_PAGE]))
    return var


@automation.register_action("jutta_proto.run_sequence", RunSequenceAction, _normalize_sequence, synchronous=False)
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


@automation.register_action("jutta_proto.status_probe", ManualStatusProbeAction, _normalize_status_probe, synchronous=False)
async def status_probe_action_to_code(config, action_id, template_args, args):
    _ = args
    parent = await _get_parent(config)
    var = cg.new_Pvariable(action_id, parent)
    cg.add(var.set_command(cg.std_string(STATUS_PROBE_COMMANDS[config[CONF_PROBE]])))
    return var
