#include "coffee_maker.hpp"

#include "esphome/core/log.h"
#include "esphome/core/time.h"
#include "jutta_commands.hpp"
#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>

//---------------------------------------------------------------------------
namespace jutta_proto {
//---------------------------------------------------------------------------

static const char* TAG = "coffee_maker";

void CoffeeMaker::CommandState::reset() {
    this->active = false;
    this->command.clear();
    this->delay_ms = 0;
    this->delay_target = 0;
    this->sent = false;
    this->timeout = std::chrono::milliseconds{5000};
}

CoffeeMaker::CoffeeMaker(std::unique_ptr<JuttaConnection>&& connection) : connection(std::move(connection)) {
    this->reset_states();
}

void CoffeeMaker::switch_page() { this->switch_page((this->pageNum + 1) % NUM_PAGES); }

void CoffeeMaker::switch_page(size_t pageNum) {
    if (this->locked) {
        ESP_LOGW(TAG, "Coffee maker busy - cannot switch page right now.");
        return;
    }

    size_t target_page = pageNum % NUM_PAGES;
    if (this->pageNum == target_page) {
        return;
    }

    this->switch_state_.target_page = target_page;
    this->start_operation(OperationType::SwitchPage);
}

void CoffeeMaker::brew_coffee(coffee_t coffee) {
    if (this->locked) {
        ESP_LOGW(TAG, "Coffee maker busy - cannot brew new coffee right now.");
        return;
    }

    this->brew_state_.coffee = coffee;
    this->brew_state_.target_page = this->get_page_num(coffee);
    this->brew_state_.button = this->get_button_num(coffee);
    this->brew_state_.stage = BrewCoffeeState::Stage::EnsurePage;
    this->start_operation(OperationType::BrewCoffee);
}

void CoffeeMaker::brew_custom_coffee(const bool* cancel, const std::chrono::milliseconds& grindTime,
                                     const std::chrono::milliseconds& waterTime) {
    if (this->locked) {
        ESP_LOGW(TAG, "Coffee maker busy - cannot brew custom coffee right now.");
        return;
    }

    this->custom_state_.cancel_flag = cancel;
    this->custom_state_.grind_duration = static_cast<uint32_t>(grindTime.count());
    this->custom_state_.water_duration = static_cast<uint32_t>(waterTime.count());
    this->custom_state_.wait_target = 0;
    this->custom_state_.stage = CustomBrewState::Stage::Start;

    ESP_LOGI(TAG, "Brewing custom coffee with %lld ms grind time and %lld ms water time...",
             static_cast<long long>(grindTime.count()), static_cast<long long>(waterTime.count()));

    this->start_operation(OperationType::BrewCustomCoffee);
}

void CoffeeMaker::loop() {
    switch (this->current_operation_) {
        case OperationType::Idle:
            break;
        case OperationType::SwitchPage:
            this->handle_switch_page();
            break;
        case OperationType::BrewCoffee:
            this->handle_brew_coffee();
            break;
        case OperationType::BrewCustomCoffee:
            this->handle_custom_brew();
            break;
    }
}

size_t CoffeeMaker::get_page_num(coffee_t coffee) const {
    for (const std::pair<const coffee_t, size_t>& c : this->coffee_page_map) {
        if (c.first == coffee) {
            return c.second;
        }
    }
    assert(false);  // Should not happen
    return std::numeric_limits<size_t>::max();
}

CoffeeMaker::jutta_button_t CoffeeMaker::get_button_num(coffee_t coffee) const {
    for (const std::pair<const coffee_t, jutta_button_t>& c : this->coffee_button_map) {
        if (c.first == coffee) {
            return c.second;
        }
    }
    assert(false);  // Should not happen
    return jutta_button_t::BUTTON_6;
}

const std::string& CoffeeMaker::command_for_button(jutta_button_t button) const {
    switch (button) {
        case jutta_button_t::BUTTON_1:
            return JUTTA_BUTTON_1;
        case jutta_button_t::BUTTON_2:
            return JUTTA_BUTTON_2;
        case jutta_button_t::BUTTON_3:
            return JUTTA_BUTTON_3;
        case jutta_button_t::BUTTON_4:
            return JUTTA_BUTTON_4;
        case jutta_button_t::BUTTON_5:
            return JUTTA_BUTTON_5;
        case jutta_button_t::BUTTON_6:
            return JUTTA_BUTTON_6;
    }
    assert(false);
    return JUTTA_BUTTON_6;
}

CoffeeMaker::CommandResult CoffeeMaker::run_command(const std::string& command, uint32_t delay_ms,
                                                    const std::chrono::milliseconds& timeout) {
    if (!this->command_state_.active) {
        this->command_state_.active = true;
        this->command_state_.command = command;
        this->command_state_.delay_ms = delay_ms;
        this->command_state_.delay_target = 0;
        this->command_state_.sent = false;
        this->command_state_.timeout = timeout;
    }

    if (!this->command_state_.sent) {
        if (!this->connection->write_decoded(this->command_state_.command)) {
            return CommandResult::InProgress;
        }
        this->command_state_.sent = true;
    }

    auto wait_result = this->connection->wait_for_ok(this->command_state_.timeout);
    if (wait_result == JuttaConnection::WaitResult::Pending) {
        return CommandResult::InProgress;
    }

    if (wait_result == JuttaConnection::WaitResult::Success) {
        if (this->command_state_.delay_ms > 0) {
            uint32_t now = esphome::millis();
            if (this->command_state_.delay_target == 0) {
                this->command_state_.delay_target = now + this->command_state_.delay_ms;
            }
            if (!time_reached(now, this->command_state_.delay_target)) {
                return CommandResult::InProgress;
            }
        }
        this->command_state_.reset();
        return CommandResult::Success;
    }

    this->command_state_.reset();
    if (wait_result == JuttaConnection::WaitResult::Timeout) {
        return CommandResult::Timeout;
    }
    return CommandResult::Error;
}

CoffeeMaker::CommandResult CoffeeMaker::run_press_button(jutta_button_t button) {
    return this->run_command(this->command_for_button(button), 500);
}

bool CoffeeMaker::handle_command(CommandResult result, const char* description) {
    switch (result) {
        case CommandResult::InProgress:
            return false;
        case CommandResult::Success:
            return true;
        case CommandResult::Timeout:
            ESP_LOGW(TAG, "%s timed out.", description);
            this->operation_failed_ = true;
            return false;
        case CommandResult::Error:
            ESP_LOGE(TAG, "%s failed.", description);
            this->operation_failed_ = true;
            return false;
    }
    return false;
}

CoffeeMaker::StepResult CoffeeMaker::ensure_page(size_t target_page) {
    if (this->pageNum == target_page) {
        return StepResult::Done;
    }

    CommandResult result = this->run_press_button(jutta_button_t::BUTTON_6);
    if (result == CommandResult::Success) {
        this->pageNum = (this->pageNum + 1) % NUM_PAGES;
        if (this->pageNum == target_page) {
            return StepResult::Done;
        }
        return StepResult::InProgress;
    }

    if (result == CommandResult::InProgress) {
        return StepResult::InProgress;
    }

    this->handle_command(result, "Switching page");
    return StepResult::Failed;
}

void CoffeeMaker::handle_switch_page() {
    if (this->operation_failed_) {
        this->finish_operation();
        return;
    }

    StepResult result = this->ensure_page(this->switch_state_.target_page);
    if (result == StepResult::Done) {
        this->finish_operation();
    } else if (result == StepResult::Failed) {
        this->finish_operation();
    }
}

void CoffeeMaker::handle_brew_coffee() {
    if (this->operation_failed_) {
        this->finish_operation();
        return;
    }

    switch (this->brew_state_.stage) {
        case BrewCoffeeState::Stage::EnsurePage: {
            StepResult page_result = this->ensure_page(this->brew_state_.target_page);
            if (page_result == StepResult::Done) {
                this->brew_state_.stage = BrewCoffeeState::Stage::PressButton;
            } else if (page_result == StepResult::Failed) {
                this->finish_operation();
            }
            break;
        }
        case BrewCoffeeState::Stage::PressButton: {
            CommandResult command_result = this->run_press_button(this->brew_state_.button);
            if (this->handle_command(command_result, "Pressing brew button")) {
                this->brew_state_.stage = BrewCoffeeState::Stage::Done;
            }
            break;
        }
        case BrewCoffeeState::Stage::Done:
            this->finish_operation();
            break;
    }

    if (this->operation_failed_) {
        this->finish_operation();
    }
}

bool CoffeeMaker::cancel_requested() const {
    return (this->custom_state_.cancel_flag != nullptr) && *(this->custom_state_.cancel_flag);
}

void CoffeeMaker::start_hot_water() {
    this->hot_water_state_.stage = HotWaterState::Stage::PumpOn;
    this->hot_water_state_.end_time = esphome::millis() + this->custom_state_.water_duration;
    this->hot_water_state_.wait_target = 0;
    this->hot_water_state_.heater_on_duration = this->custom_state_.water_duration / 8;
    this->hot_water_state_.heater_off_duration = this->custom_state_.water_duration / 20;
}

CoffeeMaker::HotWaterResult CoffeeMaker::run_hot_water() {
    if (this->operation_failed_) {
        return HotWaterResult::Failed;
    }

    uint32_t now = esphome::millis();

    switch (this->hot_water_state_.stage) {
        case HotWaterState::Stage::PumpOn: {
            CommandResult result = this->run_command(JUTTA_COFFEE_WATER_PUMP_ON);
            if (this->handle_command(result, "Turning water pump on")) {
                this->hot_water_state_.stage = HotWaterState::Stage::CycleStart;
            }
            break;
        }
        case HotWaterState::Stage::CycleStart:
            if (this->cancel_requested()) {
                this->hot_water_state_.stage = HotWaterState::Stage::CancelPumpOff;
            } else if (time_reached(now, this->hot_water_state_.end_time)) {
                this->hot_water_state_.stage = HotWaterState::Stage::PumpOff;
            } else {
                this->hot_water_state_.stage = HotWaterState::Stage::HeaterOn;
            }
            break;
        case HotWaterState::Stage::HeaterOn: {
            CommandResult result = this->run_command(JUTTA_COFFEE_WATER_HEATER_ON);
            if (this->handle_command(result, "Turning water heater on")) {
                if (this->hot_water_state_.heater_on_duration == 0) {
                    this->hot_water_state_.stage = HotWaterState::Stage::HeaterOff;
                } else {
                    this->hot_water_state_.wait_target = now + this->hot_water_state_.heater_on_duration;
                    this->hot_water_state_.stage = HotWaterState::Stage::WaitHeaterOn;
                }
            }
            break;
        }
        case HotWaterState::Stage::WaitHeaterOn:
            if (this->cancel_requested()) {
                this->hot_water_state_.stage = HotWaterState::Stage::CancelHeaterOff;
            } else if (time_reached(now, this->hot_water_state_.wait_target)) {
                this->hot_water_state_.stage = HotWaterState::Stage::HeaterOff;
            }
            break;
        case HotWaterState::Stage::HeaterOff: {
            CommandResult result = this->run_command(JUTTA_COFFEE_WATER_HEATER_OFF);
            if (this->handle_command(result, "Turning water heater off")) {
                if (this->hot_water_state_.heater_off_duration == 0) {
                    this->hot_water_state_.stage = HotWaterState::Stage::CycleStart;
                } else {
                    this->hot_water_state_.wait_target = now + this->hot_water_state_.heater_off_duration;
                    this->hot_water_state_.stage = HotWaterState::Stage::WaitHeaterOff;
                }
            }
            break;
        }
        case HotWaterState::Stage::WaitHeaterOff:
            if (this->cancel_requested()) {
                this->hot_water_state_.stage = HotWaterState::Stage::CancelPumpOff;
            } else if (time_reached(now, this->hot_water_state_.wait_target)) {
                this->hot_water_state_.stage = HotWaterState::Stage::CycleStart;
            }
            break;
        case HotWaterState::Stage::PumpOff: {
            CommandResult result = this->run_command(JUTTA_COFFEE_WATER_PUMP_OFF);
            if (this->handle_command(result, "Turning water pump off")) {
                this->hot_water_state_.stage = HotWaterState::Stage::Done;
                return HotWaterResult::Completed;
            }
            break;
        }
        case HotWaterState::Stage::WaitPumpOff:
            // Should never be reached with current state machine
            break;
        case HotWaterState::Stage::CancelHeaterOff: {
            CommandResult result = this->run_command(JUTTA_COFFEE_WATER_HEATER_OFF);
            if (this->handle_command(result, "Turning water heater off after cancel")) {
                this->hot_water_state_.stage = HotWaterState::Stage::CancelPumpOff;
            }
            break;
        }
        case HotWaterState::Stage::CancelPumpOff: {
            CommandResult result = this->run_command(JUTTA_COFFEE_WATER_PUMP_OFF);
            if (this->handle_command(result, "Turning water pump off after cancel")) {
                this->hot_water_state_.stage = HotWaterState::Stage::Cancelled;
                return HotWaterResult::Cancelled;
            }
            break;
        }
        case HotWaterState::Stage::Done:
            return HotWaterResult::Completed;
        case HotWaterState::Stage::Cancelled:
            return HotWaterResult::Cancelled;
        case HotWaterState::Stage::Error:
            return HotWaterResult::Failed;
    }

    if (this->operation_failed_) {
        this->hot_water_state_.stage = HotWaterState::Stage::Error;
        return HotWaterResult::Failed;
    }

    return HotWaterResult::InProgress;
}

void CoffeeMaker::handle_custom_brew() {
    if (this->operation_failed_) {
        this->finish_operation();
        return;
    }

    uint32_t now = esphome::millis();

    switch (this->custom_state_.stage) {
        case CustomBrewState::Stage::Idle:
            this->finish_operation();
            return;
        case CustomBrewState::Stage::Start:
            ESP_LOGI(TAG, "Custom coffee grinding...");
            this->custom_state_.stage = CustomBrewState::Stage::GrinderOn;
            break;
        case CustomBrewState::Stage::GrinderOn: {
            CommandResult result = this->run_command(JUTTA_GRINDER_ON);
            if (this->handle_command(result, "Turning grinder on")) {
                this->custom_state_.wait_target = now + this->custom_state_.grind_duration;
                this->custom_state_.stage = CustomBrewState::Stage::WaitGrinding;
            }
            break;
        }
        case CustomBrewState::Stage::WaitGrinding:
            if (this->cancel_requested()) {
                this->custom_state_.stage = CustomBrewState::Stage::CancelGrindingReset;
            } else if (time_reached(now, this->custom_state_.wait_target)) {
                this->custom_state_.stage = CustomBrewState::Stage::GrinderOff;
            }
            break;
        case CustomBrewState::Stage::CancelGrindingReset:
            if (this->handle_command(this->run_command(JUTTA_BREW_GROUP_RESET),
                                     "Reset brew group after grind cancel")) {
                this->custom_state_.stage = CustomBrewState::Stage::Cancelled;
            }
            break;
        case CustomBrewState::Stage::GrinderOff:
            if (this->handle_command(this->run_command(JUTTA_GRINDER_OFF), "Turning grinder off")) {
                this->custom_state_.stage = CustomBrewState::Stage::MoveBrewGroup;
            }
            break;
        case CustomBrewState::Stage::MoveBrewGroup:
            if (this->handle_command(this->run_command(JUTTA_BREW_GROUP_TO_BREWING_POSITION),
                                     "Moving brew group")) {
                ESP_LOGI(TAG, "Custom coffee compressing...");
                this->custom_state_.stage = CustomBrewState::Stage::PressOn;
            }
            break;
        case CustomBrewState::Stage::PressOn:
            if (this->handle_command(this->run_command(JUTTA_COFFEE_PRESS_ON), "Turning coffee press on")) {
                this->custom_state_.wait_target = now + this->custom_state_.grind_duration;
                this->custom_state_.stage = CustomBrewState::Stage::WaitCompression;
            }
            break;
        case CustomBrewState::Stage::WaitCompression:
            if (this->cancel_requested()) {
                this->custom_state_.stage = CustomBrewState::Stage::CancelPressOff;
            } else if (time_reached(now, this->custom_state_.wait_target)) {
                this->custom_state_.wait_target = now + 500;
                this->custom_state_.stage = CustomBrewState::Stage::DelayAfterPress;
            }
            break;
        case CustomBrewState::Stage::CancelPressOff:
            if (this->handle_command(this->run_command(JUTTA_COFFEE_PRESS_OFF),
                                     "Turning coffee press off after cancel")) {
                this->custom_state_.stage = CustomBrewState::Stage::CancelPressReset;
            }
            break;
        case CustomBrewState::Stage::CancelPressReset:
            if (this->handle_command(this->run_command(JUTTA_BREW_GROUP_RESET),
                                     "Reset brew group after press cancel")) {
                this->custom_state_.stage = CustomBrewState::Stage::Cancelled;
            }
            break;
        case CustomBrewState::Stage::DelayAfterPress:
            if (time_reached(now, this->custom_state_.wait_target)) {
                this->custom_state_.stage = CustomBrewState::Stage::PressOff;
            }
            break;
        case CustomBrewState::Stage::PressOff:
            if (this->handle_command(this->run_command(JUTTA_COFFEE_PRESS_OFF), "Turning coffee press off")) {
                ESP_LOGI(TAG, "Custom coffee brewing...");
                this->custom_state_.stage = CustomBrewState::Stage::PumpOn;
            }
            break;
        case CustomBrewState::Stage::PumpOn:
            if (this->handle_command(this->run_command(JUTTA_COFFEE_WATER_PUMP_ON), "Turning water pump on")) {
                this->custom_state_.wait_target = now + 2000;
                this->custom_state_.stage = CustomBrewState::Stage::WaitPreBrew;
            }
            break;
        case CustomBrewState::Stage::WaitPreBrew:
            if (this->cancel_requested()) {
                this->custom_state_.stage = CustomBrewState::Stage::CancelPreBrewPumpOff;
            } else if (time_reached(now, this->custom_state_.wait_target)) {
                this->custom_state_.stage = CustomBrewState::Stage::PumpOff;
            }
            break;
        case CustomBrewState::Stage::CancelPreBrewPumpOff:
            if (this->handle_command(this->run_command(JUTTA_COFFEE_WATER_PUMP_OFF),
                                     "Turning water pump off after cancel")) {
                this->custom_state_.stage = CustomBrewState::Stage::CancelPreBrewReset;
            }
            break;
        case CustomBrewState::Stage::CancelPreBrewReset:
            if (this->handle_command(this->run_command(JUTTA_BREW_GROUP_RESET),
                                     "Reset brew group after pump cancel")) {
                this->custom_state_.stage = CustomBrewState::Stage::Cancelled;
            }
            break;
        case CustomBrewState::Stage::PumpOff:
            if (this->handle_command(this->run_command(JUTTA_COFFEE_WATER_PUMP_OFF), "Turning water pump off")) {
                this->custom_state_.wait_target = now + 2000;
                this->custom_state_.stage = CustomBrewState::Stage::WaitBetweenBrews;
            }
            break;
        case CustomBrewState::Stage::WaitBetweenBrews:
            if (this->cancel_requested()) {
                this->custom_state_.stage = CustomBrewState::Stage::CancelAfterPreBrewReset;
            } else if (time_reached(now, this->custom_state_.wait_target)) {
                this->custom_state_.stage = CustomBrewState::Stage::HotWaterInit;
            }
            break;
        case CustomBrewState::Stage::CancelAfterPreBrewReset:
            if (this->handle_command(this->run_command(JUTTA_BREW_GROUP_RESET),
                                     "Reset brew group after pre-brew cancel")) {
                this->custom_state_.stage = CustomBrewState::Stage::Cancelled;
            }
            break;
        case CustomBrewState::Stage::HotWaterInit:
            this->start_hot_water();
            this->custom_state_.stage = CustomBrewState::Stage::HotWaterActive;
            break;
        case CustomBrewState::Stage::HotWaterActive: {
            HotWaterResult hw_result = this->run_hot_water();
            if (hw_result == HotWaterResult::Completed) {
                ESP_LOGI(TAG, "Custom coffee finishing up...");
                this->custom_state_.stage = CustomBrewState::Stage::Reset;
            } else if (hw_result == HotWaterResult::Cancelled) {
                this->custom_state_.stage = CustomBrewState::Stage::CancelAfterHotWaterReset;
            } else if (hw_result == HotWaterResult::Failed) {
                this->operation_failed_ = true;
            }
            break;
        }
        case CustomBrewState::Stage::CancelAfterHotWaterReset:
            if (this->handle_command(this->run_command(JUTTA_BREW_GROUP_RESET),
                                     "Reset brew group after hot water cancel")) {
                this->custom_state_.stage = CustomBrewState::Stage::Cancelled;
            }
            break;
        case CustomBrewState::Stage::Reset:
            if (this->handle_command(this->run_command(JUTTA_BREW_GROUP_RESET), "Reset brew group")) {
                this->custom_state_.stage = CustomBrewState::Stage::Done;
            }
            break;
        case CustomBrewState::Stage::Done:
            ESP_LOGI(TAG, "Custom coffee done.");
            this->finish_operation();
            break;
        case CustomBrewState::Stage::Cancelled:
            ESP_LOGI(TAG, "Custom coffee cancelled.");
            this->finish_operation();
            break;
        case CustomBrewState::Stage::Error:
            this->operation_failed_ = true;
            break;
    }

    if (this->operation_failed_) {
        ESP_LOGE(TAG, "Custom coffee failed.");
        this->finish_operation();
    }
}

bool CoffeeMaker::is_locked() const { return this->locked; }

void CoffeeMaker::start_operation(OperationType operation) {
    this->current_operation_ = operation;
    this->operation_failed_ = false;
    this->command_state_.reset();
    this->hot_water_state_ = {};
    this->locked = true;
}

void CoffeeMaker::finish_operation() {
    this->command_state_.reset();
    this->hot_water_state_ = {};
    this->custom_state_.stage = CustomBrewState::Stage::Idle;
    this->custom_state_.cancel_flag = nullptr;
    this->brew_state_ = {};
    this->switch_state_ = {};
    this->current_operation_ = OperationType::Idle;
    this->operation_failed_ = false;
    this->locked = false;
}

void CoffeeMaker::reset_states() {
    this->command_state_.reset();
    this->hot_water_state_ = {};
    this->custom_state_ = {};
    this->brew_state_ = {};
    this->switch_state_ = {};
    this->current_operation_ = OperationType::Idle;
    this->operation_failed_ = false;
    this->locked = false;
}

bool CoffeeMaker::time_reached(uint32_t now, uint32_t target) {
    return static_cast<int32_t>(now - target) >= 0;
}

//---------------------------------------------------------------------------
}  // namespace jutta_proto
//---------------------------------------------------------------------------
