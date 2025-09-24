#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "jutta_connection.hpp"

//---------------------------------------------------------------------------
namespace jutta_proto {
//---------------------------------------------------------------------------
class CoffeeMaker {
 public:
    /**
     * All available coffee types.
     **/
    enum coffee_t { ESPRESSO = 0,
                    COFFEE = 1,
                    CAPPUCCINO = 2,
                    MILK_FOAM = 3,
                    CAFFE_BARISTA = 4,
                    LUNGO_BARISTA = 5,
                    ESPRESSO_DOPPIO = 6,
                    MACCHIATO = 7 };
    enum jutta_button_t {
        BUTTON_1 = 1,
        BUTTON_2 = 2,
        BUTTON_3 = 3,
        BUTTON_4 = 4,
        BUTTON_5 = 5,
        BUTTON_6 = 6,
    };

    std::unique_ptr<JuttaConnection> connection;

 private:
    static constexpr size_t NUM_PAGES = 2;
    /**
     * Mapping of all coffee types to page.
     **/
    std::map<coffee_t, size_t> coffee_page_map{{coffee_t::ESPRESSO, 0}, {coffee_t::COFFEE, 0}, {coffee_t::CAPPUCCINO, 0}, {coffee_t::MILK_FOAM, 0}, {coffee_t::CAFFE_BARISTA, 1}, {coffee_t::LUNGO_BARISTA, 1}, {coffee_t::ESPRESSO_DOPPIO, 1}, {coffee_t::MACCHIATO, 1}};
    /**
     * Mapping of all coffee types to their button.
     **/
    std::map<coffee_t, jutta_button_t> coffee_button_map{{coffee_t::ESPRESSO, jutta_button_t::BUTTON_1}, {coffee_t::COFFEE, jutta_button_t::BUTTON_2}, {coffee_t::CAPPUCCINO, jutta_button_t::BUTTON_4}, {coffee_t::MILK_FOAM, jutta_button_t::BUTTON_5}, {coffee_t::CAFFE_BARISTA, jutta_button_t::BUTTON_1}, {coffee_t::LUNGO_BARISTA, jutta_button_t::BUTTON_2}, {coffee_t::ESPRESSO_DOPPIO, jutta_button_t::BUTTON_4}, {coffee_t::MACCHIATO, jutta_button_t::BUTTON_5}};

    /**
     * The current page we are on.
     **/
    size_t pageNum{0};

    /**
     * True in case we are currently making a something like a cup of coffee. 
     **/
    bool locked{false};

 public:
    /**
     * Takes an initialized JuttaConnection.
     **/
    explicit CoffeeMaker(std::unique_ptr<JuttaConnection>&& connection);

    /**
     * Switches to the next page.
     * 0 -> 1
     * 1 -> 0
     **/
    void switch_page();
    /**
     * Switches to the given page number.
     * Does nothing, in case the page number is the same as the current one.
     **/
    void switch_page(size_t pageNum);
    /**
     * Brews the given coffee and switches to the appropriate page for this.
     **/
    void brew_coffee(coffee_t coffee);
    /**
     * Brews a custom coffee with the given grind and water times.
     * A default coffee on a JUTTA E6 (2019) grinds for 3.6 seconds and then lets the water run for 40 seconds (200 ml).
     * This corresponds to a water flow rate of 5 ml/s.
     * As long as cancel is set to true, the process will continue.
     * In case it changes from true to false, the coffee maker will cancel brewing and will reset the coffee maker to it's default state before returning.
     **/
    void brew_custom_coffee(const bool* cancel, const std::chrono::milliseconds& grindTime = std::chrono::milliseconds{3600}, const std::chrono::milliseconds& waterTime = std::chrono::milliseconds{40000});
    /**
     * Progresses the internal state machine.
     * Has to be called regularly from the ESPHome loop.
     **/
    void loop();

    /**
     * Returns true in case the coffee maker is locked due to it currently interacting with the coffee maker e.g. brewing a coffee.
     **/
    [[nodiscard]] bool is_locked() const;

 private:
    enum class CommandResult { InProgress, Success, Timeout, Error };
    enum class StepResult { InProgress, Done, Failed };
    enum class OperationType { Idle, SwitchPage, BrewCoffee, BrewCustomCoffee };

    struct CommandState {
        bool active{false};
        std::string command{};
        uint32_t delay_ms{0};
        uint32_t delay_target{0};
        bool sent{false};
        std::chrono::milliseconds timeout{std::chrono::milliseconds{5000}};

        void reset();
    };

    struct SwitchPageState {
        size_t target_page{0};
    };

    struct BrewCoffeeState {
        enum class Stage { EnsurePage, PressButton, Done } stage{Stage::EnsurePage};
        coffee_t coffee{ESPRESSO};
        size_t target_page{0};
        jutta_button_t button{jutta_button_t::BUTTON_1};
    };

    struct CustomBrewState {
        enum class Stage {
            Idle,
            Start,
            GrinderOn,
            WaitGrinding,
            CancelGrindingReset,
            GrinderOff,
            MoveBrewGroup,
            PressOn,
            WaitCompression,
            CancelPressOff,
            CancelPressReset,
            DelayAfterPress,
            PressOff,
            PumpOn,
            WaitPreBrew,
            CancelPreBrewPumpOff,
            CancelPreBrewReset,
            PumpOff,
            WaitBetweenBrews,
            CancelAfterPreBrewReset,
            HotWaterInit,
            HotWaterActive,
            CancelAfterHotWaterReset,
            Reset,
            Done,
            Cancelled,
            Error
        } stage{Stage::Idle};

        const bool* cancel_flag{nullptr};
        uint32_t grind_duration{0};
        uint32_t water_duration{0};
        uint32_t wait_target{0};
    };

    struct HotWaterState {
        enum class Stage {
            Idle,
            PumpOn,
            WaitPumpOn,
            CycleStart,
            HeaterOn,
            WaitHeaterOn,
            HeaterOff,
            WaitHeaterOff,
            PumpOff,
            WaitPumpOff,
            CancelHeaterOff,
            CancelPumpOff,
            Done,
            Cancelled,
            Error
        } stage{Stage::Idle};

        uint32_t end_time{0};
        uint32_t wait_target{0};
        uint32_t heater_on_duration{0};
        uint32_t heater_off_duration{0};
    };

    enum class HotWaterResult { InProgress, Completed, Cancelled, Failed };

    /**
     * Returns the page number for the given coffee type.
     **/
    [[nodiscard]] size_t get_page_num(coffee_t coffee) const;

    /**
     * Returns the button number for the given coffee type.
     **/
    [[nodiscard]] jutta_button_t get_button_num(coffee_t coffee) const;
    void start_operation(OperationType operation);
    void finish_operation();
    [[nodiscard]] static bool time_reached(uint32_t now, uint32_t target);
    [[nodiscard]] StepResult ensure_page(size_t target_page);
    [[nodiscard]] CommandResult run_command(const std::string& command, uint32_t delay_ms = 0,
                                            const std::chrono::milliseconds& timeout = std::chrono::milliseconds{5000});
    [[nodiscard]] CommandResult run_press_button(jutta_button_t button);
    [[nodiscard]] bool handle_command(CommandResult result, const char* description);
    [[nodiscard]] const std::string& command_for_button(jutta_button_t button) const;
    void handle_switch_page();
    void handle_brew_coffee();
    void handle_custom_brew();
    [[nodiscard]] bool cancel_requested() const;
    void start_hot_water();
    HotWaterResult run_hot_water();
    void reset_states();

    OperationType current_operation_{OperationType::Idle};
    SwitchPageState switch_state_{};
    BrewCoffeeState brew_state_{};
    CustomBrewState custom_state_{};
    HotWaterState hot_water_state_{};
    CommandState command_state_{};
    bool operation_failed_{false};
};
//---------------------------------------------------------------------------
}  // namespace jutta_proto
//---------------------------------------------------------------------------
