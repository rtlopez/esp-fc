/**
 * Simple, reliable button with multiple types of click detection.
 *
 * Supports debounced click, singleClick, doubleClick, longPress and
 * release events.
 *
 * This class provides a generic button that allows you to use it with
 * any kind of 'raw' button (e.g. capacitive sensor).
 * See PinButton for a simple wrapper to simply use an Arduino digital
 * pin.
 *
 * Note: in order to provide a minimal code/memory footprint, the
 * default timings for debounce etc are hardcoded, but should be decent
 * for most cases.
 *
 * Copyright (C) Martin Poelstra
 * MIT License
 */

#pragma once

#include <Arduino.h>

struct MultiButtonConfig {
  int debounceDecay;     // ms
  int singleClickDelay;  // ms
  int longClickDelay;    // ms
};

const static MultiButtonConfig DEFAULT_MULTIBUTTON_CONFIG = { 20, 250, 300 };

/**
 * Generic button with single, double and long click detection.
 *
 * Note: to use a simple switch between an Arduino pin and ground,
 * see PinButton for an easier wrapper.
 *
 * Usage example:
 * ```cpp
 * MultiButton btn1;
 *
 * void setup() {
 *   Serial.begin(9600);
 * }
 *
 * void loop() {
 *   bool pressed = getSomeButtonPressedState();
 *   btn1.update(pressed);
 *   if (btn1.isClick()) {
 *     Serial.println("click");
 *   }
 * }
 * ```
 */
class MultiButton {
  public:
    /**
     * MultiButton constructor with default settings for debounce/click delays.
     *
     * See class doc for usage and example.
     */
    MultiButton()
        : MultiButton(&DEFAULT_MULTIBUTTON_CONFIG) {
    };

    /**
     * MultiButton constructor with custom settings for debounce/click delays.
     *
     * See class doc for usage and example.
     */
    MultiButton(const MultiButtonConfig* configuration)
        : _lastTransition(millis()), _state(StateIdle), _new(false) {
        _configPtr = configuration;
    };

    /**
     * Decode hardware button state into clean clicks (single, double, long).
     * The state of isClick() etc is valid until the next call to update().
     *
     * Periodically call update() with the current (raw, non-debounced)
     * state of a (hardware) button.
     * Suggestion is to put it in e.g. loop(), but make sure to call it at
     * least every ~20ms.
     *
     * @param pressed {bool} Whether raw button is currently pressed
     */
    void update(bool pressed) {
      _new = false;

      // Optimization for the most common case: nothing happening.
      if (!pressed && _state == StateIdle) {
        return;
      }

      // Only take the low word, works as long as update() is called
      // at least every ~32s, and typical use-case is <50ms.
      // Also note that wrapping during StateIdle doesn't really matter,
      // as the time isn't used then.
      unsigned int now = millis();
      int diff = now - _lastTransition;

      State next = StateIdle;
      switch (_state) {
        case StateIdle:                next = _checkIdle(pressed, diff);                break;
        case StateDebounce:            next = _checkDebounce(pressed, diff);            break;
        case StatePressed:             next = _checkPressed(pressed, diff);             break;
        case StateClickUp:             next = _checkClickUp(pressed, diff);             break;
        case StateClickIdle:           next = _checkClickIdle(pressed, diff);           break;
        case StateSingleClick:         next = _checkSingleClick(pressed, diff);         break;
        case StateDoubleClickDebounce: next = _checkDoubleClickDebounce(pressed, diff); break;
        case StateDoubleClick:         next = _checkDoubleClick(pressed, diff);         break;
        case StateLongClick:           next = _checkLongClick(pressed, diff);           break;
        case StateOtherUp:             next = _checkOtherUp(pressed, diff);             break;
      }

      if (next != _state) {
        // Mark current state transition to make 'diff' meaningful
        _lastTransition = now;
        // Enter next state
        _state = next;
        // Only mark this state 'new' for one iteration, to cause
        // e.g. isClick() to return true until the next call to update()
        _new = true;
      }
    }

    /**
     * True when any kind of click is detected (single, double or long).
     * A Click is detected at the same time as a Double or Long click, but
     * earlier than a Single click.
     */
    bool isClick() const {
      return _new && (_state == StatePressed || _state == StateDoubleClick);
    }

    /**
     * True when a Single click is detected, i.e. it will not trigger before
     * e.g. a Double click.
     */
    bool isSingleClick() {
      return _new && _state == StateSingleClick;
    }

    /**
     * True when a Double click is detected.
     */
    bool isDoubleClick() {
      return _new && _state == StateDoubleClick;
    }

    /**
     * True when a Long click is detected.
     */
    bool isLongClick() {
      return _new && _state == StateLongClick;
    }

    /**
     * True once the button is released after Click, Long click or Double click.
     *
     * Note: there is no release event after a Single click, because that is a
     * 'synthetic' event that happens after a normal click.
     */
    bool isReleased() {
      return _new && (_state == StateClickUp || _state == StateOtherUp);
    }

  private:
    const MultiButtonConfig* _configPtr;

    /**
     * Note:
     * - SingleClick is only emitted when it's definitely not going to be a double or long click
     * - Click is emitted when either Pressed or DoubleClick happens (so it's emitted whenever any
     *   click happens, but earlier than the SingleClick or LongClick is detected)
     *
     * Several example patterns and resulting states.
     * Delays aren't at scale. State transitions when input
     * changes are based on that input, transitions when
     * input didn't change are due to timeouts.
     * Start state is Idle.
     *
     * Clean single click:
     * _______########___________________
     *        |   |   ||      |Idle
     *        |   |   ||      SingleClick
     *        |   |   |ClickIdle
     *        |   |   ClickUp
     *        |   Pressed
     *        Debounce
     *
     * Clean double click:
     * _______########______########_____
     *        |   |   ||    |   |   |Idle
     *        |   |   ||    |   |   OtherUp
     *        |   |   ||    |   DoubleClick
     *        |   |   ||    DoubleClickDebounce
     *        |   |   |ClickIdle
     *        |   |   ClickUp
     *        |   Pressed
     *        Debounce
     *
     * Clean long click:
     * _______####################_______
     *        |   |         |     |Idle
     *        |   |         |     OtherUp
     *        |   |         LongClick
     *        |   Pressed
     *        Debounce
     *
     * Single click with bouncing:
     * ___#_#_########__#_#_______________
     *    |||||   |   ||||||      |Idle
     *    |||||   |   ||||||      SingleClick
     *    |||||   |   |||||ClickIdle
     *    |||||   |   ||||DoubleClickDebounce
     *    |||||   |   |||ClickIdle
     *    |||||   |   ||DoubleClickDebounce
     *    |||||   |   |ClickIdle
     *    |||||   |   ClickUp
     *    |||||   Pressed
     *    ||||Debounce
     *    |||Idle
     *    ||Debounce
     *    |Idle
     *    Debounce
     */
    enum State {
      StateIdle,
      StateDebounce,
      StatePressed,
      StateClickUp,
      StateClickIdle,
      StateSingleClick,
      StateDoubleClickDebounce,
      StateDoubleClick,
      StateLongClick,
      StateOtherUp,
    };

    unsigned int _lastTransition;
    State _state;
    bool _new;

    State _checkIdle(bool pressed, int diff) {
      (void)diff;
      // Wait for a key press
      return pressed ? StateDebounce : StateIdle;
    }

    State _checkDebounce(bool pressed, int diff) {
      // If released in this state: it was a glitch
      if (!pressed) {
        return StateIdle;
      }
      if (diff >= _configPtr->debounceDecay) {
        // Still pressed after debounce delay: real 'press'
        return StatePressed;
      }
      return StateDebounce;
    }

    State _checkPressed(bool pressed, int diff) {
      // If released, go wait for either a double-click, or
      // to generate the actual SingleClick event,
      // but first mark that the button is released.
      if (!pressed) {
        return StateClickUp;
      }
      // If pressed, keep waiting to see if it's a long click
      if (diff >= _configPtr->longClickDelay) {
        return StateLongClick;
      }
      return StatePressed;
    }

    State _checkClickUp(bool pressed, int diff) {
      (void)pressed;
      (void)diff;
      return StateClickIdle;
    }

    State _checkClickIdle(bool pressed, int diff) {
      if (pressed) {
        return StateDoubleClickDebounce;
      }
      if (diff >= _configPtr->singleClickDelay) {
        return StateSingleClick;
      }
      return StateClickIdle;
    }

    State _checkSingleClick(bool pressed, int diff) {
      (void)pressed;
      (void)diff;
      return StateIdle;
    }

    State _checkDoubleClickDebounce(bool pressed, int diff) {
      if (!pressed) {
        return StateClickIdle;
      }
      if (diff >= _configPtr->debounceDecay) {
        return StateDoubleClick;
      }
      return StateDoubleClickDebounce;
    }

    State _checkDoubleClick(bool pressed, int diff) {
      (void)diff;
      if (!pressed) {
        return StateOtherUp;
      }
      return StateDoubleClick;
    }

    State _checkLongClick(bool pressed, int diff) {
      (void)diff;
      if (!pressed) {
        return StateOtherUp;
      }
      return StateLongClick;
    }

    State _checkOtherUp(bool pressed, int diff) {
      (void)pressed;
      (void)diff;
      return StateIdle;
    }
};
