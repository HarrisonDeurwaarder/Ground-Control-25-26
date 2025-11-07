package org.firstinspires.ftc.teamcode.teamcode.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class GamepadBindings {
    private Map<Supplier<Boolean>, Consumer<Boolean>> toggle;
    private Map<Supplier<Boolean>, Consumer<Boolean>> hold;
    private HashMap<Supplier<Boolean>, Boolean> previousPresses;
    private HashMap<Supplier<Boolean>, Boolean> toggleStatuses;

    public GamepadBindings(
            Map<Supplier<Boolean>, Consumer<Boolean>> toggle,
            Map<Supplier<Boolean>, Consumer<Boolean>> hold
    ) {
        // Save button getters and action executors
        this.toggle = toggle;
        this.hold = hold;
        // Array of past presses
        // Only for the buttons that need to be toggles or have a pre-hold method
        previousPresses = new HashMap<>();
        for (Supplier<Boolean> pressGetter : toggle.keySet()) {
            previousPresses.put(pressGetter, false);
        }
        toggleStatuses = new HashMap<>();
        for (Supplier<Boolean> pressGetter : toggle.keySet()) {
            toggleStatuses.put(pressGetter, false);
        }
    }

    public void update() {
        boolean toggleValue;

        // Execute actions for the toggle buttons
        for (Supplier<Boolean> pressedGetter : toggle.keySet()) {
            // Verify that the button should be toggled
            if (pressedGetter.get() && !previousPresses.get(pressedGetter)) {
                // If the button should be toggled, track whether it should be toggled on/off
                toggleValue = !toggleStatuses.get(pressedGetter);
                // Invert the toggle status for this button
                toggleStatuses.put(pressedGetter, toggleValue);
                // Call the given function with the toggle status
                toggle.get(pressedGetter).accept(toggleValue);
            }
        }

        // Update the past presses
        for (Supplier<Boolean> pressedGetter : toggle.keySet()) {
            previousPresses.put(pressedGetter, pressedGetter.get());
        }

        // Execute actions for the hold buttons
        for (Supplier<Boolean> pressedGetter : hold.keySet()) {
            // Send the button status to the executor
            hold.get(pressedGetter).accept(
                pressedGetter.get()
            );
        }
    }
}
