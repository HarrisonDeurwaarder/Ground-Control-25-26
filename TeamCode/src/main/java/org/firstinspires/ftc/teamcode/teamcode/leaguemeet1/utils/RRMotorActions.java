package org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RRMotorActions {
    private DcMotorEx transport;

    public RRMotorActions(DcMotorEx transport) {
        this.transport = transport;
    }

    public class FeedFlywheel implements Action {
        private double feedDuration;
        private double endMs;
        private boolean initialized = false;

        public FeedFlywheel(double feedDurationSec) {
            feedDuration = feedDurationSec;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // If motor has not yet been turned on
            if (!initialized) {
                transport.setVelocity(
                        MotorDriverPID.toTPS(MotorDriverPID.TRANSPORT_RPS)
                );
                initialized = true;
                // Mark the desired ending time
                endMs = System.currentTimeMillis() + feedDuration * 1000.0;
            }
            // Only end if the desired time has been reached
            // Duration hasn't been reached
            if (System.currentTimeMillis() < endMs) {
                return true;
            }
            // Duration has been reached; disable motor
            else {
                transport.setVelocity(0.0);
                return false;
            }
        }
    }

    public Action feedFlywheel(double feedDurationSec) {
        return new FeedFlywheel(feedDurationSec);
    }
}