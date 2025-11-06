/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teamcode.leaguemeet1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.keybinds.GamepadBindings;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

/*
 * This OpMode runs a manual omni-directional drivetrain
 */

@TeleOp(name="League Meet 1 (TeleOp)", group="Linear OpMode")
public class TeleOpLeagueMeet1 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private MotorDriverLeagueMeet1 motorDriver;
    private GamepadBindings keybinds;
    private Map<Supplier<Boolean>, Consumer<Boolean>> toggleKeybinds, holdKeybinds;


    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Instanciate a motor driver using the now non-null hardwareMap
        motorDriver = new MotorDriverLeagueMeet1(hardwareMap);

        // Define the toggle keybinds
        toggleKeybinds = Map.<Supplier<Boolean>, Consumer<Boolean>>of(
            // Cycle to intake
            () -> gamepad1.right_trigger > MotorDriverLeagueMeet1.TRIGGER_THRESHOLD,
            (Boolean mode) -> motorDriver.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE),
            // Cycle to outtake
            () -> gamepad1.left_trigger > MotorDriverLeagueMeet1.TRIGGER_THRESHOLD,
            (Boolean mode) -> motorDriver.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD),
            // Toggle flywheel
            () -> gamepad1.left_bumper,
            (Boolean mode) -> motorDriver.flywheelMotor.setVelocity((mode) ? MotorDriverLeagueMeet1.FLYWHEEL_SPEED * MotorDriverLeagueMeet1.TPR : 0.0)
        );
        // Define the hold keybinds
        holdKeybinds = Map.<Supplier<Boolean>, Consumer<Boolean>>of(
            // Power the intake/outtake wheel
            () -> gamepad1.right_trigger > MotorDriverLeagueMeet1.TRIGGER_THRESHOLD || gamepad1.left_trigger > MotorDriverLeagueMeet1.TRIGGER_THRESHOLD,
            (Boolean mode) -> motorDriver.intakeMotor.setPower((mode) ? MotorDriverLeagueMeet1.INTAKE_POWER : 0.0),
            // Power the transfer mechanism (launch)
            () -> gamepad1.right_bumper,
            (Boolean mode) -> motorDriver.transportMotor.setPower((mode) ? MotorDriverLeagueMeet1.TRANSPORT_POWER : 0.0)
        );

        // Create the keybind handler
        keybinds = new GamepadBindings(
            toggleKeybinds,
            holdKeybinds
        );

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double y = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;

            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            /*
             * Handle wheel powering
             * Compute desired power for each wheel given the gamepad inputs
             * Normalize power to ensure nothing exceeds 1.0
             */
            double max;

            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > MotorDriverLeagueMeet1.MAX_DRIVE_POWER) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Send calculated power to wheels
            motorDriver.setDrivetrainPower(
                    frontLeftPower,
                    backLeftPower,
                    frontRightPower,
                    backRightPower
            );

            // Update control binds
            // Trigger events based on presses
            keybinds.update();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Active Time", "%.1f seconds\n", runtime.seconds());

            telemetry.addData("Front Power (Left / Right)", "(%4.2f / %4.2f)", frontLeftPower, frontRightPower);
            telemetry.addData("Back Power (Left / Right)", "(%4.2f / %4.2f)", backLeftPower, backRightPower);
            telemetry.addData("Mechanism Power (Intake / Belt)", "(%b / %b)", motorDriver.intakeMotor.getPowerFloat(), motorDriver.transportMotor.getPowerFloat());
            telemetry.addData("Flywheel Velocity", "%4.2f Revolutions / Second\n", motorDriver.flywheelMotor.getVelocity() / MotorDriverLeagueMeet1.TPR);

            telemetry.addData("RT", "Intake");
            telemetry.addData("LT", "Outtake");
            telemetry.addData("RB", "Launch");
            telemetry.addData("LB", "Flywheel Toggle");
            telemetry.update();
        }
    }}
