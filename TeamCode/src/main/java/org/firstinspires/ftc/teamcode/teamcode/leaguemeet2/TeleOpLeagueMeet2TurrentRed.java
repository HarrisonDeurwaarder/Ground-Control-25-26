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

package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils.GamepadBindings;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils.GamepadBindingsCfg;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils.MotorDriver;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils.MotorDriverPID;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils.LimelightController;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils.HardwareController;

/*
 * This OpMode runs a manual omni-directional drivetrain
 */

@TeleOp(name="League Meet Turrent Test Red (TeleOp)", group="League Meet 2")
public class TeleOpLeagueMeet2TurrentRed extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private MotorDriverPID motorDriver;
    private GamepadBindings keybinds;
    private GamepadBindingsCfg keybindsCfg;
    private LimelightController limelightController;
    private HardwareController hardwareController;

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Instanciate a motor driver using the now non-null hardwareMap
        motorDriver = new MotorDriverPID(hardwareMap);

        // Access bindings for current driver
        // Set config bindings to Louis setting
        keybindsCfg = new GamepadBindingsCfg(
            gamepad1,
            motorDriver,
            MotorDriverPID.MAX_DRIVE_RPS
        );
        keybindsCfg.initLouisPID();

        // Create the keybind handler
        keybinds = new GamepadBindings(
            keybindsCfg.toggleKeybinds,
            keybindsCfg.holdKeybinds
        );

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double y = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double x = -gamepad1.left_stick_x;
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

            if (max > MotorDriver.MAX_DRIVE_POWER) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Send calculated power to wheels
            motorDriver.setDrivetrainPower(
                    MotorDriverPID.toTPS(frontLeftPower),
                    MotorDriverPID.toTPS(backLeftPower),
                    MotorDriverPID.toTPS(frontRightPower),
                    MotorDriverPID.toTPS(backRightPower),
                    keybindsCfg.drivetrainPowerScale
            );

            // Do limelight thingy then do turrent thingy
            LLResultTypes.FiducialResult goalOffset = limelightController.getGoalOffset("Red");
            // if the red goal can't be found (update later to include for any goal)
            if (goalOffset == null) {
                hardwareController.searchForGoal();
            }
            // else, if the red goal is found/in frame
            else {
                hardwareController.alignTurretToGoal(goalOffset);
            }

            // Update control binds
            // Trigger events based on presses
            keybinds.update();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Active Time", "%.1f seconds\n", runtime.seconds());

            telemetry.addData("Front RPS (Left / Right)", "(%4.2f / %4.2f)", MotorDriverPID.toRPS(motorDriver.frontLeftDrive.getVelocity()), MotorDriverPID.toRPS(motorDriver.frontRightDrive.getVelocity()));
            telemetry.addData("Back RPS (Left / Right)", "(%4.2f / %4.2f)", MotorDriverPID.toRPS(motorDriver.backLeftDrive.getVelocity()), MotorDriverPID.toRPS(motorDriver.backRightDrive.getVelocity()));
            telemetry.addData("Intake Direction", "%s", motorDriver.intakeMotor.getDirection().toString());
            telemetry.addData("Mechanism Power (Intake / Transport)", "(%b / %b)", !motorDriver.intakeMotor.getPowerFloat(), !motorDriver.transportMotor.getPowerFloat());
            telemetry.addData("Flywheel RPS", "%4.2f Revolutions / Second\n", MotorDriverPID.toRPS(motorDriver.flywheelMotor.getVelocity()));

            telemetry.addData("RT", "Intake");
            telemetry.addData("LT", "Outtake");
            telemetry.addData("RB", "Launch");
            telemetry.addData("LB", "Flywheel Toggle");
            telemetry.addData("A", "Precision Movement");
            telemetry.update();
        }
    }}
