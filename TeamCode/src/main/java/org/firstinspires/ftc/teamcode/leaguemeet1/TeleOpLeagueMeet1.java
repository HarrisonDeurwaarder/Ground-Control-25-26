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

package org.firstinspires.ftc.teamcode.leaguemeet1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.keybinds.GamepadBindings;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

/*
 * This OpMode runs a manual omni-directional drivetrain
 */

@TeleOp(name="League Meet 0 (TeleOp)", group="Linear OpMode")
public class TeleOpLeagueMeet1 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private DcMotor intakeMotor;
    private DcMotor beltMotor;
    private DcMotor flywheelMotor;

    private GamepadBindings keybinds;
    private Map<Supplier<Boolean>, Consumer<Boolean>> toggleKeybinds;
    private Map<Supplier<Boolean>, Consumer<Boolean>> holdKeybinds;

    final private double intakePower = 0.6;
    final private double beltPower = 0.6;
    final private double flywheelPower = 0.6;

    final private double triggerThreshold = 0.1;
    final private double maxDrivePower = 1.0;

    @Override
    public void runOpMode() {

        // Initialize the motor variables
        frontLeftDrive = hardwareMap.get(DcMotor.class, "drivetrain_fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "drivetrain_bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "drivetrain_fr");
        backRightDrive = hardwareMap.get(DcMotor.class, "drivetrain_br");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        beltMotor = hardwareMap.get(DcMotor.class, "belt");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");

        // Set directions
        // The left-side motors need to be reversed
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        beltMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to brake if under zero power
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Define the toggle keybinds
        toggleKeybinds = Map.<Supplier<Boolean>, Consumer<Boolean>>of(
            // Cycle to intake
            () -> gamepad1.right_trigger > triggerThreshold,
            (Boolean mode) -> intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD),
            // Cycle to outtake
            () -> gamepad1.left_trigger > triggerThreshold,
            (Boolean mode) -> intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE),
            // Toggle flywheel
            () -> gamepad1.left_bumper,
            (Boolean mode) -> flywheelMotor.setPower((mode) ? flywheelPower : 0.0)
        );
        // Define the hold keybinds
        holdKeybinds = Map.<Supplier<Boolean>, Consumer<Boolean>>of(
            // Power the intake/outtake wheel
            () -> gamepad1.right_trigger > triggerThreshold || gamepad1.left_trigger > triggerThreshold,
            (Boolean mode) -> intakeMotor.setPower((mode) ? intakePower : 0.0),
            // Power the transfer mechanism (launch)
            () -> gamepad1.right_bumper,
            (Boolean mode) -> beltMotor.setPower((mode) ? flywheelPower : 0.0)
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
            /*
            * Handle wheel powering
            * Compute desired power for each wheel given the gamepad inputs
            * Normalize power to ensure nothing exceeds 1.0
            */
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double backLeftPower   = axial - lateral + yaw;

            double frontRightPower = axial - lateral - yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > maxDrivePower) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower * maxDrivePower);
            frontRightDrive.setPower(frontRightPower * maxDrivePower);
            backLeftDrive.setPower(backLeftPower * maxDrivePower);
            backRightDrive.setPower(backRightPower * maxDrivePower);

            // Update control binds
            keybinds.update();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Active Time", "%.1f seconds\n", runtime.seconds());

            telemetry.addData("Drivetrain Max Power", "%4.2f", maxDrivePower);
            telemetry.addData("Front (Left / Right)", "(%4.2f / %4.2f)", frontLeftPower, frontRightPower);
            telemetry.addData("Back (Left / Right)", "(%4.2f / %4.2f)\n", backLeftPower, backRightPower);

            telemetry.addData("Mechanism Speeds (Intake / Belt / Flywheel)", "(%4.2f / %4.2f / %4.2f)", intakePower, beltPower, flywheelPower);
            telemetry.addData("Mechanism Power (Intake / Outtake / Belt / Flywheel)", "(%b / %b / %b)", intakeMotor.getPowerFloat(), beltMotor.getPowerFloat(), flywheelMotor.getPowerFloat());

            telemetry.addData("RT", "Intake");
            telemetry.addData("LT", "Outtake");
            telemetry.addData("RB", "Launch");
            telemetry.addData("LB", "Flywheel Toggle");
            telemetry.update();
        }
    }}
