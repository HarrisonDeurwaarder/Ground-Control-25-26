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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils.GamepadBindings;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils.GamepadBindingsCfg;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils.MotorDriverPID;

/*
 * This OpMode runs a manual omni-directional drivetrain
 */

@Autonomous(name="League Meet 1 (3BAuto)", group="League Meet 1")
public class Auto3BLeagueMeet1 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private MotorDriverPID motorDriver;

    private static final double TRAVEL_DISTANCE = 25.0; // Inches

    // Compute desired tick distance
    private static final int TRAVEL_TICKS = (int) Math.round((MotorDriverPID.TPR * TRAVEL_DISTANCE) / (Math.PI * MotorDriverPID.WHEEL_DIAMETER * MotorDriverPID.GEAR_RATIO));

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Instanciate a motor driver using the now non-null hardwareMap
        motorDriver = new MotorDriverPID(hardwareMap);

        // Set to run to distance for each drivetrain motor
        motorDriver.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriver.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriver.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDriver.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        // Or there are no more steps to execute
        while (opModeIsActive()) {
            // Enable flywheel for duration of auto
            motorDriver.flywheelMotor.setVelocity(
                    MotorDriverPID.toTPS(MotorDriverPID.FLYWHEEL_RPS)
            );
            // Set goal to all drivetrain motors
            motorDriver.frontRightDrive.setTargetPosition(TRAVEL_TICKS);
            motorDriver.frontLeftDrive.setTargetPosition(TRAVEL_TICKS);
            motorDriver.backRightDrive.setTargetPosition(TRAVEL_TICKS);
            motorDriver.backLeftDrive.setTargetPosition(TRAVEL_TICKS);

            motorDriver.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriver.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriver.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDriver.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // If position has been reached, start feeding
            if ((double) (motorDriver.frontRightDrive.getCurrentPosition() / motorDriver.frontRightDrive.getTargetPosition()) >= 0.95) {
                // Enable feeding until auto ends
                motorDriver.transportMotor.setVelocity(
                        (motorDriver.flywheelInRange()) ? MotorDriverPID.toTPS(MotorDriverPID.TRANSPORT_RPS) : 0.0
                );
                motorDriver.intakeMotor.setPower(
                        MotorDriverPID.INTAKE_POWER
                );
                // Stop motors
                motorDriver.frontRightDrive.setPower(0.0);
                motorDriver.frontLeftDrive.setPower(0.0);
                motorDriver.backRightDrive.setPower(0.0);
                motorDriver.backLeftDrive.setPower(0.0);

            // If position has not been reached
            } else {
                // Set the power for drive
                motorDriver.frontRightDrive.setPower(0.6);
                motorDriver.frontLeftDrive.setPower(0.6);
                motorDriver.backRightDrive.setPower(0.6);
                motorDriver.backLeftDrive.setPower(0.6);
            }

            // TELEMETRY
            telemetry.addData("Active Time", "%.1f seconds\n", runtime.seconds());

            telemetry.addData("Target Distance", "%d", TRAVEL_TICKS);
            telemetry.addData("Current Distance", "%d", motorDriver.frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front RPS (Left / Right)", "(%4.2f / %4.2f)", MotorDriverPID.toRPS(motorDriver.frontLeftDrive.getVelocity()), MotorDriverPID.toRPS(motorDriver.frontRightDrive.getVelocity()));
            telemetry.addData("Back RPS (Left / Right)", "(%4.2f / %4.2f)", MotorDriverPID.toRPS(motorDriver.backLeftDrive.getVelocity()), MotorDriverPID.toRPS(motorDriver.backRightDrive.getVelocity()));
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
