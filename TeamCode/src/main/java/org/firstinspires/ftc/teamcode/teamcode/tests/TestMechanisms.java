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

package org.firstinspires.ftc.teamcode.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * OpMode to test the functionality of non-wheel mechanisms (intake, belt, flywheel)
 *
 * ONLY powers these mechanisms, no driving is done
 */

@Disabled
@TeleOp(name="Mechanism Test", group="Tests")
//@Disabled
public class TestMechanisms extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Mechanism motors
    private DcMotor intakeMotor;
    private DcMotor beltMotor;
    private DcMotor flywheelMotor;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        beltMotor = hardwareMap.get(DcMotor.class, "transport");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheel");

        // Previous button statuses are stored to prevent over-toggling
        boolean prevIntakeButton = false;
        boolean prevBeltButton = false;
        boolean prevFlywheelButton = false;

        // Previous button statuses are stored to prevent over-toggling
        boolean intakeToggle = false;
        boolean beltToggle = false;
        boolean flywheelToggle = false;

        // These directions WILL need to be changed
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        beltMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until driver presses STOP
        while (opModeIsActive()) {

            // Forward push changes the speed of components
            double motorPower   = -gamepad2.left_stick_y;

            // Corresponding button presses active given motors for tests
            boolean intakeButton = gamepad2.a;
            boolean beltButton = gamepad2.b;
            boolean flywheelButton = gamepad2.x;

            // Update toggles
            intakeToggle = (!prevIntakeButton && intakeButton) != intakeToggle;
            beltToggle = (!prevBeltButton && beltButton) != beltToggle;
            flywheelToggle = (!prevFlywheelButton && flywheelButton) != flywheelToggle;

            // Send power to motors (only if they are toggled ON)
            intakeMotor.setPower((intakeToggle) ? motorPower : 0.0);
            beltMotor.setPower((beltToggle) ? motorPower : 0.0);
            flywheelMotor.setPower((flywheelToggle) ? motorPower : 0.0);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Intake Toggle (A)", intakeToggle);
            telemetry.addData("Intake Toggle (B)", beltToggle);
            telemetry.addData("Intake Toggle (X)", flywheelToggle);
            telemetry.addData("Power", "%4.2f", motorPower);
            telemetry.update();

            // Update previous button statuses
            prevIntakeButton = intakeButton;
            prevBeltButton = beltButton;
            prevFlywheelButton = flywheelButton;
        }
    }}
