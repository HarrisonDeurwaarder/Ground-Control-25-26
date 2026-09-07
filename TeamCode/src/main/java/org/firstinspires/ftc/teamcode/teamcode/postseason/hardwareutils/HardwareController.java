package org.firstinspires.ftc.teamcode.teamcode.postseason.hardwareutils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teamcode.state.PIDController;

@Config
public class HardwareController {

    // declare actuators
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    // declare more as needed
    // ...
    public Limelight3A limelight;

    // PID constants
    public static PIDController PIDexample = new PIDController(0.1, 0.0, 0.0, 0.01, 0.003);
    // add more PIDs as needed
    // ...

    /**
     * Instantiate devices and set all to default configuration
     *
     * @param hardwareMap HardwareMap object
     */
    public HardwareController(HardwareMap hardwareMap) {
        // map drivetrain motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // map limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // add more devices as needed
        // ...

        setAllToDefault();
    }

    /**
     * Set all devices to default configuration (direction, usage of encoder, etc)
     */
    private void setAllToDefault() {
        // set poll rate
        limelight.setPollRateHz(100);
        limelight.start();

        // set device direction, encoder usage, and zero power behavior (i.e. braking) here
        // do not set drivetrain configuration; pedro will handle this
        // ...

    }

    // continue adding methods that pertain to the devices
    // ...
}