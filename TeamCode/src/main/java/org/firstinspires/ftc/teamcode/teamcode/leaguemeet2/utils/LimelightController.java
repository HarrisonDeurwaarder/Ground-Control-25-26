package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimelightController {
    // Define constants
    private static final int POLL_RATE = 100; // Hz
    private static final int OBELISK_ID1 = 21; // GPP
    private static final int OBELISK_ID2 = 22; // PGP
    private static final int OBELISK_ID3 = 23; // PPG
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID = 24;

    // Declare variables
    private HardwareMap hardwareMap;
    private Limelight3A limelight;
    private LLResult result;

    private LimelightController() {
        // Map and set the limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Set poll rate
        limelight.setPollRateHz(POLL_RATE);
        limelight.start();
    }

    public LLResultTypes.FiducialResult updateResult(int goalID) {
        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial
                if(id == goalID) {
                    return fiducial;
                }
            }
        }
        return null;
    }

    public double getTargetDist(double targetArea) {
        double scale = 14.76;
        return scale/Math.sqrt(targetArea);
    }



}