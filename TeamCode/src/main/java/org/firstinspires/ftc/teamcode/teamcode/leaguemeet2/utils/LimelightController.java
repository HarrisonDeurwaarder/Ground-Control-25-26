package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import androidx.annotation.Nullable;

public class LimelightController {
    // Define constants
    private static final int POLL_RATE = 100; // Hz
    private static final int OBELISK_ID1 = 0;
    private static final int OBELISK_ID2 = 0;
    private static final int OBELISK_ID3 = 0;
    private static final int BLUE_GOAL_ID = 0;
    private static final int RED_GOAL_ID = 0;

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

    public void updateResult() {
        // Query and save the latest update
        result = limelight.getLatestResult();
    }

    public int updatePattern() throws NullPointerException {
        // Loop through all detections
        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            // If the obelisk is in frame, return that ID
            int id = fiducial.getFiducialId();
            if (id == OBELISK_ID1 || id == OBELISK_ID2 || id == OBELISK_ID3) {
                // Return detected ID if it is in the set of obelisk IDs
                return id;
            }
        }
        // Else, indicate that no ID was found
        return -1;
    }

    @Nullable // so the function can return null
    public LLResultTypes.FiducialResult getGoalOffset(String team) throws NullPointerException, IllegalArgumentException {
        // Determine the ID of the tag for the given team
        int goalID;
        if (team.equals("BLUE")) {
            goalID = BLUE_GOAL_ID;
        } else if (team.equals("RED")) {
            goalID = RED_GOAL_ID;
        } else {
            throw new IllegalArgumentException("Team is invalid");
        }

        // Loop through all detections
        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            // If the goal is in frame
            if (fiducial.getFiducialId() == goalID) {
                return fiducial;
            }
        }

        // Else, return null to indicate that no goal fiducial was detected
        return null;
    }
}