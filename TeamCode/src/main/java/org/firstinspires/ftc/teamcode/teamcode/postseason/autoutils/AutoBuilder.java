package org.firstinspires.ftc.teamcode.teamcode.postseason.autoutils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import java.util.ArrayList;

public class AutoBuilder {
    private Follower follower;
    private final ArrayList<Cycle> cycles = new ArrayList<>();
    private Timer cycleTimer;
    private int cycleState;

    public AutoBuilder(Follower follower) {
        // initialize timer
        cycleTimer = new Timer();
        // reset cycleState and timer
        cycleState = 0;
        cycleTimer.resetTimer();

        this.follower = follower;
    }

    public double getCycleTime() { return cycleTimer.getElapsedTimeSeconds(); }

    public double getCycleState() { return cycleState; }

    public Cycle getCurrentCycle() { return cycles.get(cycleState); }

    public int getAutoLength() { return cycles.size(); }

    public void addCycle(Cycle cycle) { cycles.add(cycle); }

    /**
     * Take an autonomous step
     *
     * @return whether the autonomous cycles have concluded
     */
    public boolean update() {
        // exit if invalid cycleState at entry
        if (cycleState == -1) return true;
        // execute next path and record termination
        boolean isCycleOver = cycles.get(cycleState).nextPath();
        // Increment cycle state if terminated
        if (isCycleOver) incrementCycleState();
        return cycleState == -1;
    }

    /**
     * Reset auto timers and paths
     */
    public void reset() {
        // reset cycleState
        cycleState = 0;
        // reset timers
        cycleTimer.resetTimer();
        getCurrentCycle().reset();
    }

    private void incrementCycleState() {
        // increment counter or set to -1 on > n_paths case
        cycleState++;
        if (cycleState >= getAutoLength()) cycleState = -1;
        // reset timer
        cycleTimer.resetTimer();
    }
}
