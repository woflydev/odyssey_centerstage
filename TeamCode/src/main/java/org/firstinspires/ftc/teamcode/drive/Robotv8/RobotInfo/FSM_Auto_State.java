package org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo;

public class FSM_Auto_State {
    public enum FSM_RootAutoState {
        BA_PLAY,
        BA_SCANNING,
        B_TURNING_TO_BACKDROP,
        BA_MOVING_TO_BACKDROP,
        A_ALIGNING_WITH_YELLOW_TRANSIT_TRAJECTORY,
        A_ALIGNING_WITH_BACKDROP_FOR_DEPOSIT,
        BA_DEPOSIT_YELLOW,
        B_TURNING_TO_SPIKEMARK,
        BA_MOVING_TO_SPIKEMARK_AND_DEPOSIT_PURPLE,

        // note: cycling is only run on backdrop side
        BA_MOVING_TO_CYCLE,
        BA_INTAKE_PIXELS_FROM_STACK,
        BA_MOVING_BACK_FROM_CYCLE,
        BA_DEPOSIT_WHITE,

        // note: handle finish
        BA_MOVING_TO_PARKING,
        BA_PARKED,
    }

    public enum RobotAlliance {
        BLUE,
        RED,
        NONE,
    }

    public enum RobotStartingPosition {
        BACKDROP,
        AUDIENCE,
    }

    public enum RobotParkingLocation {
        INNER,
        OUTER,
    }

    public enum RobotTaskFinishBehaviour {
        DO_NOT_CYCLE,
        CYCLE,
        CYCLE_TWICE_NONONONONO,
    }

    public enum RobotLocMode {
        CAM,
        MEC,
    }
}

