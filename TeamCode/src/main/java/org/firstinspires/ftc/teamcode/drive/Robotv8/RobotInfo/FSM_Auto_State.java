package org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo;

public class FSM_Auto_State {
    public enum FSM_RootAutoState {
        PLAY,
        SCANNING,
        TURNING_TO_BACKDROP,
        MOVING_TO_BACKDROP,
        DEPOSIT_YELLOW,
        TURNING_TO_SPIKEMARK,
        MOVING_TO_SPIKEMARK_AND_DEPOSIT_PURPLE,
        MOVING_TO_CYCLE,
        INTAKE_PIXELS_FROM_STACK,
        MOVING_BACK_FROM_CYCLE,
        DEPOSIT_WHITE,
        MOVING_TO_PARKING,
        PARKED,
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
    }
}

