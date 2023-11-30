package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Robotv8.FSM_Auto_Fullstack;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Auto_State.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Auto_State.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Auto_State.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Auto_State.RobotTaskFinishBehaviour;
import org.opencv.core.Point;

@Config
@Autonomous(name="NAT_RedAudienceAuto_OUTER", group="Final")
public class AC2303AO_AutoRed extends FSM_Auto_Fullstack {
    public static double r1x = 25;
    public static double r1y = 90;
    public static double r2x = 150;
    public static double r2y = 60;
    public static double r3x = 260;
    public static double r3y = 90;

    public AC2303AO_AutoRed() {
        super(
                RobotAlliance.RED,
                RobotStartingPosition.AUDIENCE,
                RobotParkingLocation.OUTER,
                RobotTaskFinishBehaviour.DO_NOT_CYCLE,
                new Point(r1x, r1y),
                new Point(r2x, r2y),
                new Point(r3x, r3y)
        );
    }
}