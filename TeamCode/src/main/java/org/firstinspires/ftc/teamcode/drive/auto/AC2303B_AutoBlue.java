package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Robotv8.AutoBase;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RR_AutoBase2;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RR_DRIVE_ONLY_AutoBase;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotStartingPosition;
import org.opencv.core.Point;

@Config
@Autonomous(name="RR_DUALPIXEL_TEST", group="Final")
public class AC2303B_AutoBlue extends RR_AutoBase2 {
    public static double r1x = 25;
    public static double r1y = 90;
    public static double r2x = 150;
    public static double r2y = 60;
    public static double r3x = 260;
    public static double r3y = 90;

    public AC2303B_AutoBlue() {
        super(
                RobotAlliance.BLUE,
                RobotStartingPosition.BACKDROP,
                RobotParkingLocation.INNER,
                new Point(r1x, r1y),
                new Point(r2x, r2y),
                new Point(r3x, r3y)
        );
    }
}