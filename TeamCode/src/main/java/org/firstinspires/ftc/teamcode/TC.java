package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TC {
    public static Trajectory RIGHT_startToCyclePole(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .forward(60)
                .build();
    }

    public static Trajectory RIGHT_poleToStack(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .forward(10)
                .build();
    }

    public static Trajectory RIGHT_stackToPole(SampleMecanumDrive drive, Pose2d pos) {
        // TODO: Write trajectory
        return null;
    }

    public static Trajectory SCANPARK_forward(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .forward(24)
                .build();
    }

    public static Trajectory SCANPARK_strafeLeft(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .strafeLeft(24)
                .build();
    }

    public static Trajectory SCANPARK_strafeRight(SampleMecanumDrive drive, Pose2d pos) {
        return drive.trajectoryBuilder(pos)
                .strafeRight(24)
                .build();
    }
}