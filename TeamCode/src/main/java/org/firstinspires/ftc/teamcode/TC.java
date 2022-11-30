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
}