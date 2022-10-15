package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TF {
    public Trajectory autonDelPreloadCone(SampleMecanumDrive drive) {
        return drive.trajectoryBuilder(new Pose2d(35, -60, Math.toRadians(90)))
                .forward(45)
                .lineToLinearHeading(new Pose2d(45, 0, Math.toRadians(180)))
                .build();
    }

    public Trajectory autonDelPreloadCone2(SampleMecanumDrive drive) {
        return drive.trajectoryBuilder(new Pose2d(35, -60, Math.toRadians(90)))
                .forward(45)
                .lineToLinearHeading(new Pose2d(45, 0, Math.toRadians(180)))
                .build();
    }
}
