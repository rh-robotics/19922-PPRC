package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TC {
    public static Trajectory startToCyclePole(SampleMecanumDrive drive) {
        return drive.trajectoryBuilder(new Pose2d(35, -60, Math.toRadians(90)))
                .forward(60)
                .build();
    }
}
