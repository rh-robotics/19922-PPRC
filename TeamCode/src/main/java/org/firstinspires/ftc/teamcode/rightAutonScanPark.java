package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class rightAutonScanPark extends LinearOpMode {
    // Variables
    HWC.autonStates state = HWC.autonStates.SCANNING_FOR_SIGNAL;
    HWC.armPositions frontArmPosition = HWC.armPositions.RESTING;
    HWC.armPositions backArmPosition = HWC.armPositions.RESTING;
    ElapsedTime timer = new ElapsedTime();

    // Target Pos
    int frontArmTarget, frontElbowTarget, backArmTarget, backElbowTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        // Tell driver bronto is initializing
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Run any initialization code necessary
        HWC bronto = new HWC(hardwareMap, telemetry);
        BrontoBrain brain = new BrontoBrain(bronto);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // Tell driver bronto is ready and waiting for start
        telemetry.addData("Status", "Initialized - Waiting for Start");
        telemetry.update();

        brain.cv();
        while (!isStarted()) {
            telemetry.addData("Status", "Scanning for Cone");
            telemetry.addData("Parking Position", bronto.sleeveDetection.getPosition());
            telemetry.update();

            bronto.parkingZone = bronto.sleeveDetection.getPosition();
        }

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
        // Set our estimated start position for Roadrunner
        bronto.drive.setPoseEstimate(bronto.START_POS_RIGHT);
        if (bronto.parkingZone != 0) {
            drive.followTrajectory(TC.SCANPARK_forward(drive, bronto.START_POS_RIGHT));
            telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f)", bronto.leftFront.getPower(), bronto.rightFront.getPower(), bronto.leftRear.getPower(), bronto.rightRear.getPower());
            telemetry.update();

        } else {
            telemetry.addData("ParkingZone", "ERROR");
            telemetry.update();
        }

        if (bronto.parkingZone == 1) {
            drive.followTrajectory(TC.SCANPARK_strafeLeft(drive, TC.SCANPARK_forward(drive, bronto.START_POS_RIGHT).end()));
        } else if (bronto.parkingZone == 3) {
            drive.followTrajectory(TC.SCANPARK_strafeRight(drive, TC.SCANPARK_forward(drive, bronto.START_POS_RIGHT).end()));
        }
    }
}