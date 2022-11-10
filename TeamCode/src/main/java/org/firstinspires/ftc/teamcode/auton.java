package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class auton extends LinearOpMode {
  // Variables
  HWC.autonStates state = HWC.autonStates.SCANNING_FOR_SIGNAL;
  HWC.armPositions armPosition = HWC.armPositions.RESTING;
  ElapsedTime timer = new ElapsedTime();
  Pose2d startPos = new Pose2d(35, -60, Math.toRadians(90));
  int cycleCount = 0;
  int parkingZone = 0;

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
    telemetry.addData("State", "Scanning for Signal");
    telemetry.addData("Arm Position", "Init");
    telemetry.update();

    brain.cv();
    while (!isStarted()) {
      telemetry.addData("ROTATION: ", bronto.sleeveDetection.getPosition());
      telemetry.update();
    }

    waitForStart();
    state = HWC.autonStates.MOVING_TO_POLE;
    while (opModeIsActive()) {
      // Update State Telemetry
      telemetry.addData("Status", "Running");
      telemetry.update();

      // State Machine
      switch(state) {
        case MOVING_TO_POLE:
          // Update cycle count & telemetry
          telemetry.addData("State", "Moving to Pole");
          telemetry.addData("Cycle Count", cycleCount);
          telemetry.update();
          cycleCount++;

          drive.setPoseEstimate(startPos);

          // Move arms to cycle pos & update telemetry

          double highPos = 2786 * 0.75;
          bronto.move_to_position_and_hold(bronto.frontArm ,.3, (int)highPos);
          telemetry.addData("Arm Position", "Cycle");
          telemetry.update();


          // Drive to pole, then rotate
          drive.followTrajectory(TC.startToCyclePole(drive, startPos));
          drive.turn(Math.toRadians(90));

          //Change state
          state = HWC.autonStates.DELIVERING_CONE;
        case DELIVERING_CONE:
          // Update State Telemetry
          telemetry.addData("State", "Delivering Cone");
          telemetry.update();

          // Run Gecko Wheel Servos to deposit cone
          bronto.runIntakeServo('R',-.3);

          if (cycleCount <= 3) {
            state = HWC.autonStates.MOVING_TO_STACK;
          } else {
            if (parkingZone > 0) {
              state = HWC.autonStates.PARKING_VALUE;
            } else {
              state = HWC.autonStates.PARKING_NO_VALUE;
            }
          }
        case MOVING_TO_STACK:
          // Update State Telemetry
          telemetry.addData("State", "Moving to Stack");
          telemetry.update();

          // Run Move to Stack Trajectory
          TC.poleToStack(drive, TC.startToCyclePole(drive, startPos).end().plus(new Pose2d(0, 0, Math.toRadians(90))));

          if (!drive.isBusy()) {
            state = HWC.autonStates.PICKING_UP_CONE;
          }
        case PICKING_UP_CONE:
          // Update State Telemetry
          telemetry.addData("State", "Picking up Cone");
          telemetry.update();

          switch(cycleCount) {
            case 0:
              // Move arm to position of first cone
            case 1:
              // Move arm to position of second cone
            case 2:
              // Move arm to position of third cone
            case 3:
              // Move arm to position of fourth cone
          }
        case PARKING_NO_VALUE:
          // Update State Telemetry
          telemetry.addData("State", "Parking no Value");
          telemetry.update();
        case PARKING_VALUE:
          // Update State Telemetry
          telemetry.addData("State", "Parking with Value");
          telemetry.update();
      }
    }
  }
}
// I teo cannot write good code without stealing it from the internet