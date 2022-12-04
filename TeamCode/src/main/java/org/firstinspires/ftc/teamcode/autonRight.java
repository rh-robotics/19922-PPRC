package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class autonRight extends LinearOpMode {
  // Variables
  HWC.autonStates state = HWC.autonStates.SCANNING_FOR_SIGNAL;
  HWC.armPositions frontArmPosition = HWC.armPositions.RESTING;
  HWC.armPositions backArmPosition = HWC.armPositions.RESTING;
  ElapsedTime timer = new ElapsedTime();

  @Override
  public void runOpMode() throws InterruptedException {
    // Tell driver bronto is initializing
    telemetry.addData("Status", "Initializing");
    telemetry.update();

    // Run any initialization code necessary
    HWC bronto = new HWC(hardwareMap, telemetry);
    BrontoBrain brain = new BrontoBrain(bronto);

    // Tell driver bronto is ready and waiting for start
    telemetry.addData("Status", "Initialized - Waiting for Start");
    telemetry.addData("State", state);
    telemetry.addData("Front Arm Position", frontArmPosition);
    telemetry.addData("Back Arm Position", backArmPosition);
    telemetry.update();

    brain.cv();
    while (!isStarted()) {
      telemetry.addData("ROTATION: ", bronto.sleeveDetection.getPosition());
      telemetry.update();
      // TODO: Waiting for jack to change cv to return an int instead of enum
//      bronto.parkingZone = bronto.sleeveDetection.getPosition();
    }

    waitForStart();

    // Set our estimated start position for Roadrunner
    bronto.drive.setPoseEstimate(bronto.START_POS_RIGHT);

    state = HWC.autonStates.MOVING_TO_POLE;
    while (opModeIsActive()) {
      // Update State Telemetry
      telemetry.addData("Status", "Running");
      telemetry.addData("State", state);
      telemetry.addData("Front Arm Position", frontArmPosition);
      telemetry.addData("Back Arm Position", backArmPosition);
      telemetry.update();

      // State Machine
      switch(state) {
        case MOVING_TO_POLE:

          if (bronto.cycleCount < 1) {
            // Move arms to cycle pos & update armPositions
            bronto.move_to_position_and_hold(bronto.frontElbow, .5, bronto.elbowIntakePos);
            bronto.move_to_position_and_hold(bronto.backElbow, .5, bronto.elbowIntakePos);
            bronto.move_to_position_and_hold(bronto.frontArm, .5, bronto.highPolePos);
            bronto.move_to_position_and_hold(bronto.backArm, .5, bronto.intakePos);

            frontArmPosition = HWC.armPositions.HIGH_POLE;
            backArmPosition = HWC.armPositions.INTAKE;

            // Run startToPole trajectory
            bronto.drive.followTrajectory(TC.RIGHT_startToCyclePole(bronto.drive, bronto.START_POS_RIGHT));
            bronto.drive.turn(Math.toRadians(90));

            //Change state
          } else {
            // Run stack to pole trajectory
            TC.RIGHT_stackToPole(bronto.drive, TC.RIGHT_startToCyclePole(bronto.drive, bronto.START_POS_RIGHT).end().plus(new Pose2d(0, 0, Math.toRadians(90)))).end();

            // TODO: Transfer cone, then move arms to cycle position

            // Change State
          }
          state = HWC.autonStates.DELIVERING_CONE;

        case DELIVERING_CONE:
          // Run Gecko Wheel Servos to deposit cone
          bronto.runIntakeServo('R',-.3);
          sleep(2000);
          bronto.runIntakeServo('R', 0);

          // Increase cycleCount value
          bronto.cycleCount++;

          // Check if the cycle count values and move to parking or do another cycle
          if (bronto.cycleCount <= 3) {
            state = HWC.autonStates.MOVING_TO_STACK;
          } else {
            if (bronto.parkingZone > 0) {
              state = HWC.autonStates.PARKING_VALUE;
            } else {
              state = HWC.autonStates.PARKING_NO_VALUE;
            }
          }

        case MOVING_TO_STACK:
          // Update State Telemetry
          telemetry.addData("State", "Moving to Stack");
          telemetry.update();

          // If cycleCount is less than 1 run startToPole trajectory, and then change states
          TC.RIGHT_poleToStack(bronto.drive, TC.RIGHT_startToCyclePole(bronto.drive, bronto.START_POS_RIGHT).end().plus(new Pose2d(0, 0, Math.toRadians(90))));

          // Change state
          state = HWC.autonStates.PICKING_UP_CONE;

        case PICKING_UP_CONE:
          // Update State Telemetry
          telemetry.addData("State", "Picking up Cone");
          telemetry.update();

          switch(bronto.cycleCount) {
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

          // TODO: Run trajectory to go to preset position

        case PARKING_VALUE:
          // Update State Telemetry
          telemetry.addData("State", "Parking with Value");
          telemetry.update();

          // TODO: Check int to see which zone we go to
      }

      if (timer.milliseconds() >= 20000) {
        // TODO: Run trajectory that navigates to the parking zone as a bailout
      }
    }
  }
}