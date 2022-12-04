package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Bronto's TeleOp", group="Iterative Opmode")

public class BrontoTeleOP extends OpMode
{
    /** Declare OpMode members. */
   HWC bronto;
   BrontoBrain brain;

    public enum TeleOpStates {
        RESTING,
        INTAKE,
        DELIVERING, //WIPED LOW,MED,HIGH POLE B/C NO DIFFERENCE SINCE NO SENSORS GET TRIPPED
        MOVING,
        TRANSFER,
        UNKNOWN
    }

    private ElapsedTime runtime = new ElapsedTime();

    boolean manualMode = false;

    int frontArmTarget = 0;
    int backArmTarget = 0;
    int frontElbowTarget = 0;
    int backElbowTarget = 0;

    TeleOpStates state = TeleOpStates.RESTING;
    TeleOpStates nextState = TeleOpStates.UNKNOWN; //determines when a moving state completes and will go to

    @Override
    public void init() {
        bronto = new HWC(hardwareMap, telemetry);
         brain = new BrontoBrain(bronto);
        telemetry.addData("Status", "Initializing");
     //   bronto.frontElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontElbow.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bronto.frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //bronto.frontElbow.setTargetPosition(0);
     //   bronto.frontArm.setTargetPosition(0);
        bronto.backElbow.setTargetPosition(0);
        bronto.backArm.setTargetPosition(0);
     //   bronto.frontElbow.setPower(0.2);
        bronto.frontArm.setPower(0.2);
    //    bronto.backElbow.setPower(0.2);
    //    bronto.backArm.setPower(0.2);

        telemetry.addData("Status", "Initialized");
    }



    /** Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY. */
    @Override
    public void init_loop() {
    }

    /** Code to run ONCE when the driver hits PLAY. */
    @Override
    public void start(){
    runtime.reset();
    ;
    }

    @Override
    public void loop() {
        /* Setup a variable for each drive wheel to save power level for telemetry. */
        double leftFPower ;
        double rightFPower;
        double leftBPower ;
        double rightBPower;
        double frontArmPow;
        double intakePow = 0;
        double frontElbowPow = 0;

        double drive = -gamepad1.left_stick_y *0.8;
        double turn  =  gamepad1.left_stick_x * 0.6;
        double strafe = -gamepad1.right_stick_x * 0.8;
        //double frontArmUp = gamepad2.left_trigger;
        //double frontArmDown = -gamepad2.right_trigger;

        //boolean elbowFOn = false;

        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.left_stick_button){
            bronto.frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}

        /* if (-gamepad2.right_stick_y > 0) { intakePow = gamepad2.right_stick_y;}
        else if (gamepad2.right_stick_y > 0) {intakePow = gamepad2.right_stick_y;}
        else {intakePow = 0;}


        if (gamepad2.right_stick_button){
            manualMode = !manualMode;
        }
         */

        /*
        if (frontArmUp > 0) {
            bronto.frontArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontArmPow = frontArmUp;
        } else if (frontArmDown < 0) {
            bronto.frontArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontArmPow = frontArmDown;
        }
        else{

            frontArmPow = 0;

        }

         */

          if (gamepad2.left_stick_x != 0) {
              frontElbowTarget += (gamepad2.left_stick_x * 10);
          } else if (gamepad2.left_stick_y != 0) {
              backElbowTarget += (gamepad2.left_stick_y * 10);
          }

          if (gamepad2.right_stick_x != 0) {
              frontArmTarget += (gamepad2.right_stick_x * 100);
          } else if (gamepad2.right_stick_y != 0) {
              backArmTarget += (gamepad2.right_stick_y * 100);
          }
          /*else if (gamepad2.left_stick_y < 0) {
              bronto.frontElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
              frontElbowPow = -gamepad2.left_stick_y;
              bronto.frontElbow.setTargetPosition(bronto.frontElbow.getCurrentPosition());

          } else {
              bronto.frontElbow.setTargetPosition(bronto.frontElbow.getCurrentPosition());
              bronto.frontElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              if (bronto.frontElbow.getCurrentPosition() - bronto.frontElbow.getTargetPosition() > 10 || bronto.frontElbow.getCurrentPosition() - bronto.frontElbow.getTargetPosition() < 10){
              frontElbowPow = 0.6;}
              else {frontElbowPow = 0.45;}
          }
          */


    //      bronto.frontElbow.setMode(manualMode ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_TO_POSITION);
/*
            if (manualMode) {
                frontElbowPow = 0;
            } else {
                if (bronto.frontElbow.getTargetPosition() - 10 < bronto.frontElbow.getCurrentPosition() && bronto.frontElbow.getTargetPosition() + 10 > bronto.frontElbow.getCurrentPosition())
                    frontElbowPow = 0.75;
                else {
                    frontElbowPow = 1;
                }
            }

*/
                if (drive != 0 || turn != 0) {
                    leftFPower = Range.clip(drive + turn, -1.0, 1.0);
                    rightFPower = Range.clip(drive - turn, -1.0, 1.0);
                    leftBPower = Range.clip(drive + turn, -1.0, 1.0);
                    rightBPower = Range.clip(drive - turn, -1.0, 1.0);
                } else if (strafe != 0) {
                    /* Strafing */
                    leftFPower = -strafe;
                    rightFPower = strafe;
                    leftBPower = strafe;
                    rightBPower = -strafe;
                } else {
                    leftFPower = 0;
                    rightFPower = 0;
                    leftBPower = 0;
                    rightBPower = 0;
                }

/*if(colorSensor.green() > 138 && colorSensor.red() > 138 && colorSensor.green() >colorSensor.red()){
    telemetry.addData("color", "Yellow Detected!");
}*/
                if (gamepad2.y) {
                    state = TeleOpStates.MOVING;
                    nextState = TeleOpStates.TRANSFER;
                    frontElbowTarget = bronto.elbowTransferPos;
                    backElbowTarget = bronto.backElbowTransferPos;
                    frontArmTarget = bronto.transferPos;
                    backArmTarget = bronto.backHighPolePos;
                    /* bronto.move_to_position_and_hold(bronto.frontArm, 1, bronto.transferPos);
                    bronto.move_to_position_and_hold(bronto.frontElbow, .5, bronto.elbowTransferPos);
                    bronto.move_to_position_and_hold(bronto.backArm, 1, bronto.backHighPolePos);
                    bronto.move_to_position_and_hold(bronto.backElbow, .5, bronto.backElbowTransferPos);
                    intakePow = 1;
                    state = TeleOpStates.TRANSFER;
                     */

                } else if (gamepad1.x) {
                    state = TeleOpStates.MOVING;
                    nextState = TeleOpStates.DELIVERING;
                    frontElbowTarget = bronto.elbowIntakePos;
                    backElbowTarget = bronto.backElbowDeliveryPosHigh;
                    frontArmTarget = bronto.intakePos;
                    backArmTarget = bronto.backHighPolePos;
                    /*
                    bronto.move_to_position_and_hold(bronto.frontArm, 1, bronto.highPolePos);
                    bronto.move_to_position_and_hold(bronto.frontElbow, 0.5, bronto.elbowDeliveryPosHigh);
                    state = TeleOpStates.DELIVERING;

                     */

                } else if (gamepad1.b) {
                    bronto.move_to_position_and_hold(bronto.frontArm, 1, bronto.medPolePos);
                    bronto.move_to_position_and_hold(bronto.frontElbow, 0.5, bronto.elbowDeliveryPosMed);
                    state = TeleOpStates.DELIVERING;
                } else if (gamepad1.a) {
                    bronto.move_to_position_and_hold(bronto.frontArm, 1, bronto.lowPolePos);
                    bronto.move_to_position_and_hold(bronto.frontElbow, 0.5, bronto.elbowDeliveryPosLow);
                    state = TeleOpStates.DELIVERING;
                }

                //setting next state
                switch (state) {
                    case RESTING:
                        telemetry.addData("Arm Position", "Resting");

                        break;
                    case DELIVERING:
                        telemetry.addData("Arm Position", "High Pole");
                        //TODO: MAKE OUTTAKE pwr
                        intakePow = -1;
                        //TODO: if (cone color sensor gets tripped i.e. cone was outtaked)
                        intakePow = 0;
                        state = TeleOpStates.UNKNOWN;

                        break;
                    case INTAKE:
                        telemetry.addData("Arm Position", "Intake");
                        //TODO: MAKE INTAKE PWR
                        intakePow = 1;
                        //if (cone color sensor gets tripped i.e. cone was intaked)
                            intakePow = 0;
                            state = TeleOpStates.UNKNOWN;
                        break;
                    case MOVING:
                        telemetry.addData("Arm Position", "Moving");
                        if (bronto.closeEnough (bronto.frontElbow.getCurrentPosition(), frontElbowTarget, 2) &&
                                bronto.closeEnough (bronto.backElbow.getCurrentPosition(), backElbowTarget, 2) &&
                                bronto.closeEnough (bronto.frontArm.getCurrentPosition(), frontArmTarget, 15) &&
                                bronto.closeEnough (bronto.backArm.getCurrentPosition(), backArmTarget, 15)) {
                            state = nextState;
                            nextState = TeleOpStates.UNKNOWN;
                        }

                        break;
                    case TRANSFER:
                        telemetry.addData("Arm Position", "Transfer");
                        //TODO: if (color sensor detects other side)
                        intakePow = 1;
                            //TODO: if (cone color sensor gets tripped i.e. cone got to other side)
                            intakePow = 0;
                            state = TeleOpStates.UNKNOWN;
                        break;
                    case UNKNOWN:
                        telemetry.addData("Arm Position", "Unknown");

                        break;
                    default:
                        state = TeleOpStates.UNKNOWN;
                        telemetry.addData("Arm Position", "Unknown");

                }


                bronto.leftFront.setPower(leftFPower);
                bronto.leftRear.setPower(leftBPower);
                bronto.rightFront.setPower(rightFPower);
                bronto.rightRear.setPower(rightBPower);
                bronto.frontIntakeL.setPower(intakePow);
                bronto.frontIntakeR.setPower(intakePow);
                //if arm motors are close enough, set to 0 b/c power draw and worm gear already holds it
                if (bronto.closeEnough(bronto.frontArm.getCurrentPosition(), frontArmTarget, 15)) {
                    bronto.frontArm.setPower(0);
                } else bronto.frontArmComponent.moveUsingPID(frontArmTarget);
                if (bronto.closeEnough(bronto.backArm.getCurrentPosition(), backArmTarget, 15)) {
                    bronto.backArm.setPower(0);
                } else bronto.backArmComponent.moveUsingPID(backArmTarget);
                bronto.frontElbowComponent.moveUsingPID(frontElbowTarget);
                bronto.backElbowComponent.moveUsingPID(backElbowTarget);

                telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f), front elbow (%.2f),  ", leftFPower, rightFPower, leftBPower, rightBPower, bronto.frontArm.getPower(), bronto.frontElbow.getPower());
                telemetry.update();

            }

            /** Code to run ONCE after the driver hits STOP. */
            @Override
            public void stop () {
            }
        }