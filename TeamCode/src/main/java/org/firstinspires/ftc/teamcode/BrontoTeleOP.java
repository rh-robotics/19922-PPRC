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
        LOW_POLE,
        MED_POLE,
        HIGH_POLE,
        TRANSFER,
        UNKNOWN
    }

    private ElapsedTime runtime = new ElapsedTime();

    boolean manualMode = false;

    TeleOpStates state = TeleOpStates.RESTING;

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
        double intakePow;
        double frontElbowPow = 0;

        double drive = -gamepad1.left_stick_y *0.8;
        double turn  =  gamepad1.left_stick_x * 0.6;
        double strafe = -gamepad1.right_stick_x * 0.8;
        double frontArmUp = gamepad2.left_trigger;
        double frontArmDown = -gamepad2.right_trigger;

        boolean elbowFOn = false;

        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.left_stick_button){
            bronto.frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}

        if (-gamepad2.right_stick_y > 0) { intakePow = gamepad2.right_stick_y;}
        else if (gamepad2.right_stick_y > 0) {intakePow = gamepad2.right_stick_y;}
        else {intakePow = 0;}


        if (gamepad2.right_stick_button){
            manualMode = !manualMode;
        }

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


          if (-gamepad2.left_stick_y > 0) {
              bronto.frontElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
              frontElbowPow = -gamepad2.left_stick_y;
              bronto.frontElbow.setTargetPosition(bronto.frontElbow.getCurrentPosition());
          } else if (gamepad2.left_stick_y < 0) {
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


    bronto.frontElbow.setMode(manualMode ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_TO_POSITION);
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
                    bronto.move_to_position_and_hold(bronto.frontElbow, .5, 200);
                    state = TeleOpStates.TRANSFER;
                } else if (gamepad1.x) {

                    bronto.move_to_position_and_hold(bronto.frontArm, .8, bronto.highPolePos);
                    bronto.move_to_position_and_hold(bronto.frontElbow, 0.5, 0);
                    state = TeleOpStates.HIGH_POLE;

                } else if (gamepad1.b) {
                    bronto.smartMove(HWC.armPositions.MED_POLE);
                    state = TeleOpStates.MED_POLE;
                } else if (gamepad1.a) {
                    // brain.mainCycle(1);
                    // bronto.frontElbow.setPower(0.4);
                }

                switch (state) {
                    case RESTING:
                        telemetry.addData("Arm Position", "Resting");

                        break;
                    case LOW_POLE:
                        telemetry.addData("Arm Position", "Low Pole");

                        break;
                    case MED_POLE:
                        telemetry.addData("Arm Position", "Medium Pole");

                        break;
                    case HIGH_POLE:
                        telemetry.addData("Arm Position", "High Pole");

                        break;
                    case INTAKE:
                        telemetry.addData("Arm Position", "Intake");

                        break;
                    case TRANSFER:
                        telemetry.addData("Arm Position", "Transfer");

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
                bronto.frontArm.setPower(frontArmPow);
                bronto.frontIntakeL.setPower(intakePow);
                bronto.frontIntakeR.setPower(intakePow);
                bronto.frontElbow.setPower(frontElbowPow);

                telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f), front elbow (%.2f),  ", leftFPower, rightFPower, leftBPower, rightBPower, bronto.frontArm.getPower(), bronto.frontElbow.getPower());
                telemetry.update();

            }

            /** Code to run ONCE after the driver hits STOP. */
            @Override
            public void stop () {
            }
        }