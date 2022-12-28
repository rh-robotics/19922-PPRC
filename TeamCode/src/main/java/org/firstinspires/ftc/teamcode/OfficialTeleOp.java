package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="19922-Official TeleOp", group="Iterative Opmode")

public class OfficialTeleOp extends OpMode
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
        bronto.frontElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        bronto.leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        bronto.rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        bronto.rightRear.setDirection(DcMotorEx.Direction.REVERSE);
        bronto.frontElbow.setTargetPosition(0);
        bronto.frontArm.setTargetPosition(0);
        bronto.backElbow.setTargetPosition(0);
        bronto.backArm.setTargetPosition(0);
        bronto.frontElbow.setPower(0.8);
        bronto.frontArm.setPower(0.8);
        bronto.backElbow.setPower(0.8);
        bronto.backArm.setPower(0.8);

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
        double frontElbowPow;
        double backArmPow;
        double backElbowPow;

        double drive = -gamepad1.left_stick_y *0.8;
        double turn  =  gamepad1.left_stick_x * 0.6;
        double strafe = -gamepad1.right_stick_x * 0.8;
        boolean frontArmUp = gamepad2.right_bumper;
        boolean frontArmDown = gamepad2.left_bumper;
        double backArmUp = gamepad2.right_trigger;
        double backArmDown = -gamepad2.left_trigger;

        boolean elbowFOn = false;

        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.left_stick_button){
            bronto.frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}

        if (gamepad1.right_trigger > 0) { intakePow = gamepad1.right_trigger;}
        else if (gamepad1.left_trigger > 0) { intakePow = -gamepad1.left_trigger;}
        else {intakePow = 0;}


        if (frontArmUp) {
            bronto.frontArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontArmPow = 1;
        } else if (frontArmDown) {
            bronto.frontArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontArmPow = -1;}
        else{
            frontArmPow = 0;}

        if (backArmUp > 1) {
            bronto.backArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backArmPow = backArmUp;
        } else if (backArmDown != 0) {
            bronto.backArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backArmPow = backArmDown;
        }
        else{
            backArmPow = 0;}


      // if (bronto.frontElbow.getCurrentPosition() - bronto.frontElbow.getTargetPosition() > 10 || bronto.frontElbow.getCurrentPosition() - bronto.frontElbow.getTargetPosition() < 10){
         //   frontElbowPow = 0.6;}
        //else {
        // frontElbowPow = 0.45;}

        if (gamepad2.right_stick_y > 0) {
            bronto.frontElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontElbowPow = gamepad2.right_stick_y;
            bronto.frontElbow.setTargetPosition(bronto.frontElbow.getCurrentPosition());
        } else if (gamepad2.right_stick_y < 0) {
            bronto.frontElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontElbowPow = gamepad2.right_stick_y;
            bronto.frontElbow.setTargetPosition(bronto.frontElbow.getCurrentPosition());

        } else {bronto.frontElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (bronto.frontElbow.getCurrentPosition() - bronto.frontElbow.getTargetPosition() > 10 || bronto.frontElbow.getCurrentPosition() - bronto.frontElbow.getTargetPosition() < 5){
                   frontElbowPow = 0.6;}
                else {
                 frontElbowPow = 0.45;}

        }

        if (gamepad2.left_stick_y > 0) {
            bronto.backElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backElbowPow = gamepad2.left_stick_y;
            bronto.backElbow.setTargetPosition(bronto.backElbow.getCurrentPosition());

        } else if (gamepad2.left_stick_y < 0) {
            bronto.backElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backElbowPow = gamepad2.left_stick_y;
            bronto.backElbow.setTargetPosition(bronto.backElbow.getCurrentPosition());

        } else {
            bronto.backElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (bronto.backElbow.getCurrentPosition() - bronto.backElbow.getTargetPosition() > 10 || bronto.backElbow.getCurrentPosition() - bronto.backElbow.getTargetPosition() < 5){
                backElbowPow = 0.6;}
            else {backElbowPow = 0.45;}
        }

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
        if (gamepad1.y) {
            bronto.frontArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bronto.frontArm.setPower(1);
            if (bronto.frontButton.getValue() > 0.9 && bronto.frontArm.getCurrentPosition() > 500 || bronto.frontArm.getCurrentPosition() > 4000){
                bronto.frontArmHighPos = bronto.frontArm.getCurrentPosition();
                bronto.frontArmTransPos = bronto.frontArm.getCurrentPosition();
                bronto.move_to_position_and_hold(bronto.frontArm, 1 , bronto.frontArm.getCurrentPosition());

            }
            bronto.backArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bronto.backArm.setPower(1);
            if (bronto.backButton.getValue() > 0.9 && bronto.backArm.getCurrentPosition() > 500 || bronto.backArm.getCurrentPosition() > 4000){
                bronto.move_to_position_and_hold(bronto.backArm,1, bronto.backArm.getCurrentPosition());
                bronto.backArmHighPos = bronto.backArm.getCurrentPosition();
            }
            //while(bronto.frontArm.getCurrentPosition() != bronto.frontArm.getTargetPosition()){telemetry.addData("moving", "true");}
            bronto.move_to_position_and_hold(bronto.frontElbow, 1, bronto.frontElbowTransPos);
            bronto.move_to_position_and_hold(bronto.backElbow, 1, bronto.backElbowTransPos);

            state = TeleOpStates.TRANSFER;
        }

        else if (gamepad1.x && gamepad1.y) {

            bronto.move_to_position_and_hold(bronto.backArm,1, bronto.backArmHighPos);
            bronto.move_to_position_and_hold(bronto.frontArm, 1, bronto.frontArmHighPos);
            if (bronto.frontButton.getValue() > 0.9 && bronto.frontArm.getCurrentPosition() > 500){
                bronto.frontArmHighPos = bronto.frontArm.getCurrentPosition();
                bronto.frontArmTransPos = bronto.frontArm.getCurrentPosition();
                bronto.move_to_position_and_hold(bronto.frontArm, 1 , bronto.frontArm.getCurrentPosition());
            }
            bronto.move_to_position_and_hold(bronto.backArm,1, bronto.backArmHighPos);
            if (bronto.backButton.getValue() > 0.9 && bronto.backArm.getCurrentPosition() > 500){
                bronto.move_to_position_and_hold(bronto.backArm,1, bronto.backArm.getCurrentPosition());
                bronto.backArmHighPos = bronto.backArm.getCurrentPosition();
            }
                bronto.move_to_position_and_hold(bronto.frontElbow, 1, bronto.elbowDeliveryPosHigh);
                bronto.move_to_position_and_hold(bronto.backElbow, 1, bronto.backElbowHighPos);

                state = TeleOpStates.HIGH_POLE;}
          else if (gamepad1.x){
            bronto.move_to_position_and_hold(bronto.backArm,1, bronto.backArmHighPos);
            if (bronto.backButton.getValue() > 0.9 && bronto.backArm.getCurrentPosition() > 500){
                bronto.move_to_position_and_hold(bronto.backArm,1, bronto.backArm.getCurrentPosition());
                bronto.backArmHighPos = bronto.backArm.getCurrentPosition();
            }
            bronto.move_to_position_and_hold(bronto.backElbow, 1, bronto.backElbowHighPos);
        }
          else if (gamepad1.dpad_right) {
            bronto.move_to_position_and_hold(bronto.frontArm, 1, bronto.frontArmIntakePos);
            if (bronto.frontButton.getValue() > 0.9 && bronto.frontArm.getCurrentPosition() < 500){
                bronto.frontArmIntakePos = bronto.frontArm.getCurrentPosition();
                bronto.move_to_position_and_hold(bronto.frontArm, 1 , bronto.frontArm.getCurrentPosition());
            }
            bronto.move_to_position_and_hold(bronto.frontElbow, 1, bronto.frontElbowIntakePos);

            state = TeleOpStates.INTAKE;
        }
            else if (gamepad1.dpad_up) {
            brain.mainCycle(1);
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
        bronto.backIntakeL.setPower(intakePow);
        bronto.backIntakeR.setPower(intakePow);
        bronto.frontElbow.setPower(frontElbowPow);
        bronto.backElbow.setPower(backElbowPow);
        bronto.backArm.setPower(backArmPow);

        telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f), back arm (%.2f) ", leftFPower, rightFPower, leftBPower, rightBPower, bronto.frontArm.getPower(), bronto.backArm.getPower());
        telemetry.update();

    }

    /** Code to run ONCE after the driver hits STOP. */
    @Override
    public void stop () {
    }
}