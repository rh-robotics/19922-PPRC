package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@Disabled
@TeleOp(name="Bronto's TeleOp", group="Iterative Opmode")

public class BrontoTeleOPCoords extends OpMode
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
    double frontX = bronto.frontElbowComponent.elbowLength; //instance doesnt actually matter
    double frontY = - bronto.frontArmComponent.armLength;
    double backX = bronto.backElbowComponent.elbowLength; //instance doesnt actually matter
    double backY = - bronto.backArmComponent.armLength;

    TeleOpStates state = TeleOpStates.RESTING;
    TeleOpStates nextState = TeleOpStates.UNKNOWN; //determines when a moving state completes and will go to

    @Override
    public void init() {
        bronto = new HWC(hardwareMap, telemetry);
        brain = new BrontoBrain(bronto);
        telemetry.addData("Status", "Initializing");
        bronto.frontElbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bronto.frontElbow.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bronto.frontArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bronto.backElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bronto.backArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //backArmTarget = bronto.backArmHighPos; //set back arm to back high pole immediately for power draw issues

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
        double intakePow = 0;
        double outakePow = 0;

        double drive = -gamepad1.left_stick_y *0.8;
        double turn  =  gamepad1.left_stick_x * 0.6;
        double strafe = -gamepad1.right_stick_x * 0.8;

        //force stop and reset
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.left_stick_button){
            bronto.frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}

        //manual intake/outake control
        //changed to gamepad2 bumpers b/c unused rn
        if (gamepad2.right_trigger != 0) { outakePow = gamepad2.right_trigger;}
        else if (gamepad2.right_bumper == true) {outakePow = -1;}
        else {outakePow = 0;}

        if (gamepad2.left_trigger != 0) { intakePow = gamepad2.left_trigger;}
        else if (gamepad2.left_bumper == true) {intakePow = -1;}
        else {intakePow = 0;}

        /*
        if (gamepad2.right_stick_button){
            manualMode = !manualMode;
        }
         */

        //manual arm control
        if (gamepad2.left_stick_x != 0) { //manual control just changes target, large numbers b/c large ticks needed
            backX += (gamepad2.left_stick_x);
        } else if (gamepad2.left_stick_y != 0) {
            backY += (gamepad2.left_stick_y);
        }

        if (gamepad2.right_stick_x != 0) {
            frontX += (gamepad2.right_stick_x);
        } else if (gamepad2.right_stick_y != 0) {
            frontY += (gamepad2.right_stick_y);
        }

        //calculate drive pwr
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


        /*if (gamepad2.y) {
            state = TeleOpStates.MOVING;
            nextState = TeleOpStates.TRANSFER;
            frontElbowTarget = bronto.frontElbowTransPos;
            backElbowTarget = bronto.backElbowTransPos;
            frontArmTarget = bronto.frontArmTransPos;
            backArmTarget = bronto.backArmHighPos;
        } else if (gamepad1.x) {
            state = TeleOpStates.MOVING;
            nextState = TeleOpStates.DELIVERING;
            frontElbowTarget = bronto.frontElbowIntakePos;
            backElbowTarget = bronto.backElbowHighPos;
            frontArmTarget = bronto.frontArmIntakePos;
            backArmTarget = bronto.backArmHighPos;
        } else if (gamepad1.b) {
            state = TeleOpStates.MOVING;
            nextState = TeleOpStates.DELIVERING;
            frontElbowTarget = bronto.frontElbowIntakePos;
            backElbowTarget = bronto.backElbowMedPos;
            frontArmTarget = bronto.frontArmIntakePos;
            backArmTarget = bronto.backArmMedPos;
        } else if (gamepad1.a) {
            state = TeleOpStates.MOVING;
            nextState = TeleOpStates.DELIVERING;
            frontElbowTarget = bronto.frontElbowIntakePos;
            backElbowTarget = bronto.backElbowLowPos;
            frontArmTarget = bronto.frontArmIntakePos;
            backArmTarget = bronto.backArmLowPos;
        }

        //setting next state
        switch (state) {
            case RESTING:
                telemetry.addData("Arm State", "Resting");

                break;
            case DELIVERING:
                telemetry.addData("Arm State", "High Pole");
                //TODO: check if should be -= and adjust target
                backElbowTarget += bronto.moveByDistance(bronto.backDistanceSensor, 150);
                outakePow = -1;
                if  (bronto.returnColor(bronto.backIntakeSensor) == "unknown"){
                    outakePow = 0;}
                state = TeleOpStates.UNKNOWN;

                break;
            case INTAKE:
                telemetry.addData("Arm State", "Intake");
                        *//*
                        basically this should add to the target some small value once it is intaking to fine tune its
                        distance from the ground
                        TODO: check if should be -= and adjust target
                         *//*
                frontElbowTarget += bronto.moveByDistance(bronto.frontDistanceSensor, 7);
                intakePow = 1;
                if (bronto.returnColor(bronto.frontIntakeSensor) != "unknown"){
                    intakePow = 0;}
                state = TeleOpStates.UNKNOWN;
                break;
            case MOVING:
                telemetry.addData("Arm State", "Moving");
                if (bronto.frontElbowComponent.closeEnough (frontElbowTarget, 2) &&
                        bronto.backElbowComponent.closeEnough (backElbowTarget, 2) &&
                        bronto.frontArmComponent.closeEnough(frontArmTarget, 5) &&
                        bronto.backArmComponent.closeEnough (backArmTarget, 5)) {
                    state = nextState;
                    nextState = TeleOpStates.UNKNOWN;
                }

                break;
            case TRANSFER:
                telemetry.addData("Arm State", "Transfer");
                //TODO: check if should be -= and adjust target
                //thus both should be at the same height and aligned
                frontElbowTarget += bronto.moveByDistance(bronto.frontDistanceSensor, 50);
                backElbowTarget += bronto.moveByDistance(bronto.backDistanceSensor, 50);
                //TODO: if the above works well, we should porbably incorporate as an && for the transfer position detection rather than a constant loop
                if (bronto.returnColor(bronto.transferSensor) != "unknown"){
                    intakePow = 1;}
                if (bronto.returnColor(bronto.backIntakeSensor) != "unknown"){
                    intakePow = 0;}
                state = TeleOpStates.UNKNOWN;
                break;
            case UNKNOWN:
                telemetry.addData("Arm State", "Unknown");
                break;
            default:
                state = TeleOpStates.UNKNOWN;
                telemetry.addData("Arm Position", "Unknown");

        }*/


        bronto.leftFront.setPower(leftFPower);
        bronto.leftRear.setPower(leftBPower);
        bronto.rightFront.setPower(rightFPower);
        bronto.rightRear.setPower(rightBPower);
        bronto.frontIntakeL.setPower(intakePow);
        bronto.frontIntakeR.setPower(intakePow);
        bronto.backIntakeL.setPower(outakePow);
        bronto.backIntakeR.setPower(outakePow);

        frontArmTarget = bronto.frontArmComponent.armTicksUsingCoords(frontX, frontY);
        frontElbowTarget = bronto.frontElbowComponent.elbowTicksUsingCoords(frontX, frontY);
        backArmTarget = bronto.backArmComponent.armTicksUsingCoords(backX, backY);
        backElbowTarget = bronto.backElbowComponent.elbowTicksUsingCoords(backX, backY);

        /*if arm motors are close enough, set to 0 b/c power draw and worm gear already holds it
        now also checking if buttons are pressed,
        ok this is an OR that will set power to 0 if position is close enough OR
        button is pressed when state is not resting, this might be really bad but idk
        TODO: TEST THIS CRAP
         */
        if (bronto.frontArmComponent.closeEnough(frontArmTarget, 5) ||
                (bronto.frontButton.isPressed() && state != TeleOpStates.RESTING)) {
            bronto.frontArm.setPower(0);
        } else {bronto.frontArmComponent.moveUsingPID(frontArmTarget);}
        if (bronto.backArmComponent.closeEnough(backArmTarget, 5) ||
                bronto.backButton.isPressed() && state != TeleOpStates.RESTING) {
            bronto.backArm.setPower(0);
        } else {bronto.backArmComponent.moveUsingPID(backArmTarget);}
        bronto.frontElbowComponent.moveUsingPID(frontElbowTarget);
        bronto.backElbowComponent.moveUsingPID(backElbowTarget);

        telemetry.addData("frontArm Pos", bronto.frontArm.getCurrentPosition());
        telemetry.addData("frontArm Pwr", bronto.frontArm.getPower());
        telemetry.addData("frontElbow Pos", bronto.frontElbow.getCurrentPosition());
        telemetry.addData("frontElbow Pwr", bronto.frontElbow.getPower());
        telemetry.addData("backArm Pos", bronto.backArm.getCurrentPosition());
        telemetry.addData("backArm Pwr", bronto.backArm.getPower());
        telemetry.addData("backElbow Pos", bronto.backElbow.getCurrentPosition());
        telemetry.addData("backElbow Pwr", bronto.backElbow.getPower());
        //telemetry.addData("Positions", "front Arm %d, Back Arm %d, Front Elbow %d, Back Elbow %d", bronto.frontArm.getCurrentPosition(), bronto.backArm.getCurrentPosition(), bronto.frontElbow.getCurrentPosition(), bronto.backElbow.getCurrentPosition());
        // telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f), front elbow (%.2f),  ", bronto.leftFront, bronto.rightFront, bronto.leftRear, bronto.rightRear, bronto.frontArm.getPower(), bronto.frontElbow.getPower());
        telemetry.update();
    }

    /** Code to run ONCE after the driver hits STOP. */
    @Override
    public void stop () {
    }
}