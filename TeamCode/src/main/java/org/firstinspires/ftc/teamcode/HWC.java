package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HWC {
    // Declare empty variables for robot hardware
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, frontArm, frontElbow, backArm, backElbow;
    public CRServo frontIntakeL, frontIntakeR, backIntakeR, backIntakeL;

    // Declare other variables to be used here
    Telemetry telemetry;
    ElapsedTime time = new ElapsedTime();

    // Code variables
    public static final double ONE_CM_IN_PPR = 7.9;
    public static final double ONE_DEGREE_IN_PPR = 4.27;

    // autonStates Enum
    public enum autonStates {
        SCANNING_FOR_SIGNAL,
        MOVING_TO_POLE,
        DELIVERING_CONE,
        MOVING_TO_STACK,
        PICKING_UP_CONE,
        PARKING_NO_VALUE,
        PARKING_VALUE
    }

    // armPositions Enum
    public enum armPositions {
        RESTING,
        INTAKE,
        LOW_POLE,
        MED_POLE,
        HIGH_POLE,
        TRANSFER,
        UNKNOWN
    }
    //We should both be using these in all our code. Makes it much easier to tune as only one person has to
    //BS numbers but I needed something
    int armRestingPos = 0;
    int intakePos = 200;
    int lowPolePos = 400;
    int medPolePos = 600;
    int highPolePos = 900;
    int transferPos = 1100;
    int elbowRestingPos = 0;
    int elbowIntakePos = 300;
    int elbowTransferPos = elbowRestingPos;
    int elbowDeliveryPos = 250;







    public HWC(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Declare all our motors
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        frontArm = hardwareMap.get(DcMotorEx.class,"frontArm");
        frontElbow = hardwareMap.get(DcMotorEx.class, "frontElbow");

        // Declare servos
        frontIntakeL = hardwareMap.get(CRServo.class, "intakeL");
        frontIntakeR = hardwareMap.get(CRServo.class, "intakeR");
        backIntakeL = hardwareMap.get(CRServo.class, "BackIntakeL");
        backIntakeR = hardwareMap.get(CRServo.class, "BackIntakeR");

        // Set the direction of all our motors
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        // Set CRServo Directions
        frontIntakeL.setDirection(CRServo.Direction.FORWARD);
        frontIntakeR.setDirection(CRServo.Direction.REVERSE);
        backIntakeL.setDirection(CRServo.Direction.FORWARD);
        backIntakeR.setDirection(CRServo.Direction.REVERSE);

        // Run motors using encoder, so that we can move accurately. If motor doesn't have, run without encoder
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontElbow.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Resets encoder position to zero
        frontArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontElbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Functions Below Because Function Class is Hard and Annoying

    // Function to run intake set of servos to intake a cone/transfer to other arm
    public void runIntakeServo(char servo, double power) {
        if (servo == 'F') {frontIntakeL.setPower(power);
        frontIntakeR.setPower(power);}
        else if (servo == 'R'){backIntakeL.setPower(power);
            backIntakeR.setPower(power);}
        else {
            frontIntakeL.setPower(power);
            frontIntakeR.setPower(power);
            backIntakeL.setPower(-power);
            backIntakeR.setPower(-power);
        }
    }

    // Function used to move any motor to different positions and hold it.
    public void move_to_position_and_hold(DcMotorEx motor, double power, int position){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(position);
        motor.setPower(power);
       /* while (motor.isBusy()){
            telemetry.addData(motor +" Moving", "TRUE");
            telemetry.update();
        }*/
    }

    // drive method is used to drive using encoder positions. This is currently deprecated
    // since it is last year's code and values. If RR usage goes ary I will use it however.
    public void drive(double distanceInCm, double wheelRPower, double wheelLPower) {
        int wheelCounts = 0;
        double wCounts = distanceInCm * HWC.ONE_CM_IN_PPR;

        leftFront.setMode(STOP_AND_RESET_ENCODER);
        rightFront.setMode(STOP_AND_RESET_ENCODER);
        leftRear.setMode(STOP_AND_RESET_ENCODER);
        rightRear.setMode(STOP_AND_RESET_ENCODER);

        leftFront.setMode(RUN_WITHOUT_ENCODER);
        rightFront.setMode(RUN_WITHOUT_ENCODER);
        leftRear.setMode(RUN_WITHOUT_ENCODER);
        rightRear.setMode(RUN_WITHOUT_ENCODER);

        time.reset();

        while(Math.abs(wheelCounts) < wCounts) {
            wheelCounts = leftFront.getCurrentPosition();

            if(Math.abs(wheelCounts) < wCounts){
                leftFront.setPower(wheelLPower);
                leftRear.setPower(wheelLPower);
                rightFront.setPower(wheelRPower);
                rightRear.setPower(wheelRPower);

            }
            else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
            }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void turn(double directionInDegrees, double wheelVelocity) {
//      384.5(PPR) = ~50cm = ~20in
//      7.9(PPR) = 1cm
//      4.27(PPR) = 1 Degree
        double pprTurn = directionInDegrees * HWC.ONE_DEGREE_IN_PPR;

        if(directionInDegrees != 0) {
            if (directionInDegrees < 0) {
                leftFront.setTargetPosition(-(int) pprTurn + leftFront.getCurrentPosition());
                rightFront.setTargetPosition((int) pprTurn + rightFront.getCurrentPosition());
                leftRear.setTargetPosition(-(int) pprTurn + leftRear.getCurrentPosition());
                rightRear.setTargetPosition((int) pprTurn + rightRear.getCurrentPosition());

            } else if (directionInDegrees > 0) {
                leftFront.setTargetPosition((int) pprTurn + leftFront.getCurrentPosition());
                rightFront.setTargetPosition(-(int) pprTurn + rightFront.getCurrentPosition());
                leftRear.setTargetPosition((int) pprTurn + leftRear.getCurrentPosition());
                rightRear.setTargetPosition(-(int) pprTurn + rightRear.getCurrentPosition());
            }

            leftFront.setMode(RUN_TO_POSITION);
            rightFront.setMode(RUN_TO_POSITION);
            leftRear.setMode(RUN_TO_POSITION);
            rightRear.setMode(RUN_TO_POSITION);

            leftFront.setVelocity(wheelVelocity);
            rightFront.setVelocity(wheelVelocity);
            rightRear.setVelocity(wheelVelocity);
            leftRear.setVelocity(wheelVelocity);
        }
    }

    // TODO: Finish Function and replace runToPosition (used above)
//    public void moveArm(HWC.armPositions state) {
//        int wheelCounts = 0;
//        int targetPos = 0;
//
//        brontoscorus.leftFront.setMode(STOP_AND_RESET_ENCODER);
//        brontoscorus.rightFront.setMode(STOP_AND_RESET_ENCODER);
//        brontoscorus.leftRear.setMode(STOP_AND_RESET_ENCODER);
//        brontoscorus.rightRear.setMode(STOP_AND_RESET_ENCODER);
//
//        brontoscorus.leftFront.setMode(RUN_WITHOUT_ENCODER);
//        brontoscorus.rightFront.setMode(RUN_WITHOUT_ENCODER);
//        brontoscorus.leftRear.setMode(RUN_WITHOUT_ENCODER);
//        brontoscorus.rightRear.setMode(RUN_WITHOUT_ENCODER);
//
//        brontoscorus.time.reset();
//
//        switch(state) {
//            case HANDOFF:
//                brontoscorus.telemetry.addData("armPosition", "HANDOFF");
//                brontoscorus.telemetry.update();
//                targetPos = 2786/2;
//            case CYCLE:
//                brontoscorus.telemetry.addData("armPosition", "CYCLE");
//                brontoscorus.telemetry.update();
//        }
//
//        while(Math.abs(wheelCounts) < targetPos) {
//            wheelCounts = brontoscorus.leftFront.getCurrentPosition();
//
//            if(Math.abs(wheelCounts) < targetPos){
//                brontoscorus.leftFront.setPower(wheelLPower);
//                brontoscorus.leftRear.setPower(wheelLPower);
//                brontoscorus.rightFront.setPower(wheelRPower);
//                brontoscorus.rightRear.setPower(wheelRPower);
//
//            }
//            else {
//                brontoscorus.leftFront.setPower(0);
//                brontoscorus.leftRear.setPower(0);
//                brontoscorus.rightFront.setPower(0);
//                brontoscorus.rightRear.setPower(0);
//            }
//        }
//
//        brontoscorus.leftFront.setPower(0);
//        brontoscorus.leftRear.setPower(0);
//        brontoscorus.rightFront.setPower(0);
//        brontoscorus.rightRear.setPower(0);
//    }
}
