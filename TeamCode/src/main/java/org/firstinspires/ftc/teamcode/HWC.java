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
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, frontArm;
    public CRServo frontIntakeL, frontIntakeR;

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
        HANDOFF,
        CYCLE,
        INIT
    }

    public HWC(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Declare all our motors
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        frontArm = hardwareMap.get(DcMotorEx.class,"frontArm");

        // Declare servos
        frontIntakeL = hardwareMap.get(CRServo.class, "intakeL");
        frontIntakeR = hardwareMap.get(CRServo.class, "intakeR");

        // Set the direction of all our motors
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        // Set CRServo Directions
        frontIntakeL.setDirection(CRServo.Direction.FORWARD);
        frontIntakeR.setDirection(CRServo.Direction.REVERSE);

        // Run motors using encoder, so that we can move accurately. If motor doesn't have, run without encoder
        leftFront.setMode(RUN_USING_ENCODER);
        rightFront.setMode(RUN_USING_ENCODER);
        leftRear.setMode(RUN_USING_ENCODER);
        rightRear.setMode(RUN_USING_ENCODER);
        frontArm.setMode(RUN_USING_ENCODER);

        // Stop and Reset encoders
        frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Functions Below Because Function Class is Hard and Annoying

    // Function to run intake set of servos to intake a cone/transfer to other arm
    public void runIntakeServo(double power) {
        frontIntakeL.setPower(power);
        frontIntakeR.setPower(power);
    }

    // Function used to move the arm to different levels will probably be deprecated for auton(eventually).
    public void move_arm(double power, int position){
        frontArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontArm.setTargetPosition(position);
        frontArm.setPower(power);
        while (frontArm.isBusy()){
            telemetry.addData("Arm Moving", "TRUE");
            telemetry.update();
        }
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
