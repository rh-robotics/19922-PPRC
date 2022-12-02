package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import com.qualcomm.robotcore.hardware.ColorSensor;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;

public class HWC {
    // Declare empty variables for robot hardware
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, frontArm, frontElbow, backArm, backElbow;
    public CRServo frontIntakeL, frontIntakeR, backIntakeR, backIntakeL;
    public ColorSensor colorSensor1;
    public int cameraMonitorViewId;

    // CV vars
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    SleeveDetection sleeveDetection = new SleeveDetection(145,168,30,50);

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
    int intakePos = -1653;
    int lowPolePos = -2560;
    int medPolePos = -4079;
    int highPolePos = -4079;
    int transferPos = -4079;
    int elbowRestingPos = 0;
    int elbowIntakePos = 423;
    int elbowTransferPos =  -150;
    int elbowDeliveryPosLow = 942;
    int elbowDeliveryPosMed = 1254;
    int elbowDeliveryPosHigh = 686;

    int backArmRestingPos = 0;
    int backIntakePos = -1325;
    int backLowPolePos = -3646;
    int backMedPolePos = -4838;
    int backHighPolePos = -4838;
    int backDeliveryPos = -4838;
    int backElbowRestingPos = 0;
    int backElbowIntakePos = 319;
    int backElbowTransferPos = 235;
    int backElbowDeliveryPosLow = 1679;
    int backElbowDeliveryPosMed = 1365;
    int backElbowDeliveryPosHigh = 980;


    public HWC(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Declare all our motors
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        frontArm = hardwareMap.get(DcMotorEx.class,"frontArm");
        frontElbow = hardwareMap.get(DcMotorEx.class, "frontElbow");
        backArm = hardwareMap.get(DcMotorEx.class, "backArm");
        backElbow = hardwareMap.get(DcMotorEx.class, "backElbow");

        // Declare servos
        frontIntakeL = hardwareMap.get(CRServo.class, "intakeL");
        frontIntakeR = hardwareMap.get(CRServo.class, "intakeR");
        backIntakeL = hardwareMap.get(CRServo.class, "BackIntakeL");
        backIntakeR = hardwareMap.get(CRServo.class, "BackIntakeR");

        //declare sensors
        colorSensor1 = hardwareMap.get(ColorSensor.class, "CS1");

        // Camera
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Set the direction of all our motors
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        frontArm.setDirection(DcMotorEx.Direction.FORWARD);
        frontElbow.setDirection(DcMotorEx.Direction.FORWARD);
        backElbow.setDirection(DcMotorEx.Direction.REVERSE);
        backArm.setDirection(DcMotorEx.Direction.REVERSE);

        //Sets the wheels to break on zero power
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        backArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Resets encoder position to zero
        frontArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontElbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backElbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    }

    // Functions Below Because Function Class is Hard and Annoying

    // Function to run intake set of servos to intake a cone/transfer to other arm
    public void runIntakeServo(char servo, double power) {
        if (servo == 'F') {
            frontIntakeL.setPower(power);
            frontIntakeR.setPower(power);
        } else if (servo == 'R'){backIntakeL.setPower(power);
            backIntakeR.setPower(power);
        } else {
            frontIntakeL.setPower(power);
            frontIntakeR.setPower(power);
            backIntakeL.setPower(-power);
            backIntakeR.setPower(-power);
        }
    }

    public String returnColor() {
        int red = colorSensor1.red();
        int green = colorSensor1.green();
        int blue = colorSensor1.blue();
        String color;

        if (red > green && red > blue && blue < 100 && green < 100) {
            color = "red";
        } else if (blue > green && red < blue && red < 100 && green < 100) {
            color = "blue";
        } else if (red < green && green > blue && blue < 100 && red < 100) {
            color = "red";
        } else {
            color = "unknown";}
        return color;
    }


    // Function used to move any motor to different positions and hold it.
    public void move_to_position_and_hold(DcMotorEx motor, double power, int position){
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
       /* while (motor.isBusy()){
            telemetry.addData(motor +" Moving", "TRUE"); */

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


    public void smartMove(armPositions pos){
        switch (pos){
            case INTAKE:
                move_to_position_and_hold(frontArm,1, intakePos);
                move_to_position_and_hold(frontElbow, 0.5 , elbowIntakePos);
                break;
            case RESTING:
                move_to_position_and_hold(frontArm,1, armRestingPos);
                move_to_position_and_hold(frontElbow,0.5, elbowRestingPos);
                break;
            case LOW_POLE:
                move_to_position_and_hold(frontArm,1, lowPolePos);
                move_to_position_and_hold(frontElbow,0.5, elbowDeliveryPosLow);
                break;
            case MED_POLE:
                move_to_position_and_hold(frontArm,1, medPolePos);
                move_to_position_and_hold(frontElbow,1, elbowTransferPos);
                break;
            case HIGH_POLE:
                move_to_position_and_hold(frontArm,1, highPolePos);
                move_to_position_and_hold(frontElbow,1, elbowTransferPos);
                if (frontArm.getCurrentPosition() == highPolePos) move_to_position_and_hold(frontElbow, 0.5, elbowDeliveryPosHigh);
                break;
            case TRANSFER:
                move_to_position_and_hold(frontElbow,1, elbowTransferPos);
                move_to_position_and_hold(frontArm,1, intakePos);
                break;
            default:
                telemetry.addData("HELP!", "This isnt possible");
                telemetry.update();
        }
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