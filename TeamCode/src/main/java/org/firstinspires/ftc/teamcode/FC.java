package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.hardware.DcMotor;

public class FC {
    HWC brontoscorus;

    public FC(HWC robot) {
        this.brontoscorus = robot;
    }

    // drive method is used to drive using encoder positions. This is currently deprecated
    // since it is last year's code and values. If RR usage goes ary I will use it however.
    public void drive(double distanceInCm, double wheelRPower, double wheelLPower) {
        int wheelCounts = 0;
        double wCounts = distanceInCm * HWC.ONE_CM_IN_PPR;

        brontoscorus.leftFront.setMode(STOP_AND_RESET_ENCODER);
        brontoscorus.rightFront.setMode(STOP_AND_RESET_ENCODER);
        brontoscorus.leftRear.setMode(STOP_AND_RESET_ENCODER);
        brontoscorus.rightRear.setMode(STOP_AND_RESET_ENCODER);

        brontoscorus.leftFront.setMode(RUN_WITHOUT_ENCODER);
        brontoscorus.rightFront.setMode(RUN_WITHOUT_ENCODER);
        brontoscorus.leftRear.setMode(RUN_WITHOUT_ENCODER);
        brontoscorus.rightRear.setMode(RUN_WITHOUT_ENCODER);

        brontoscorus.time.reset();

        while(Math.abs(wheelCounts) < wCounts) {
            wheelCounts = brontoscorus.leftFront.getCurrentPosition();

            if(Math.abs(wheelCounts) < wCounts){
                brontoscorus.leftFront.setPower(wheelLPower);
                brontoscorus.leftRear.setPower(wheelLPower);
                brontoscorus.rightFront.setPower(wheelRPower);
                brontoscorus.rightRear.setPower(wheelRPower);

            }
            else {
                brontoscorus.leftFront.setPower(0);
                brontoscorus.leftRear.setPower(0);
                brontoscorus.rightFront.setPower(0);
                brontoscorus.rightRear.setPower(0);
            }
        }
        brontoscorus.leftFront.setPower(0);
        brontoscorus.leftRear.setPower(0);
        brontoscorus.rightFront.setPower(0);
        brontoscorus.rightRear.setPower(0);
    }

    // Function used to move the arm to different levels will probably be deprecated for auton(eventually).
    public void move_arm(double power, int position){
        brontoscorus.frontArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brontoscorus.frontArm.setTargetPosition(position);
        brontoscorus.frontArm.setPower(power);
        while (brontoscorus.frontArm.isBusy()){
            brontoscorus.telemetry.addData("Arm Moving", "TRUE");
            brontoscorus.telemetry.update();
        }
    }

    // Function used to intake cones
    public static void runIntakeServo(double power) {
        brontoscorus.frontIntakeL.setPower(power);
        brontoscorus.frontIntakeR.setPower(power);
    }

    public void turn(double directionInDegrees, double wheelVelocity) {
//      384.5(PPR) = ~50cm = ~20in
//      7.9(PPR) = 1cm
//      4.27(PPR) = 1 Degree
        double pprTurn = directionInDegrees * HWC.ONE_DEGREE_IN_PPR;

        if(directionInDegrees != 0) {
            if (directionInDegrees < 0) {
                brontoscorus.leftFront.setTargetPosition(-(int) pprTurn + leftFront.getCurrentPosition());
                brontoscorus.rightFront.setTargetPosition((int) pprTurn + rightFront.getCurrentPosition());
                brontoscorus.leftRear.setTargetPosition(-(int) pprTurn + leftRear.getCurrentPosition());
                brontoscorus.rightRear.setTargetPosition((int) pprTurn + rightRear.getCurrentPosition());

            } else if (directionInDegrees > 0) {
                brontoscorus.leftFront.setTargetPosition((int) pprTurn + leftFront.getCurrentPosition());
                brontoscorus.rightFront.setTargetPosition(-(int) pprTurn + rightFront.getCurrentPosition());
                brontoscorus.leftRear.setTargetPosition((int) pprTurn + leftRear.getCurrentPosition());
                brontoscorus.rightRear.setTargetPosition(-(int) pprTurn + rightRear.getCurrentPosition());
            }

            brontoscorus.leftFront.setMode(RUN_TO_POSITION);
            brontoscorus.rightFront.setMode(RUN_TO_POSITION);
            brontoscorus.leftRear.setMode(RUN_TO_POSITION);
            brontoscorus.rightRear.setMode(RUN_TO_POSITION);

            brontoscorus.leftFront.setVelocity(wheelVelocity);
            brontoscorus.rightFront.setVelocity(wheelVelocity);
            brontoscorus.rightRear.setVelocity(wheelVelocity);
            brontoscorus.leftRear.setVelocity(wheelVelocity);
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
