package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

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

    // TODO: Finish Function and replace runToPosition in auton file
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
