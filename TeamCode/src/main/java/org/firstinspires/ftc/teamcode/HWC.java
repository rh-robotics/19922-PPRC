package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HWC {
    // Robot variables
    public DcMotorEx leftFront, rightFront, leftRear, rightRear;
    Telemetry telemetry;
    ElapsedTime time = new ElapsedTime();

    // Code variables
    public static final double ONE_CM_IN_PPR = 7.9;
    public static final double ONE_DEGREE_IN_PPR = 4.27;

    public HWC(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Declare all our motors and servos
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");


        // Set the direction of all our motors
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        // Run motors using encoder, so that we can move accurately. If motor doesn't have, run without encoder
        leftFront.setMode(RUN_USING_ENCODER);
        rightFront.setMode(RUN_USING_ENCODER);
        leftRear.setMode(RUN_USING_ENCODER);
        rightRear.setMode(RUN_USING_ENCODER);
    }

    // Declare all methods used for moving all parts of our robot.
    public void driveAndArm(double distanceInCm, double wheelRPower, double wheelLPower) {
        int wheelCounts = 0;
        double wCounts = distanceInCm * ONE_CM_IN_PPR;

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
        double pprTurn = directionInDegrees * ONE_DEGREE_IN_PPR;

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
}
