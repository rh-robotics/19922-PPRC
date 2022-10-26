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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HWC {
    // Robot variables
    public DcMotorEx frontL, frontR, backL, backR;
    Telemetry telemetry;
    ElapsedTime waitTime = new ElapsedTime();

    // Code variables
    public static final double ONE_CM_IN_PPR = 7.9;
    public static final double ONE_DEGREE_IN_PPR = 4.27;

    public HWC(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Declare all our motors and servos
        frontL = hardwareMap.get(DcMotorEx.class, "frontL");
        frontR = hardwareMap.get(DcMotorEx.class, "frontR");
        backL = hardwareMap.get(DcMotorEx.class, "backL");
        backR = hardwareMap.get(DcMotorEx.class, "backR");

        // Set the direction of all our motors and servos
        frontL.setDirection(DcMotorEx.Direction.FORWARD);
        backL.setDirection(DcMotorEx.Direction.FORWARD);
        frontR.setDirection(DcMotorEx.Direction.REVERSE);
        backR.setDirection(DcMotorEx.Direction.REVERSE);

        // Run motors using encoder, so that we can move accurately. If motor doesn't have, run without encoder
        frontL.setMode(RUN_USING_ENCODER);
        frontR.setMode(RUN_USING_ENCODER);
        backL.setMode(RUN_USING_ENCODER);
        backR.setMode(RUN_USING_ENCODER);
    }

    // Declare all methods used for moving all parts of our robot.
    public void driveAndArm(double distanceInCm, double wheelRPower, double wheelLPower) {
        int wheelCounts = 0;
        double wCounts = distanceInCm * ONE_CM_IN_PPR;

        frontL.setMode(STOP_AND_RESET_ENCODER);
        frontR.setMode(STOP_AND_RESET_ENCODER);
        backL.setMode(STOP_AND_RESET_ENCODER);
        backR.setMode(STOP_AND_RESET_ENCODER);

        frontL.setMode(RUN_WITHOUT_ENCODER);
        frontR.setMode(RUN_WITHOUT_ENCODER);
        backL.setMode(RUN_WITHOUT_ENCODER);
        backR.setMode(RUN_WITHOUT_ENCODER);

        waitTime.reset();

        while(Math.abs(wheelCounts) < wCounts) {
            wheelCounts = frontL.getCurrentPosition();

            if(Math.abs(wheelCounts) < wCounts){
                frontL.setPower(wheelLPower);
                backL.setPower(wheelLPower);
                frontR.setPower(wheelRPower);
                backR.setPower(wheelRPower);

            }
            else {
                frontL.setPower(0);
                backL.setPower(0);
                frontR.setPower(0);
                backR.setPower(0);
            }
        }

        frontL.setPower(0);
        backL.setPower(0);
        frontR.setPower(0);
        backR.setPower(0);
    }

    public void turn(double directionInDegrees, double wheelVelocity) {
//      384.5(PPR) = ~50cm = ~20in
//      7.9(PPR) = 1cm
//      4.27(PPR) = 1 Degree
        final double oneDegreeInPPR = 4.27;
        double pprTurn = directionInDegrees * oneDegreeInPPR;

        if(directionInDegrees != 0) {
            if (directionInDegrees < 0) {
                frontL.setTargetPosition(-(int) pprTurn + frontL.getCurrentPosition());
                frontR.setTargetPosition((int) pprTurn + frontR.getCurrentPosition());
                backL.setTargetPosition(-(int) pprTurn + backL.getCurrentPosition());
                backR.setTargetPosition((int) pprTurn + backR.getCurrentPosition());

            } else if (directionInDegrees > 0) {
                frontL.setTargetPosition((int) pprTurn + frontL.getCurrentPosition());
                frontR.setTargetPosition(-(int) pprTurn + frontR.getCurrentPosition());
                backL.setTargetPosition((int) pprTurn + backL.getCurrentPosition());
                backR.setTargetPosition(-(int) pprTurn + backR.getCurrentPosition());
            }

            frontL.setMode(RUN_TO_POSITION);
            frontR.setMode(RUN_TO_POSITION);
            backL.setMode(RUN_TO_POSITION);
            backR.setMode(RUN_TO_POSITION);

            frontL.setVelocity(wheelVelocity);
            frontR.setVelocity(wheelVelocity);
            backR.setVelocity(wheelVelocity);
            backL.setVelocity(wheelVelocity);
        }
    }
}
