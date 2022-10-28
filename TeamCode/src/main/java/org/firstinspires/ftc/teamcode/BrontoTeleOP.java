package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HWC;



@TeleOp(name="Bronto's Steps", group="Iterative Opmode")

public class BrontoTeleOP extends OpMode
{
    /** Declare OpMode members. */

 static final int motorTickCount = 2786;
    public enum TeleOpStates {
        RESTING,
        INTAKE,
        LOW_POLE,
        MED_POLE,
        HIGH_POLE,
        TRANSFER,
        UNKNOWN
    }
    private DcMotor frontL = null;
    private DcMotor frontR = null;
    private DcMotor backL = null;
    private DcMotor backR = null;
    private DcMotor frontArm = null;
    private CRServo frontIntakeL = null;
    private CRServo frontIntakeR = null;

    private ElapsedTime runtime = new ElapsedTime();

    TeleOpStates state = TeleOpStates.RESTING;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        frontL  = hardwareMap.get(DcMotor.class, "leftFront");
        frontR = hardwareMap.get(DcMotor.class, "rightFront");
        backL  = hardwareMap.get(DcMotor.class, "leftRear");
        backR = hardwareMap.get(DcMotor.class, "rightRear");
        frontArm = hardwareMap.get(DcMotor.class,"frontArm");
        frontIntakeL = hardwareMap.get(CRServo.class, "intakeL");
        frontIntakeR = hardwareMap.get(CRServo.class, "intakeR");

        frontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontL.setDirection(DcMotor.Direction.FORWARD);
        backL.setDirection(DcMotor.Direction.REVERSE);
        frontR.setDirection(DcMotor.Direction.REVERSE);
        backR.setDirection(DcMotor.Direction.REVERSE);
        frontIntakeL.setDirection(DcMotor.Direction.FORWARD);
        frontIntakeR.setDirection(DcMotor.Direction.REVERSE);





        telemetry.addData("Status", "Initialized");
    }

    public void move_arm(double power, int position){
        frontArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontArm.setTargetPosition(position);
        frontArm.setPower(power);
        runtime.reset();
        busyLoop: {
        while (frontArm.isBusy()){
            while (runtime.milliseconds() < 8000){telemetry.addData("Arm Moving", "TRUE");
                telemetry.update();}
            break busyLoop;}}
        telemetry.addData("Arm Moving", "FALSE");
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



        double drive = gamepad1.left_stick_y;
        double turn  =  -gamepad1.left_stick_x ;
        double strafe = gamepad1.right_stick_x;
        double frontArmUp = gamepad2.left_trigger;
        double frontArmDown = -gamepad2.right_trigger * 5;



        if (gamepad1.left_trigger > 0) { intakePow = gamepad1.left_trigger;}
        else if (gamepad1.right_trigger > 0) {intakePow = -gamepad1.right_trigger;}
        else {intakePow = 0;}


        if (frontArmDown < 0 && frontArmUp > 0 ){ frontArmPow = 0;}
        else if (frontArmUp > 0){frontArmPow = frontArmUp * 0.5;}
        else if (frontArmDown < 0){frontArmPow = frontArmDown;}
        else{frontArmPow=0;}



        if (drive != 0 || turn != 0) {
            leftFPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFPower = Range.clip(drive - turn, -1.0, 1.0);
            leftBPower = Range.clip(drive + turn, -1.0, 1.0);
            rightBPower = Range.clip(drive - turn, -1.0, 1.0);
        }

        else if (strafe != 0 ) {
            /* Strafing */
            leftFPower = -strafe;
            rightFPower = strafe;
            leftBPower =  strafe;
            rightBPower = -strafe;
        }

        else {
            leftFPower = 0;
            rightFPower = 0;
            leftBPower = 0;
            rightBPower = 0;
        }


        if(gamepad1.y){
            move_arm(.3,motorTickCount/2);
            state = TeleOpStates.TRANSFER;
        }
        else if (gamepad1.x){
            double highPos =motorTickCount *0.75;
            move_arm(.3, (int)highPos);
            state = TeleOpStates.HIGH_POLE;
        }
        else if (gamepad1.b){
            move_arm(.3, motorTickCount/2);
        }

        switch(state){
            case RESTING:
                telemetry.addData("Arm Position", "Resting");
                telemetry.update();
                break;
            case LOW_POLE:
                telemetry.addData("Arm Position", "Low Pole");
                telemetry.update();
                break;
            case MED_POLE:
                telemetry.addData("Arm Position", "Medium Pole");
                telemetry.update();
                break;
            case HIGH_POLE:
                telemetry.addData("Arm Position", "High Pole");
                telemetry.update();
                break;
            case INTAKE:
                telemetry.addData("Arm Position", "Intake");
                telemetry.update();
                break;
            case TRANSFER:
                telemetry.addData("Arm Position", "Transfer");
                telemetry.update();
                break;
            case UNKNOWN:
                telemetry.addData("Arm Position", "Unknown");
                telemetry.update();
                break;
            default:
                state = TeleOpStates.UNKNOWN;
                telemetry.addData("Arm Position", "Unknown");
                telemetry.update();
        }


        frontL.setPower(leftFPower);
        backL.setPower(leftBPower);
        frontR.setPower(rightFPower);
        backR.setPower(rightBPower);
        frontArm.setPower(frontArmPow);
        frontIntakeL.setPower(intakePow);
        frontIntakeR.setPower(intakePow);

        telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f), front arm (%.2f)", leftFPower, rightFPower,leftBPower, rightBPower, frontArmPow);

    }

    /** Code to run ONCE after the driver hits STOP. */
    @Override
    public void stop() {

    }
}