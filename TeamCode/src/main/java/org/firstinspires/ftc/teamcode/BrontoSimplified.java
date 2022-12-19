package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name="Bronto Simplified", group="Iterative Opmode")

public class BrontoSimplified extends OpMode
{
    /** Declare OpMode members. */
   HWC bronto;
   BrontoBrain brain;
   PIDController controller;

    private ElapsedTime runtime = new ElapsedTime();

    int backElbowTarget = 0;

    double be_P = .018;
    double be_I = .2;
    double be_D = .001;
    double be_F = .05;

    double pid;
    double ff;
    double power;

    public PIDController backElbowPID;

    @Override
    public void init() {
        bronto = new HWC(hardwareMap, telemetry);
        brain = new BrontoBrain(bronto);
        telemetry.addData("Status", "Initializing");
        bronto.backElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backElbowPID = new PIDController(.018, .2, .001);
        //backArmTarget = bronto.backHighPolePos; //set back arm to back high pole immediately for power draw issues

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

        //backElbowPID.reset();//testing to see if the integral accumulation of error is the problem

        /* Setup a variable for each drive wheel to save power level for telemetry. */

          if (gamepad2.left_stick_y != 0) { //manual control just changes target, large numbers b/c large ticks needed
              backElbowTarget += (gamepad2.left_stick_y * 10);
          }

        controller = new PIDController(be_P,be_I,be_D);
          //controller.setPID(be_P, be_I, be_D);

        bronto.backElbow.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        pid = backElbowPID.calculate(bronto.backElbow.getCurrentPosition(), backElbowTarget);
        ff = Math.cos(Math.toRadians(backElbowTarget / (145.1/360))) * .05;
        power = pid + ff;
        bronto.backElbow.setPower(power);


        telemetry.addData("backElbow Target", backElbowTarget);
        telemetry.addData("backElbow Pos", bronto.backElbow.getCurrentPosition());
        telemetry.addData("backElbow Pwr", bronto.backElbow.getPower());
        telemetry.addData("P: ", controller.getP());
        telemetry.addData("I: ", controller.getI());
        telemetry.addData("D: ", controller.getD());
        telemetry.update();
    }

            /** Code to run ONCE after the driver hits STOP. */
            @Override
            public void stop () {
            }
        }