package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="PID Tuning Op", group="Iterative Opmode")

public class PIDtuning extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;
    private final double TICKS_IN_DEGREES = 384.5/360; //435 rpm has 384.5 PPR, 145.1 for 60rpm

    private DcMotorEx arm_motor;
    @Override
    public void init() {
        controller = new PIDController (p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor= hardwareMap.get(DcMotorEx.class, "backElbow");
        arm_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void loop() {
        controller.setPID (p,i,d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target ", target);
        telemetry.addData("power ", power);
        telemetry.addData ("p ", p);
        telemetry.addData ("i ", i);
        telemetry.addData ("d ", d);
        telemetry.addData ("f ", f);

        telemetry.update();

    }
}