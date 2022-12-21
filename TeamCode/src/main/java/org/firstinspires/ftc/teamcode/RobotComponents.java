package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class RobotComponents {

    private final DcMotorEx motor;
    private final double ticks_in_degrees;
    private final double F;
    private final PIDController controller;

    RobotComponents (DcMotorEx motor, double ticks_in_degrees, double p, double i, double d, double f) {
        this.motor = motor;
        this.ticks_in_degrees = ticks_in_degrees;
        this.F = f;

        controller = new PIDController (p,i,d);
    }

    public void moveUsingPID (int target) {

        controller.reset();
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        int armPos = motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * F;

        double power = pid + ff;

        motor.setPower(power);

    }

    public boolean closeEnough (int target, int range) {
        if ((target - range <= motor.getCurrentPosition()) && (target + range >= motor.getCurrentPosition())) return true;
        return false;
    }

        /*
        public double[] pidf() {
            return [p, i, d, f];
        }

         */
}
