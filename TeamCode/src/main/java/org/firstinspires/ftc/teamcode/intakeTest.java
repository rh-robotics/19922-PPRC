package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HWC;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@TeleOp(name="intake", group="Iterative Opmode")

public class intakeTest extends OpMode
{
    /** Declare OpMode members. */
    HWC bronto;
    private ElapsedTime runtime = new ElapsedTime();
    public CRServo frontIntakeL, frontIntakeR;
    @Override
    public void init() {
        bronto = new HWC(hardwareMap, telemetry);

        telemetry.addData("Status", "Initializing");
        frontIntakeL = hardwareMap.get(CRServo.class, "intakeL");
        frontIntakeR = hardwareMap.get(CRServo.class, "intakeR");
        frontIntakeL.setDirection(CRServo.Direction.FORWARD);
        frontIntakeR.setDirection(CRServo.Direction.REVERSE);

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
        frontIntakeL.setPower(-gamepad1.right_trigger);
        frontIntakeR.setPower(-gamepad1.right_trigger);
        frontIntakeL.setPower(gamepad1.left_trigger);
        frontIntakeR.setPower(gamepad1.left_trigger);
    }
    @Override
    public void loop() {




    }
}