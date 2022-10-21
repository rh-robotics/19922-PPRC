package org.firstinspires.ftc.teamcode;

@Autonomous
public class auton extends LinearOpMode {
  states

  @Override
  public void runOpMode() throws InterruptedException {
    HWC robot = new HWC(hardwareMap, telemetry);

    telemetry.addData("Status", "Initializing");
    telemetry.update();

    waitForStart();
    while(opModeIsActive()) {
    }
  }
}
