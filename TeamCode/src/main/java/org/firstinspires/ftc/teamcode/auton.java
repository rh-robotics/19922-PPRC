package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class auton extends LinearOpMode {
  // Declare enums
  HWC.autonStates state = HWC.autonStates.SCANNING_FOR_SIGNAL;
  HWC.armPositions armPosition = HWC.armPositions.INIT;

  // Timer & Road Runner Drive
  ElapsedTime timer = new ElapsedTime();
  SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

  // Variables for CV
  // TODO: Move to bronto HWC
  SleeveDetection sleeveDetection = new SleeveDetection(145,168,30,50);
  OpenCvCamera camera;
  String webcamName = "Webcam 1";

  @Override
  public void runOpMode() throws InterruptedException {
    // Call outside classes for HWC and FC
    HWC bronto = new HWC(hardwareMap, telemetry);

    // Tell driver bronto is initializing
    telemetry.addData("Status", "Initializing");
    telemetry.update();

    // Run any initialization code necessary

    // Tell driver bronto is ready and waiting for start
    telemetry.addData("Status", "Initialized - Waiting for Start");
    telemetry.addData("State", "Scanning for Signal");
    telemetry.addData("Arm Position", "Init");
    telemetry.update();

    // Vision Code (From Jack Revoy)
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
    sleeveDetection = new SleeveDetection(145,168,30,50);
    camera.setPipeline(sleeveDetection);

    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
      @Override
      public void onOpened()
      {
        camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
      }

      @Override
      public void onError(int errorCode) {}
    });

    while (!isStarted()) {
      telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
      telemetry.update();
    }

    // Wait for Driver to press start
    waitForStart();
    while (opModeIsActive()) {
      // State Machine
      switch(state) {
        case MOVING_TO_POLE:
          // Update State Telemetry
          telemetry.addData("State", "Moving to Pole");
          telemetry.update();

          // Move arms to cycle pos & update telemetry
          double highPos = 2786 * 0.75;
          bronto.move_arm(.3, (int)highPos);
          telemetry.addData("Arm Position", "Cycle");
          telemetry.update();

          // Drive to pole, then rotate
          drive.followTrajectory(TC.startToCyclePole(this.drive));
          drive.turn(Math.toRadians(90));

          state = HWC.autonStates.DELIVERING_CONE;
        case DELIVERING_CONE:
          // Update State Telemetry
          telemetry.addData("State", "Delivering Cone");
          telemetry.update();

          // Run Gecko Wheel Servos to deposit cone
          bronto.runIntakeServo(-.3);

          state = HWC.autonStates.MOVING_TO_STACK;
        case MOVING_TO_STACK:
          // Update State Telemetry
          telemetry.addData("State", "Moving to Stack");
          telemetry.update();
        case PICKING_UP_CONE:
          // Update State Telemetry
          telemetry.addData("State", "Picking up Cone");
          telemetry.update();
        case PARKING_NO_VALUE:
          // Update State Telemetry
          telemetry.addData("State", "Parking no Value");
          telemetry.update();
        case PARKING_VALUE:
          // Update State Telemetry
          telemetry.addData("State", "Parking with Value");
          telemetry.update();
      }
    }
  }
}
