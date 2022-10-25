package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HWC;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class auton extends LinearOpMode {
  // TODO: Move state to bronto HWC
  enumStates state = enumStates.SCANNING_FOR_SIGNAL;
  ElapsedTime timer = new ElapsedTime();

  // Variables for CV
  // TODO: Move to bronto HWC
  SleeveDetection sleeveDetection = new SleeveDetection();
  OpenCvCamera camera;
  String webcamName = "Webcam 1";

  @Override
  public void runOpMode() throws InterruptedException {
    HWC bronto = new HWC(hardwareMap, telemetry);

    // Tell driver it is initializing
    telemetry.addData("Status", "Initializing");
    telemetry.update();

    // Run any initialization code necessary

    // Tell driver ready and waiting for start
    telemetry.addData("Status", "Initialized - Waiting for Start");
    telemetry.addData("State", "Scanning for Signal");
    telemetry.update();

    // Vision Code Implement - Jack Might need to modify
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
    sleeveDetection = new SleeveDetection();
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
      if (state == enumStates.MOVING_TO_POLE) {
        telemetry.addData("State", "Moving to Pole");
        telemetry.update();
      } else if (state == enumStates.DELIVERING_CONE) {
        telemetry.addData("State", "Delivering Cone");
        telemetry.update();
      } else if (state == enumStates.MOVING_TO_STACK) {
        telemetry.addData("State", "Moving to Stack");
        telemetry.update();
      } else if (state == enumStates.PICKING_UP_CONE) {
        telemetry.addData("State", "Picking up Cone");
        telemetry.update();
      } else if (state == enumStates.PARKING_NO_VALUE) {
        telemetry.addData("State", "Parking no Value");
        telemetry.update();
      } else if (state == enumStates.PARKING_VALUE) {
        telemetry.addData("State", "Parking with Value");
        telemetry.update();
      }
    }
  }
}
