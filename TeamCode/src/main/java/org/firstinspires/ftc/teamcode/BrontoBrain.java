package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class BrontoBrain {
    HWC bronto;

    public BrontoBrain(HWC hwc) {
        bronto = hwc;
    }

    private ElapsedTime timer = new ElapsedTime();

    public void mainCycle(int cycles){
        for (int i = 0; i < cycles; i++) {
        bronto.move_to_position_and_hold(bronto.frontArm, 0.4, bronto.intakePos);
            bronto.move_to_position_and_hold(bronto.frontElbow, 0.4, bronto.intakePos);
        bronto.runIntakeServo('F', 1);
        timer.reset();
        while(timer.milliseconds() < 2000){
            // no sleep functions in teleOp. Probably for the best
        }
        bronto.runIntakeServo('F', 0);
        bronto.move_to_position_and_hold(bronto.frontElbow, 0.8, bronto.transferPos);
        bronto.move_to_position_and_hold(bronto.backArm, 0.8, bronto.transferPos);
        bronto.runIntakeServo('A', 1);
        timer.reset();
        while(timer.milliseconds() < 2000){
            // no sleep functions in teleOp. Probably for the best
        }
        bronto.runIntakeServo('A', 0);
    //    bronto.move_to_position_and_hold(bronto.backArm, 0.3, bronto.highPolePos);
    //    bronto.runIntakeServo('R', 1);
        timer.reset();
        while(timer.milliseconds() < 2000){
            // no sleep functions in teleOp. Probably for the best
        }
     //   bronto.runIntakeServo('R', 0);
   //     bronto.move_to_position_and_hold(bronto.backArm, 0.3, bronto.transferPos);
        }
        // Cycle over ground junction and  deliver to high pole for x number of times
        // inputs: number of cycles
        // assumption: robot must be start in team's terminal
        // move front-arm into intake position
        //      and back-arm into transfer position
        //intake
        //move front-arm into transfer position
        //transfer cone
        //move back-arm into high-pole position
        //      and move front-arm into intake
        //drop cone
        // move back-arm into transfer position
        //loop(x)

    }

    public void cv() {
        bronto.camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, bronto.webcamName), bronto.cameraMonitorViewId);
        bronto.camera.setPipeline(bronto.sleeveDetection);

        bronto.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                bronto.camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }
}
