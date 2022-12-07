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
    RobotComponents PID;

    public BrontoBrain(HWC hwc) {
        bronto = hwc;
    }

    private ElapsedTime timer = new ElapsedTime();

    public void mainCycle(int cycles){
        for (int i = 0; i < cycles; i++) {
            bronto.frontArmComponent.moveUsingPID(bronto.intakePos);
            bronto.frontElbowComponent.moveUsingPID(bronto.elbowIntakePos);
            bronto.runIntakeServo('F', 1);
            timer.reset();
            while(timer.milliseconds() < 2000){
                // no sleep functions in teleOp. Probably for the best
                }
            bronto.runIntakeServo('F', 0);
            bronto.frontArmComponent.moveUsingPID(bronto.transferPos);
            bronto.backArmComponent.moveUsingPID(bronto.backHighPolePos);
            bronto.backElbowComponent.moveUsingPID(bronto.backElbowTransferPos);
            bronto.frontElbowComponent.moveUsingPID(bronto.elbowTransferPos);
            bronto.runIntakeServo('A', 1);
            timer.reset();
            while(timer.milliseconds() < 2000){
                // no sleep functions in teleOp. Probably for the best
            }
            bronto.runIntakeServo('A', 0);
            bronto.backElbowComponent.moveUsingPID(bronto.backElbowDeliveryPosHigh);
            bronto.runIntakeServo('R', 1);
            timer.reset();
            while(timer.milliseconds() < 2000){
                // no sleep functions in teleOp. Probably for the best
            }
            bronto.runIntakeServo('A', 0);
            //     bronto.move_to_position_and_hold(bronto.backArm, 0.3, bronto.transferPos);
        }
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