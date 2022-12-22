package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
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
            bronto.frontArmComponent.moveUsingPID(bronto.frontArmIntakePos);
            bronto.frontElbowComponent.moveUsingPID(bronto.frontElbowIntakePos);
            bronto.runIntakeServo('F', 1);
            timer.reset();
            while(timer.milliseconds() < 2000){
                // no sleep functions in teleOp. Probably for the best
                }
            bronto.runIntakeServo('F', 0);
            bronto.frontArmComponent.moveUsingPID(bronto.frontArmTransPos);
            bronto.backArmComponent.moveUsingPID(bronto.backArmHighPos);
            bronto.backElbowComponent.moveUsingPID(bronto.backElbowTransPos);
            bronto.frontElbowComponent.moveUsingPID(bronto.frontElbowTransPos);
            bronto.runIntakeServo('A', 1);
            timer.reset();
            while(timer.milliseconds() < 2000){
                // no sleep functions in teleOp. Probably for the best
            }
            bronto.runIntakeServo('A', 0);
            bronto.backElbowComponent.moveUsingPID(bronto.backElbowHighPos);
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