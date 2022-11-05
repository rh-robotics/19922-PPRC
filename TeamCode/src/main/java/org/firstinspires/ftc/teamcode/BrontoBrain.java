package org.firstinspires.ftc.teamcode;

public class BrontoBrain {

    public void mainCycle(int cycles){
        HWC.move_to_position_and_hold(HWC.frontArm, 0.3, HWC.intakePos);
        HWC.move_to_position_and_hold(HWC.frontElbow, 0.3, HWC.transferPos);
        HWC.runIntakeServo("F", 1);
        HWC.move_to_position_and_hold(HWC.backArm, 0.3, HWC.transferPos);
        HWC.runIntakeServo("L", 1);
        HWC.move_to_position_and_hold(HWC.backArm, 0.3 HWC.highPolePos);
        HWC.runIntakeServo("R", 1);
        HWC.move_to_position_and_hold(HWC.backArm, 0.3, HWC.transferPos);
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







}
