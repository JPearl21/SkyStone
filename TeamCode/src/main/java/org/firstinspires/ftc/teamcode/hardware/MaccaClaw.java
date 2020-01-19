package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class MaccaClaw {

    public static double lift_kP = 0.004;

    private OpMode parentOpMode;
    private HardwareMap hardwareMap;

    private Servo wrist, grabber;

    public enum ClawState { DOWN_OPEN, DOWN_CLOSED, UP_OPEN, UP_CLOSED, DEPOSIT }

    public static double WRIST_DOWN_POS = 0.45;
    public static double WRIST_UP_POS = 0.55;
    public static double GRABBER_OPEN_POS = 0.25;
    public static double GRABBER_CLOSED_POS = 0.75;

    public MaccaClaw(OpMode parentOpMode) {
        this.parentOpMode = parentOpMode;
        this.hardwareMap = parentOpMode.hardwareMap;
    }

    public void initializeClaw() {
        parentOpMode.telemetry.addLine("MaccaClaw Initializing...");
        // map hardware
        wrist = hardwareMap.servo.get("wrist");
        grabber = hardwareMap.servo.get("grabber");
        // tell the world you've succeeded
        parentOpMode.telemetry.addLine("AutoClaw Initialization complete.");
    }

    public void setClawState(ClawState clawState) {
        switch (clawState) {
            case DOWN_OPEN:
                wrist.setPosition(WRIST_DOWN_POS);
                grabber.setPosition(GRABBER_OPEN_POS);
                break;
            case DOWN_CLOSED:
                wrist.setPosition(WRIST_DOWN_POS);
                grabber.setPosition(GRABBER_CLOSED_POS);
            case UP_OPEN:
                wrist.setPosition(0.55);
                grabber.setPosition(0.75);
                break;
            case UP_CLOSED:
                wrist.setPosition(WRIST_UP_POS);
                grabber.setPosition(GRABBER_CLOSED_POS);
                break;
            case DEPOSIT:
                wrist.setPosition((WRIST_UP_POS + WRIST_DOWN_POS) / 2);
                grabber.setPosition(GRABBER_OPEN_POS);
        }
    }

}
