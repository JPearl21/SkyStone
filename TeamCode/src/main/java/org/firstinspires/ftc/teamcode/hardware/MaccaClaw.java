package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class MaccaClaw {

    public static double lift_kP = 0.004;

    private OpMode parentOpMode;
    private HardwareMap hardwareMap;

    //private DcMotorEx ;
    //private CRServo ;
    private Servo chad;
    private double liftTargetPos;

    public MaccaClaw(OpMode parentOpMode) {
        this.parentOpMode = parentOpMode;
        this.hardwareMap = parentOpMode.hardwareMap;
    }

    public void initializeLift() {
        parentOpMode.telemetry.addLine("MaccaLift Initializing...");
        // map hardware

        // tell the world you've succeeded
        parentOpMode.telemetry.addLine("Lift Initialization complete.");
    }

    //public void rightpivot{

    //}
    //TODO Create method to pivot right claw down amd up

    //TODO Create method to pivot left claw down and up

    //TODO Create method to grab or release using the left claw

}
