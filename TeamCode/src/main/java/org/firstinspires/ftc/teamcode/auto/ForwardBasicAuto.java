package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.MaccaDrive;
import org.firstinspires.ftc.teamcode.hardware.MaccabotV2;


@Autonomous(name="Forward Basic Auto")
public class ForwardBasicAuto extends LinearOpMode {

    private MaccabotV2 robot;
    private ElapsedTime timer;

    public static double DRIVE_VELOCITY = 0.9;

    public void runOpMode() throws InterruptedException {

        robot = new MaccabotV2(this);
        robot.initialize(true);

        waitForStart();

        while(opModeIsActive()) {
            robot.drive.setTargetsTicks(MaccaDrive.inchesToEncoderTicks(30), MaccaDrive.inchesToEncoderTicks(30));
            timer.reset();
            robot.drive.runToTargets(DRIVE_VELOCITY, DRIVE_VELOCITY);
            while (opModeIsActive() && robot.drive.isDriveBusy() && timer.seconds() < 3) {
                telemetry.update();
            }
            robot.drive.setMotorPowers(0);

            stop();
        }
    }
}
