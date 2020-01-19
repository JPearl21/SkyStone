package org.firstinspires.ftc.teamcode.auto;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.hardware.MaccaClaw;
import org.firstinspires.ftc.teamcode.hardware.MaccaDrive;
import org.firstinspires.ftc.teamcode.hardware.MaccabotV2;

@Autonomous(name="Beta Auto Red")
@Disabled
public class RedCompetitionAutoBeta extends LinearOpMode {

    private MaccabotV2 robot;
    private ElapsedTime timer;

    public static double DRIVE_VELOCITY = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        timer = new ElapsedTime();

        // Micah's greatest contribution to the code
        MediaPlayer mp = MediaPlayer.create(hardwareMap.appContext, R.raw.poortown_scrub_noah_cut);

        robot = new MaccabotV2(this);
        robot.initialize(true);
        telemetry.addData("Orientation", robot.drive.getOrientation());
        telemetry.update();
        mp.start();

        waitForStart();

        mp.stop();

        telemetry.clearAll();
        telemetry.addLine("OpMode Started.");
        robot.drive.composeTelemetry(MaccaDrive.TelemetryLevel.FULL);
        telemetry.update();

        robot.autoClaw.setClawState(MaccaClaw.ClawState.DOWN_OPEN);

        // FORWARD MOVE 32"
        robot.drive.setTargetsTicks(MaccaDrive.inchesToEncoderTicks(30), MaccaDrive.inchesToEncoderTicks(30));
        timer.reset();
        robot.drive.runToTargets(DRIVE_VELOCITY, DRIVE_VELOCITY);
        while (opModeIsActive() && robot.drive.isDriveBusy() && timer.seconds() < 3) {
            telemetry.update();
        }
        robot.drive.setMotorPowers(0);

        robot.autoClaw.setClawState(MaccaClaw.ClawState.DOWN_CLOSED);
        sleep(500);
        robot.autoClaw.setClawState(MaccaClaw.ClawState.UP_CLOSED);
        //Back ~16"
        robot.drive.setTargetsTicks(-MaccaDrive.inchesToEncoderTicks(16), -MaccaDrive.inchesToEncoderTicks(16));
        timer.reset();
        robot.drive.runToTargets(-DRIVE_VELOCITY, -DRIVE_VELOCITY);
        while (opModeIsActive() && robot.drive.isDriveBusy() && timer.seconds() < 3) {
            telemetry.update();
        }
        robot.drive.setMotorPowers(0);

        // RIGHT TURN
        robot.drive.setTargetsTicks(MaccaDrive.inchesToEncoderTicks((13.75 * 2 * Math.PI) / 4),
                                    -MaccaDrive.inchesToEncoderTicks((13.75 * 2 * Math.PI) / 4));
        timer.reset();
        robot.drive.runToTargets(DRIVE_VELOCITY, -DRIVE_VELOCITY);
        while (opModeIsActive() && robot.drive.isDriveBusy() && timer.seconds() < 1.5) {
            telemetry.update();
        }
        robot.drive.setMotorPowers(0);


        // FORWARD MOVE 64"
        robot.drive.setTargetsTicks(MaccaDrive.inchesToEncoderTicks(64), MaccaDrive.inchesToEncoderTicks(64));
        timer.reset();
        robot.drive.runToTargets(DRIVE_VELOCITY, DRIVE_VELOCITY);
        while (opModeIsActive() && robot.drive.isDriveBusy() && timer.seconds() < 5) {
            telemetry.update();
        }
        robot.drive.setMotorPowers(0);

        //Left Turn 90*
        robot.drive.setTargetsTicks(-MaccaDrive.inchesToEncoderTicks((13.75 * 2 * Math.PI) / 4),
                                    MaccaDrive.inchesToEncoderTicks((13.75 * 2 * Math.PI) / 4));
        timer.reset();
        robot.drive.runToTargets(-DRIVE_VELOCITY, DRIVE_VELOCITY);
        while (opModeIsActive() && robot.drive.isDriveBusy() && timer.seconds() < 1.5) {
            telemetry.update();
        }
        robot.drive.setMotorPowers(0);

        // FORWARD MOVE 6"
        robot.drive.setTargetsTicks(MaccaDrive.inchesToEncoderTicks(6), MaccaDrive.inchesToEncoderTicks(6));
        timer.reset();
        robot.drive.runToTargets(DRIVE_VELOCITY, DRIVE_VELOCITY);
        while (opModeIsActive() && robot.drive.isDriveBusy() && timer.seconds() < 3) {
            telemetry.update();
        }
        robot.drive.setMotorPowers(0);

        //TODO A Drop
    }
}
