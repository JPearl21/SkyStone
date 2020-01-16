package org.firstinspires.ftc.teamcode.auto;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.hardware.MaccaDrive;
import org.firstinspires.ftc.teamcode.hardware.MaccabotV2;

@Autonomous(name="NEEDHAM AUTO RED")
//@Disabled
public class RedCompetitionAutoBeta extends LinearOpMode {

    private MaccabotV2 robot;
    private ElapsedTime timer;

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


        // FORWARD MOVE
        robot.drive.setTargetsTicks(MaccaDrive.inchesToEncoderTicks(24), MaccaDrive.inchesToEncoderTicks(24));
        timer.reset();
        robot.drive.runToTargets(0.9, 0.9);
        while (opModeIsActive() && robot.drive.isDriveBusy() && timer.seconds() < 3) {
            telemetry.update();
        }
        robot.drive.setMotorPowers(0);


        // RIGHT TURN
        robot.drive.setTargetsTicks(MaccaDrive.inchesToEncoderTicks((13.75 * 2 * Math.PI) / 4),
                                    -MaccaDrive.inchesToEncoderTicks((13.75 * 2 * Math.PI) / 4));
        timer.reset();
        robot.drive.runToTargets(0.9, -0.9);
        while (opModeIsActive() && robot.drive.isDriveBusy() && timer.seconds() < 1.5) {
            telemetry.update();
        }
    }
}
