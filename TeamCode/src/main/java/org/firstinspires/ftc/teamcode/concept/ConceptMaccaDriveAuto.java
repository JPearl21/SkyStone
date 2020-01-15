package org.firstinspires.ftc.teamcode.concept;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.hardware.MaccaDrive;
import org.firstinspires.ftc.teamcode.hardware.MaccabotV2;

@Autonomous(name="ConceptMaccaDriveAuto")
//@Disabled
public class ConceptMaccaDriveAuto extends LinearOpMode {

    private MaccabotV2 robot;

    @Override
    public void runOpMode() throws InterruptedException {

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

        robot.drive.setTargetsTicks(MaccaDrive.inchesToEncoderTicks(24), MaccaDrive.inchesToEncoderTicks(24));
        robot.drive.runToTargets(0.9, 0.9);
        while (opModeIsActive() && robot.drive.isDriveBusy()) {
            telemetry.update();
        }

        telemetry.addLine("Target achieved. All clear!");
        telemetry.update();
        sleep(5000);
    }
}
