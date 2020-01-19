package org.firstinspires.ftc.teamcode.auto;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.hardware.MaccaClaw;
import org.firstinspires.ftc.teamcode.hardware.MaccaDrive;
import org.firstinspires.ftc.teamcode.hardware.MaccabotV2;

@Autonomous(name="NEEDHAM AUTO RED")
//@Disabled
public class RedCompetitionAutoMethodized extends LinearOpMode {

    private MaccabotV2 robot;
    private ElapsedTime overallTimer;

    public static double DRIVE_VELOCITY = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        overallTimer = new ElapsedTime();

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

        // Here's where the fun starts.

        doFirstStone(); // total time: 7.25s

        doSecondStone(); // total time: 9.75s. Accumulated time at 17s

        doThirdStone(); // total time: 11.25s. Accumulated time at 28.25s

        // BACKWARDS MOVE 48" w/ SERVO EXTEND
        robot.lift.runRack(0.5);
        moveLine(-40, 0.9, 1.25);
        robot.lift.runRack(0);
    }

    void doFirstStone() {
        robot.autoClaw.setClawState(MaccaClaw.ClawState.DOWN_OPEN);

        // FORWARD MOVE 32"
        moveLine(32, 0.9, 1);

        robot.autoClaw.setClawState(MaccaClaw.ClawState.DOWN_CLOSED);
        sleep(500);
        robot.autoClaw.setClawState(MaccaClaw.ClawState.UP_CLOSED);

        //Back ~8"
        moveLine(-8, 0.9, 0.75);

        // LEFT TURN
        turnDegrees(-90, 0.9, 0.5);


        // BACKWARD MOVE 72"
        moveLine(-72, 0.9, 2.25);

        depositFromAlignment(); // total time: 2.25s
    }

    void doSecondStone() {
        // FORWARD MOVE 80"
        moveLine(80, 0.9, 2.25);

        // TURN RIGHT
        turnDegrees(90, 0.9, 0.5);

        // FORWARD MOVE 8"
        moveLine(8, 0.9, 0.5);

        robot.autoClaw.setClawState(MaccaClaw.ClawState.DOWN_CLOSED);
        sleep(500);
        robot.autoClaw.setClawState(MaccaClaw.ClawState.UP_CLOSED);

        //Back 8"
        moveLine(-8, 0.9, 0.75);

        // LEFT TURN
        turnDegrees(-90, 0.9, 0.5);


        // BACKWARD MOVE 88"
        moveLine(-80, 0.9, 2.5);

        depositFromAlignment(); // total time: 2.25s
    }

    void doThirdStone() {
        // FORWARD MOVE 96"
        moveLine(96, 0.9, 3);

        // RIGHT TURN
        turnDegrees(90, 0.9, 0.5);

        // FORWARD MOVE 8"
        moveLine(8, 0.9, 0.5);

        robot.autoClaw.setClawState(MaccaClaw.ClawState.DOWN_CLOSED);
        sleep(500);
        robot.autoClaw.setClawState(MaccaClaw.ClawState.UP_CLOSED);

        //Back 8"
        moveLine(-8, 0.9, 0.75);

        // LEFT TURN
        turnDegrees(-90, 0.9, 0.5);

        // BACKWARD MOVE 104"
        moveLine(-104, 0.9, 3.25);

        // RIGHT TURN
        turnDegrees(90, 0.9, 0.5);

        // FORWARD MOVE 8"
        moveLine(8, 0.9, 0.5);

        robot.autoClaw.setClawState(MaccaClaw.ClawState.DEPOSIT);
        sleep(250);
        robot.autoClaw.setClawState(MaccaClaw.ClawState.DOWN_OPEN);

        moveLine(-6, 0.9, 0.5);

        // RIGHT TURN
        turnDegrees(90, 0.9, 0.5);
    }

    void depositFromAlignment() {
        // RIGHT TURN
        turnDegrees(90, 0.9, 0.5);

        // FORWARD MOVE 8"
        moveLine(8, 0.9, 0.5);

        robot.autoClaw.setClawState(MaccaClaw.ClawState.DEPOSIT);
        sleep(250);
        robot.autoClaw.setClawState(MaccaClaw.ClawState.DOWN_OPEN);

        moveLine(-6, 0.9, 0.5);

        // LEFT TURN
        turnDegrees(-90, 0.9, 0.5);
    }

    void moveLine(double distanceInches, double maxPower, double maxTime) {
        ElapsedTime localTimer = new ElapsedTime();

        robot.drive.setTargetsTicks(MaccaDrive.inchesToEncoderTicks(distanceInches),
                                    MaccaDrive.inchesToEncoderTicks(distanceInches));
        localTimer.reset();
        robot.drive.runToTargets(Math.signum(distanceInches) * maxPower,
                                Math.signum(distanceInches) * maxPower);
        while (opModeIsActive() && robot.drive.isDriveBusy() && localTimer.seconds() < maxTime) {
            telemetry.update();
        }
        robot.drive.setMotorPowers(0);
    }

    void turnDegrees(double angleDegrees, double maxPower, double maxTime) {
        ElapsedTime localTimer = new ElapsedTime();

        double angleFraction = angleDegrees / 360;

        robot.drive.setTargetsTicks(MaccaDrive.inchesToEncoderTicks((13.75 * 2 * Math.PI) / angleFraction),
                                    -MaccaDrive.inchesToEncoderTicks((13.75 * 2 * Math.PI) / angleFraction));
        localTimer.reset();
        robot.drive.runToTargets(Math.signum(angleDegrees) * maxPower,
                                -1.0 * Math.signum(angleDegrees) * maxPower);
        while (opModeIsActive() && robot.drive.isDriveBusy() && localTimer.seconds() < maxTime) {
            telemetry.update();
        }
        robot.drive.setMotorPowers(0);
    }
}
