package org.firstinspires.ftc.teamcode.concept;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="ComplexityFreeAuto")
public class ConceptComplexityFreeAutoDrive extends LinearOpMode {

    private DcMotorEx front_left, front_right, back_left, back_right;

    @Override
    public void runOpMode() throws InterruptedException {
        front_left = hardwareMap.get(DcMotorEx.class, "front_left"); // Port 0
        front_right = hardwareMap.get(DcMotorEx.class, "front_right"); // Port 1
        back_left = hardwareMap.get(DcMotorEx.class,"back_left"); // Port 2
        back_right = hardwareMap.get(DcMotorEx.class,"back_right"); // Port 3

        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive() && front_left.getCurrentPosition() < 1000) {
            front_left.setPower(2.5 * (1000 - front_left.getCurrentPosition()));
            front_right.setPower(2.5 * (1000 - front_left.getCurrentPosition()));
            back_left.setPower(2.5 * (1000 - front_left.getCurrentPosition()));
            back_right.setPower(2.5 * (1000 - front_left.getCurrentPosition()));
        }
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
}
