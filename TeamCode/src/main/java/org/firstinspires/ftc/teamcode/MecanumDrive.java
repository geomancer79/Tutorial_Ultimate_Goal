package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MecanumDrive", group="tutorial")
public class MecanumDrive extends LinearOpMode {
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;

    @Override
    public void runOpMode(){
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontRight = hardwareMap.dcMotor.get("frontRight");

        waitForStart();
        while (opModeIsActive()){
            double horizontal = 0.7 * gamepad1.left_stick_x;
            double vertical = -0.7 * gamepad1.left_stick_y;
            double turn = -0.7 * gamepad1.right_stick_x;

            backLeft.setPower(vertical + turn - horizontal);
            frontLeft.setPower(vertical + turn + horizontal);
            backRight.setPower(vertical - turn + horizontal);
            frontRight.setPower(vertical - turn - horizontal);
        }
    }
}
