package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PIDTutorial extends LinearOpMode {

    private DcMotorEx shooterMotor;

    public static double speed = 1200;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");


        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                PID(speed);

                telemetry.update();
            }
        }
    }

    double integral = 0;
    double lastError = 0;

    public void PID(double targetVelocity) {

        PIDTimer.reset();

        double currentVelocity = shooterMotor.getVelocity();

        double error = targetVelocity - currentVelocity;

        integral += error * PIDTimer.time();

        double deltaError = error - lastError;
        double derivative = deltaError / PIDTimer.time();

        pidGains.p = pidCoeffs.p * error;
        pidGains.i = pidCoeffs.i * integral;
        pidGains.d = pidCoeffs.d * derivative;

        shooterMotor.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);

        lastError = error;
    }
}
