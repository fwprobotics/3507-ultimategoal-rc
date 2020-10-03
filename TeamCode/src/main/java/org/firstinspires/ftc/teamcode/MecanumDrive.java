package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "MecanumDrive", group = "TeleOp")
public class MecanumDrive extends LinearOpMode {

    Drivetrain drivetrain;

    @Override
    public void runOpMode() {

        drivetrain = new Drivetrain(this, hardwareMap, telemetry);

        // Ensuring correct subsystem statuses

        telemetry.addLine("Ready and WAITING :)");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        if (opModeIsActive()) {

            telemetry.clearAll();

            while (opModeIsActive()) {

                drivetrain.JoystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                telemetry.update();

            }
        }
    }
}
