package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "MecanumDrive", group = "TeleOp")
public class MecanumDrive extends LinearOpMode {

    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;

    @Override
    public void runOpMode() {

        drivetrain = new Drivetrain(this, hardwareMap, telemetry);
        intake = new Intake(this, hardwareMap, telemetry);
        shooter = new Shooter(this, hardwareMap, telemetry);

        // Ensuring correct subsystem statuses

        telemetry.addLine("Ready and WAITING :)");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        if (opModeIsActive()) {

            telemetry.clearAll();

            while (opModeIsActive()) {

                drivetrain.JoystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

                intake.toggleIntake(gamepad1.x);
                intake.directionControl(gamepad1.a);
                intake.runIntake();

                shooter.toggleShooter(gamepad2.x);
                shooter.runShooter();


                telemetry.update();

            }
        }
    }
}
