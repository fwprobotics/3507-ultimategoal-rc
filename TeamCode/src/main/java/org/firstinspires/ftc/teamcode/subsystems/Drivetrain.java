package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
Drivetrain subsystem. Currently holds teleop driving control. By Jake B, 2019.
 */

public class Drivetrain {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    public LinearOpMode l;
    public Telemetry realTelemetry;


    @Config
    public static class TeleOpDTConstants {
        public static double turning_modifier = 0.5;
        public static double y_modifier = 0.6;
        public static double x_modifier = 0.4;

    }


    public Drivetrain(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;
        realTelemetry = telemetry;
        realTelemetry.setAutoClear(true);

        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        l.idle();
    }

    public void JoystickMovement(double leftStickY, double leftStickX, double rightStickX){

        //Sets motor values based on adding and subtracting joystick values
        double LeftX = -leftStickX * TeleOpDTConstants.x_modifier;
        double LeftY = -leftStickY * TeleOpDTConstants.y_modifier;
        double RightX = -rightStickX * TeleOpDTConstants.turning_modifier;

        double frontLeftVal = ((LeftY - RightX) - LeftX);
        double frontRightVal = ((LeftY + RightX) + LeftX);
        double backLeftVal = ((LeftY - RightX) + LeftX);
        double backRightVal = ((LeftY + RightX) - LeftX);

        frontLeftDrive.setPower(frontLeftVal);
        frontRightDrive.setPower(frontRightVal);
        backLeftDrive.setPower(backLeftVal);
        backRightDrive.setPower(backRightVal);
    }
}