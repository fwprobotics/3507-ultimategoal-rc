package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
Arm subsystem. By Jake, 11/9/20.
 */


public class Arm{

    public DcMotor armMotor;
    public LinearOpMode l;
    public Telemetry realTelemetry;
    public Servo armGripper;

    @Config
    public static class ArmConstants {
        public static double arm_modifier = 0.8;
    }

    public enum armRunMode {
        AUTONOMOUS,
        TELEOP
    }

    public Arm(armRunMode runMode, LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;

        realTelemetry = telemetry;

        armMotor = hardwareMap.dcMotor.get("armMotor");
        armGripper = hardwareMap.servo.get("armGripperServo");

        if (runMode.equals(armRunMode.TELEOP)) {
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (runMode.equals(armRunMode.AUTONOMOUS)) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // This wont work rn with current encoder setup
            armMotor.setTargetPosition(0);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

    }

    public void autoDrive(int msecs, float power){
        armMotor.setPower(power);
        l.sleep(msecs);
        armMotor.setPower(0);
    }

    public void teleOpControl(double input){
        armMotor.setPower(input * ArmConstants.arm_modifier);
        //realTelemetry.addData("Arm Encoder Count:", armMotor.getCurrentPosition());
    }


}
