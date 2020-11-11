package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
Arm subsystem. By Jake, 11/9/20.
 */


public class Arm{

    public DcMotor armMotor;
    public DcMotor rightLiftMotor;
    public LinearOpMode l;
    public Telemetry realTelemetry;

    @Config
    public static class ArmConstants {
        public static double arm_modifier = 0.8;
    }

    public Arm(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;

        realTelemetry = telemetry;

        armMotor = hardwareMap.dcMotor.get("armMotor");

        // Different motor configurations depending on use case

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void teleOpControl(double input){
        armMotor.setPower(input * ArmConstants.arm_modifier);
        realTelemetry.addData("Arm Encoder Count:", armMotor.getCurrentPosition());
    }


}
