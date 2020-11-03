package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
Class for controlling robot's intake. Very simple state machine for toggling on/off.
Probably will add some direction control later, just in case. Includes toggling for teleop
and set control for autonomous. By Jake, 1/27/20.
 */

public class Shooter {

    private DcMotor shooterMotor;
    private LinearOpMode l;
    private Telemetry realTelemetry;

    public enum shooterStatuses {
        ON,
        OFF
    }

    private shooterStatuses shooterStatus = shooterStatuses.OFF;
    private boolean inputButtonPressed;

    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(RevRoboticsCoreHexMotor.class);


    public static class ShooterConstants {
        public static double shooter_power = -1.0;
    }


    public Shooter(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry){

        l = Input;
        realTelemetry = telemetry;
        realTelemetry.setAutoClear(true);

        shooterMotor = hardwareMap.dcMotor.get("intakeMotor");

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    // TELEOP FUNCTIONS

    // Toggle on and off intake via inputButton
    public void toggleShooter(boolean inputButton){
        if (inputButton && !inputButtonPressed) {
            switch (shooterStatus) {
                case OFF:
                    inputButtonPressed = true;
                    shooterStatus = shooterStatuses.ON;
                    break;
                case ON:
                    inputButtonPressed = true;
                    shooterStatus = shooterStatuses.OFF;
                    break;
            }
        }

        if (!inputButton) {
            inputButtonPressed = false;
        }
    }

    // Sets power of intake depending on direction
    public void runShooter(){
        if (shooterStatus == shooterStatuses.ON) {
            shooterMotor.setPower(ShooterConstants.shooter_power);

        }
        else if (shooterStatus == shooterStatuses.OFF) {
            shooterMotor.setPower(0);

        }
    }

}
