package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Gyro;

/**
 * Created by 292486 on 9/20/2016.
 */
public class HardwareHolonomic {

    public DcMotor frontLeft, frontRight, backLeft, backRight;

    public DcMotor shooterRed, shooterBlue; //Facing the 2 motors from the outside of the robot, red it right, blue is left
    public DcMotor intake;

    public Servo servo;

    public LightSensor lightBeacon;
    public LightSensor lightFloor;

    public UltrasonicSensor sonar;

    public Gyro gyro;

    public ColorSensor sensorRgb;

    private HardwareMap hMap;
    private ElapsedTime period = new ElapsedTime();

    public HardwareHolonomic(){

    }

    public void init(HardwareMap map){
        hMap = map;

        frontLeft = hMap.dcMotor.get("front_left");
        frontRight = hMap.dcMotor.get("front_right");
        backLeft = hMap.dcMotor.get("back_left");
        backRight = hMap.dcMotor.get("back_right");

        shooterRed = hMap.dcMotor.get("shooter_red");
        shooterBlue = hMap.dcMotor.get("shooter_blue");
        intake = hMap.dcMotor.get("intake");

        //servo = hMap.servo.get("servo");

        //lightBeacon = hMap.lightSensor.get("lightB");
        lightFloor = hMap.lightSensor.get("lightF");

        sonar = hMap.ultrasonicSensor.get("sonar");

        //gyro = new Gyro(hMap, "gyro");

        sensorRgb = hMap.colorSensor.get("color");
        hMap.deviceInterfaceModule.get("dim").setDigitalChannelMode(0, DigitalChannelController.Mode.OUTPUT);
        hMap.deviceInterfaceModule.get("dim").setDigitalChannelState(0, false);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /***
     * [Taken from HardwarePushbot. What we can use it for is not immediately apparent]
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
