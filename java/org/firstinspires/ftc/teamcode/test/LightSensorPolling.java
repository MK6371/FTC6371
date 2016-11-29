package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.HardwareHolonomic;

/**
 * Created by 292486 on 11/28/2016.
 */

public class LightSensorPolling extends OpMode {

    private HardwareHolonomic robot;

    @Override
    public void init() {
        robot = new HardwareHolonomic();
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

    }
}
