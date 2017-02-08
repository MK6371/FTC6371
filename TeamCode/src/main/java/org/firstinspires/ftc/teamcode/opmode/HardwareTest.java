package org.firstinspires.ftc.teamcode.opmode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.HardwareHolonomic;

import java.util.ArrayList;

/**
 * Created by 292486 on 11/30/2016.
 */

@TeleOp(name = "Hardware Diagnostics")
public class HardwareTest extends OpMode {

    private HardwareHolonomic robot;
    private String menu = "Motors";

    @Override
    public void init() {
        robot = new HardwareHolonomic();
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        switch(menu)
        {
            case "Motors":
        }
    }
}
