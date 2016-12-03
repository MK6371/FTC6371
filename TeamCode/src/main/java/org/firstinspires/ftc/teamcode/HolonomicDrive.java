package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 292486 on 9/20/2016.
 */

@TeleOp(name = "Holonomic Drive", group = "Teleop Drive")
@Disabled
public class HolonomicDrive extends OpMode {

    HardwareHolonomic robot = new HardwareHolonomic();
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private double x, y;
    //////////////////////////
    @Override
    public void init() {
        robot.init(hardwareMap);    //Initialize all the robot components
    }

    @Override
    public void init_loop(){

    }
    //////////////////////////
    @Override
    public void start(){
        timer.reset();
    }

    @Override
    public void loop() {
        /* 1 joystick controls the whole robot by adding the individual components */
        /*this does things
        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        robot.frontLeft.setPower(x + y);
        robot.frontRight.setPower(x - y);
        robot.backLeft.setPower(-x + y);
        robot.backRight.setPower(-x - y);
        */

        /* 2 joysticks: left controls y-axis movement and right controls x-axis movement (add components) */
        y = -gamepad1.left_stick_y;
        x = gamepad1.right_stick_x;
        robot.frontLeft.setPower(Range.clip(x + y, 1, 1));
        robot.frontRight.setPower(Range.clip(x - y, 1, 1));
        robot.backLeft.setPower(Range.clip(-x + y, 1, 1));
        robot.backRight.setPower(Range.clip(-x - y, 1, 1));

        if (gamepad2.left_bumper)
            robot.intake.setPower(-.75);
        else if (gamepad2.right_bumper)
            robot.intake.setPower(.75);

        //if (gamepad2.a) robot.servo.setPosition(90);
        //else robot.servo.setPosition(0);

        if (gamepad2.x) robot.shooter.setPower(1);
        else robot.shooter.setPower(0);



    }
    //////////////////////////
    private double scaleInput(double input){
        double scaled = 0;


        return scaled;
    }
}
