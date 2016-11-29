package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.Holonomic;
import org.firstinspires.ftc.teamcode.hardware.HardwareHolonomic;

/**
 * Created by 292486 on 9/20/2016.
 */

@TeleOp(name = "Holonomic Drive", group = "Teleop Drive")
public class HolonomicDrive extends OpMode {

    HardwareHolonomic robot = new HardwareHolonomic();
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private double y, x, l, r;
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
        y = -gamepad1.left_stick_y; //For some reason, the left y-axis stick is negative when pushed forward
        x = gamepad1.right_stick_x;
        l = gamepad1.left_trigger;
        r = gamepad1.right_trigger;

        Holonomic.arcade(y, x, l, r, robot);

        if(gamepad1.x)
        {
            robot.shooter.setPower(.7);
        } else {
            robot.shooter.setPower(0);
        }

        if(gamepad1.a)
        {
            robot.intake.setPower(.5);
        } else if(gamepad1.b){
            robot.intake.setPower(-0.5);
        } else {
            robot.intake.setPower(0);
        }

    }
}
