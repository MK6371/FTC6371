package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 292486 on 10/26/2016.
 */
//To be composed in a Hardware classtype, only heading and rotation should be accessible
public class GyroTest {

    private GyroSensor gyro;
    private ElapsedTime timer;
    private double lastTime, deltaTime;
    private double lastRotation;
    private HardwareMap map;
    public double heading;  //In degrees

    public GyroTest(HardwareMap map){
        this(map, "gyro");
    }

    public GyroTest(HardwareMap map, String name){
        this.map = map;
        gyro = map.gyroSensor.get(name);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    //Robot must be still. Test if we need it
    public void calibrate(){
        double averageRotation = 0;
        for(int i = 0; i < 1000; i++){
            averageRotation += gyro.getRotationFraction();
        }
        averageRotation /= 1000;
    }

    public void resetHeading(){
        heading = 0;
    }

    public void updateHeading(){
        deltaTime = timer.time() - lastTime;
        heading += (1/2) * (deltaTime) * (gyro.getRotationFraction() + lastRotation)*360;   //Trapezoidal integration

        lastTime = timer.time();
        lastRotation = gyro.getRotationFraction();
    }
}
