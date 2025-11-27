
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous
public class SimpleAuto extends LinearOpMode {

    DriveTrain      dt = new DriveTrain();
    IMUOdometry     imu = new IMUOdometry();
  //  RobotFunctions  func = new RobotFunctions(1.0, 1.0, 1.0, h);


    public void runOpMode(){

        dt.init(hardwareMap);
        //imu.init(hardwareMap);
        //func.init(hardwareMap);

        while (opModeIsActive()){

        }

    }

}
