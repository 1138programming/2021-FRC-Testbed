package frc.robot.subsystems;

import java.util.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Base extends SubsystemBase {
    //Creating the Talons
    private final TalonFX leftFrontSpeed, leftBackSpeed, rightBackSpeed, rightFrontSpeed;
    private final TalonFX leftFrontAngle, leftBackAngle, rightBackAngle, rightFrontAngle;

    int CurrentMotor = 0; 
    List<String> motorName;
    List<TalonFX> talons;

    public Base() {
        // Instantiating the Talons
        leftFrontSpeed = new TalonFX(KLeftFrontSpeedTalon);
        leftBackSpeed = new TalonFX(KLeftBackSpeedTalon);
        rightFrontSpeed = new TalonFX(KRightFrontSpeedTalon);
        rightBackSpeed = new TalonFX(KRightBackSpeedTalon);

        leftFrontAngle = new TalonFX(KLeftFrontAngleTalon);
        leftBackAngle = new TalonFX(KLeftBackAngleTalon);
        rightFrontAngle = new TalonFX(KRightFrontAngleTalon);
        rightBackAngle = new TalonFX(KRightBackAngleTalon);

        //put the talons into an array
        talons = new ArrayList<TalonFX>();
        talons.add(leftFrontSpeed);
        talons.add(leftBackSpeed);
        talons.add(rightFrontSpeed);
        talons.add(rightBackSpeed);

        talons.add(leftFrontAngle);
        talons.add(leftBackAngle);
        talons.add(rightFrontAngle);
        talons.add(rightBackAngle);
    
        //Set brake mode
        int i = 0;
        while(i < Talons.size()){
            Talons.get(i).setNeutralMode(NeutralMode.Brake); 
            i = i + 1;
        }

        motorName = new ArrayList<String>();
        motorName.add("leftFrontSpeed");
        motorName.add("leftBackSpeed");
        motorName.add("rightFrontSpeed");
        motorName.add("rightBackSpeed");

        motorName.add("leftFrontAngle");
        motorName.add("leftBackAngle");
        motorName.add("rightFrontAngle");
        motorName.add("rightBackAngle");
    }

    public void UpdateCurrentMotor() {
        if (CurrentMotor < Talons.size()) {
            CurrentMotor = 0;             
        } else {
            CurrentMotor = CurrentMotor + 1;
        }
    }

    public string getCurrentMotor() {
        return motorName.get(CurrentMotor);
    }

    public void move(double PWNM) {
        motorName.get(CurrentMotor).set(ControlMode.PercentOutput, PWM);
    }

    @Override
    public void periodic() {
    }
  
    @Override
    public void simulationPeriodic() {
    }
}
