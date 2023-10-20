// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limelightSubsystem extends SubsystemBase {
  /** Creates a new limelightSubsystem. */
  private final NetworkTable m_limelightTable;
  private double tv, tx, ta, ty;
  
  public limelightSubsystem() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight"); 
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tv = m_limelightTable.getEntry("tv").getDouble(0); 
    tx = m_limelightTable.getEntry("tx").getDouble(0); 
    ta = m_limelightTable.getEntry("ta").getDouble(0);
    ty = m_limelightTable.getEntry("ty").getDouble(0); 
  }

  public NetworkTable limelighTableRead(){
    return m_limelightTable; 
  }

  public double tx(){
    return tx; 
  }

  public double specificTx(int index){
    return m_limelightTable.getEntry("tx" + index).getDouble(0); 
  }

  public double ty(){
    return ty; 
  }

  public double ta(){
    return ta; 
  }

  public double tv(){
    return tv; 
  }

  public int pipeline(){
    return 1; 
  }
  
  public void setPipeline(int pipelineNumber){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineNumber);
  }

  public void setLED(int ledMode){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledMode);
  }
  
  public int numberOfTargets(){
    double tvValues = m_limelightTable.getEntry("tv").getDouble(0);
    int tvVal = (int) tvValues; 
    return tvVal; 
  }

  public double targetArea(int index){
    return m_limelightTable.getEntry("ta" + index).getDouble(0); 
  }

}
