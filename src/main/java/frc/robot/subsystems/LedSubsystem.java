
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new LedSubsystem. */  
  private AddressableLED led1; 
  // private AddressableLED led2; 

  private AddressableLEDBuffer led1_Buffer; 
  // private AddressableLEDBuffer led2_Buffer; 

  public LedSubsystem() {
    led1 = new AddressableLED(LedConstants.ledPort);
    // led2 = new AddressableLED(3);
    
    led1_Buffer = new AddressableLEDBuffer(LedConstants.ledLength); 
    // led2_Buffer = new AddressableLEDBuffer(60); 

    led1.setLength(led1_Buffer.getLength());
    // led2.setLength(led2_Buffer.getLength());

    led1.setData(led1_Buffer);
    // led2.setData(led2_Buffer);
  
    led1.start();       
    // led2.start();      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOneColour(int r, int g, int b){
      for (var i = 0; i < led1_Buffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        led1_Buffer.setRGB(i, r, g, b);
        // led2_Buffer.setRGB(i, r, g, b);
      }
      
      led1.setData(led1_Buffer);
      // led2.setData(led2_Buffer);
  }

  // public void setPoliceFlash(double startTime){

  //   startTime = System.currentTimeMillis(); 

  //   int r = 0; 
  //   int g = 0; 
  //   int b = 0; 

  //   if(Math.abs(startTime - System.currentTimeMillis()) < 2000){



  //   }

  //   else{
  //     startTime = System.currentTimeMillis(); 
  //     r = 255;
  //     g = 100;
  //     b = 100;
  //   }

  //   setOneColour(r, g, b);
  //     // led2.setData(led2_Buffer);
  // }
}
