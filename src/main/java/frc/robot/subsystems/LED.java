// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Ports;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  /** Creates a new leds. */
  AddressableLED led; 
  AddressableLEDBuffer ledBuffer; 
  //Timer timer; 
  int rainbowFirstPixelHue;    
  



  public LED() {
    led = new AddressableLED(Ports.LEDPorts.LED_PORT); 
    ledBuffer = new AddressableLEDBuffer(250); 
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer); 
    led.start(); 
    

  }

  public void setLedPurple(){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, 160, 32, 340);
    }
    led.setData(ledBuffer); 
  }

  public void setLedGreen(){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, 0, 225, 0);
    }
    led.setData(ledBuffer); 
  }

  public void setDefault(){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
    System.out.println("Default Working ");
  }


  public void setRainbow(){ 
    //timer.start(); 
   // if (timer.get() <= 3){
      for(int i = 0; i < ledBuffer.getLength(); i++){ 
        final int hue = (rainbowFirstPixelHue + (i * 180/ ledBuffer.getLength())) % 180; 
        ledBuffer.setHSV(i, hue, 255, 128); 
      }
      rainbowFirstPixelHue +=3; 
      rainbowFirstPixelHue %= 180;
      led.setData(ledBuffer);
    
  //else {
   // timer.stop(); 
    
    System.out.println("Rainbow Running ");
  }

  public void setGrupleFlicker(boolean isGreen){ 
   // timer.start();
  
    boolean green = isGreen;
    for (int i = 0; i < (ledBuffer.getLength()); i++) {
      if (green) {
        ledBuffer.setLED(i, Color.kGreen);
        // ledBuffer.setRGB(i, 0, 255, 0); // GREEN
        System.out.println("green");
      } else {
        ledBuffer.setLED(i, Color.kPurple);
        // ledBuffer.setRGB(i, 255, 0, 255); // PURPLE
        System.out.println("purple");

      }
      green = !green;
    }
    led.setData(ledBuffer);
    System.out.println("green purple led test");

    // if (timer.get() == 3) {
    //   led.close();
    }

     public void setGrupleChase(){ 
     for(int i = 0; i < ledBuffer.getLength(); i++){ 
        final int hue = (rainbowFirstPixelHue + (i * 180/ ledBuffer.getLength())) % 180; 
        ledBuffer.setRGB(i, 300, 255, hue );
        
      }
      rainbowFirstPixelHue +=3; 
      rainbowFirstPixelHue %= 180;
      led.setData(ledBuffer);
    }

  
    


  public void setRed(){

      for(int i = 0; i<ledBuffer.getLength(); i++){
        ledBuffer.setRGB(i, 255, 0, 0);

      }
    led.setData(ledBuffer); 
    System.out.println("RED!");
}
  


public void setSolid(int[] solid){ 

      for(int j = 0; j < ledBuffer.getLength(); j++){
        ledBuffer.setRGB(j, solid[0], solid[1], solid[2]);
      }
      led.setData(ledBuffer);
      System.out.println("SOLID"); 

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


