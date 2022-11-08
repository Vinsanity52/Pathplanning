void setup() {
  // input settings
  int depthmin = 2ft(example);
  int depthmax = 20ft(example);
  float Waypoint = {Lat, Long};
  float AngleTolerance = {.01,.01};                                     
  float DistTolerance = 1;
  int command = 0;
  float targetRoll = 0;


  //-------------------------------------------------------------------------------------------------------

}

//----------------------------------------------------------------------------------------------------------
// Main loop
void loop() {
  //command 0 will wait for a command
  //command 1 will priming syringes
  //command 2 will prime balasts
  //command 3 will start autonomus navigation

  while(command != 3)
  {
    CheckCommands();

      if(command = 1)
    {
      primingSyringes();
    }

    else if(command = 2)
    {
      primingBalast();
    }

  }

  CheckCommands();
  CheckComp_GPS(Waypoint);

  if(Dist > DistTolerance)                                              //If calculated distance is greater than our tollerance run navigation
  {
    if(abs(AngleDiffrence) > AngleTolerance)                            //If the angle the glider is facing is not within our tolerance, correct the angle 
    {
      CheckComp_GPS(Waypoint);                                          // check GPS and compass and calculate distance and angle diffrence
      AngleCorrection(AngleDiffrence);                                  // Angle Correction to turn the glider 
      moveForward();
      CheckComp_GPS(Waypoint);

    }
    //once angle is matched proceed forward.
    else if(abs(AngleDiffrence) <= AngleTolerance)
    {
      targetRoll = 0;
      CheckComp_GPS(Waypoint);
      moveForward();
      RollBalance(targetRoll);
      CheckComp_GPS(Waypoint);
    }
  }
  else
  command = 0;

}

int CheckComp_GPS(Waypoint)
{
  readCompass();                                                        // Function that reads the current compass angle reading
  readGPS();                                                            // Function that reads current Lat Long from GPS                                                 
  float CompAngle = CompassReadout;                                    //Compass readout angle

  float CartCompAng = {Cos(CompAngle), Sin(CompAngle)};                //Compass readout converted to unit circle Cartisiean angle
  float GliderLocation = {Lat, Long};                                  //Glider’s current location

  float Waypoint = {Lat, Long};                                        //Waypoint destination

  //vector between glider location and waypoint
  float DistCord= {(Waypoint(1) - GliderLocation(1)), ((Waypoint(1) - GliderLocation(1))};
  float Dist = sqrt((DistCord(1))^2+(DistCord(2))^2);
  float NormDist = DistCord/Dist;
  float DistAngle = arctan(NormDist(2) / NormDist(1));
  float CartDistAngle = {Cos(DistAngle), Sin(DistAngle)};

  //Angle difference between compass angle and the angle it should face
  float AngleCorr = {CartCompAng(1) - CartDistAngle(1), CartCompAng(2) - CartDistAngle(2)};
  float AngleDiffrence = arctan(AngleCorr(2)/AngleCorr(1))

  return AngleDiffrence, Dist, CompassReadout; // I dont think you can return two varriables like this might need to be a struct
{


// -----------------------------------------------------------------------------------------------------------------------
// Bouancy engine and ballast controlls

int primingSyringes()
{


  // once priming process is finised return command varriable = 0 
  command = 0;
  return command;
}

void primingbalast()
{


  // once priming process is finised return command varriable = 0 
  command = 0;
  return command;
}
// this function activates the moving foward loop.
void moveForward()
{

  int pitch;
  int servo CW;
  int servo CCW;
  int depth;
  int depthmin = 2ft(example);
  int depthmax = 20ft(example);

  //while {
    //if (depth=depthmin)
      // activate servo CW for x amount of time (full time to retract syringes)
      // check depth 
      //if (depth=maxdepth)
        //activate servo CCW for x amount of time ( full time to extend syringes)
      // check depth
    //else
        // check depth
  PitchBalance()
  RollBalance()
}

void AngleCorrection(AngleDiffrence)
{
  yawChange = AngleDiffrence           //Required yaw change
  if (AngleCorr(2) > 0)                                     //If positive turn left
  {
    turningLeft(yawChange);
  } 
  else if (AngleCorr(2) < 0)                                // If negative turn Right
  {
    turningRight(yawChange);
  }
  RollBalance(targetRoll);
}

float turningLeft(yawChange)
{
  // Turning Left Function 
  
  if (yawChange > 90)
  {
    float targetRoll = 45                           // target roll angle in degrees
    float RollTolerance = 1;                        // tolerance for roll angle
    rollDiffrence(targetRoll);
    while()
    {

    }    
  }


  // Activate Solenoids 2 and 3
  //Activate pump for certain time (Large amount)
  //turn off pump
  //close solenoids 2 and 3

  // Activate Solenoids 1 and 4
  //Activate pump for certain time (Large amount same as before)
  //turn off pump
  //close solenoids 1 and 4
  // hold until roll angle is achived with a tolerance 

  //Call to Roll Balance function 
  return targetRoll; 
}

void turningRight()
{
  //Turning Right Function  
  //int Rollangle

  // Activate Solenoids 1 and 4
  //Activate pump for certain time (Large amount same as before)
  //turn off pump
  //close solenoids 1 and 4
  // hold until roll angle is achived with a tolerance 

  // Activate Solenoids 2 and 3
  //Activate pump for certain time (Large amount)
  //turn off pump
  //close solenoids 2 and 3


  //Call to Roll Balance function 

}

void PitchBalance()
{
   //Pitch Balance Function 
  //int LowTolerance = -5degrees;
  //int Tolerancehigh = 5degrees;
  //int pitch;
  // int servo CW
  // int servo CCW

  // Do{
    //Loop{
    //if pitch < Tolerancelow
      //     run servo CW for x time
      //     wait a certain time
        //     read pitch
    //}

    //Loop{
    //if pitch> Tolerancehigh
        //run servo CCW for x time
        //wait a certain
        //read pitch
    //}

  //While pitch !=0

}

void rollBalance()
{
  //Roll Balance Function 
  int Tolerancelow = -5degrees;
  int Tolerancehigh = 5degrees;
  int Roll;


  // Do{
    //Loop{
    //if Roll < Tolerancelow
      //     call to CWRollBalance for x amouint of time
      //     wait a certain time
      //     call to CCWRollBalance for x amount of time, same amount as before to balance ballast water level
        //     read Roll
    //}

    //Loop{
    //if pitch> Tolerancehigh
        // call to CCWRollBalance for x amount of time
        //wait a certain time      
        //call to CWRollBalance for x amount of time, same amount as before to balance ballast water level
        //read Roll
    //}

  //While Roll !=0

  //CWRollBlance Function
  // Activate Solenoids 1 and 4
  //Activate pump for certain time (small amount)
  //turn off pump
  //close solenoids 1 and 4


  //CCWRollBalance Function
  // Activate Solenoids 2 and 3
  //Activate pump for certain time (small amount)
  //turn off pump
  //close solenoids 2 and 3
}
// ------------------------------------------------------------------------------------------------------------
// Code that utilizes sensors and informaiton gathered from the bouy


int readCompass()
{
  return CompassReadout
}
int readGPS()
{
  return GPS{}
}



//---------------------------------------------------------------------------------------------------------------
// Code involving on board sensors

float readGyroscope()
{
  float GliderRoll                        //Glider's actual roll angle according to Gyroscope
  return GliderRoll
}

int readPressure()
{

}

rollDiffrence(targetRoll)
{
  readGyroscope()
}
------------------------------------------------------------------------------------------------------------------
// Mobile app communication code

int CheckCommands()
{
  
}