void setup() {
  // input settings
  int depthmin = 2ft(example);
  int depthmax = 20ft(example);
  float Waypoint = {Lat, Long};

  //-------------------------------------------------------------------------------------------------------
  primingSyringes();
  primingBalast();


}

//----------------------------------------------------------------------------------------------------------
// Main loop
void loop() {
  //while CartCompAng is not within tolerance 
  //{
    //loop buoyancy engine to keep forward momentum
    //loop ballast correction
    //CheckComp_GPS(Compass, GPS{})

  //}
  //once angle is matched proceed forward.
  //While CartCompAng is within tolerance 
  //{
  //loop buoyancy engine
  //CheckComp_GPS(Compass, GPS{})
  //}


}

int CheckComp_GPS(GPS{}, Waypoint)
{
  readCompass()                                                        // Function that reads the current compass angle reading
  readGPS()                                                            // Function that reads current Lat Long from GPS                                                 
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

  return AngleCorr; 
{


// -----------------------------------------------------------------------------------------------------------------------
// Bouancy engine and ballast controlls

void primingSyringes()
{

}

void primingbalast()
{

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

void turningLeft()
{
  // Turning Left Function 
  int Rollangle;


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