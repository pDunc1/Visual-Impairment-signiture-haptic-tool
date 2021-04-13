/**
 **********************************************************************************************************************
 * @file       Guided Exploration.pde
 * @author     Paul Duncan Modified from Base Template Created By Steve Ding and Colin Gallacher
 * @version    V3
 * @date       20 - March - 2021
 * @brief      Haply Haptic Device to allow for Proportional controll following Bezier Coordinate plots
 *             
 **********************************************************************************************************************
 */
 
 
 
 /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import processing.pdf.*;
import processing.sound.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 3;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           rendering_force                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/*Position Control*/

PVector           error                               = new PVector(0,0); 
PVector           targetPosition                      = new PVector(0,0);
int               currentIndex                        = 0;
int               simulationTimeStep                 = 0;
float             k_p                                 = 3000; // Gain

/* task space */
PVector           pos_ee                              = new PVector(0, 0); //position of end effector
PVector           f_ee                                = new PVector(0, 0); //force of end effector

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  //Haply Fisica world width
float             worldHeight                         = 10.0;  // Haply Fisica world height

float             edgeTopLeftX                        = 0.0; //Coordinate of edge top left X 0.0
float             edgeTopLeftY                        = 0.0; // Coordinate of edge top left Y 0.0
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

/* Bezier Curve Coordinates */
FloatList         XCoordinates;
FloatList         YCoordinates;
int               POINTS                              =   11; // sets 11 points for 9 bezier coordinates
PVector[]         BZ                                   = new PVector[POINTS]; //Sets parameter for coordinates

/* Sound*/
SinOsc[]          sineWaves; // Array of sines
float[]           sineFreq; // Array of frequencies
int               numSines = 3; // Number of oscillators to use
SinOsc            sine; //Pan
SoundFile         beep; // beep
/* Initialization of virtual tool */
HVirtualCoupling  s;


/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 400); // Processing sketch screen size
  background(255); //white
  
  /*Float lists*/
  XCoordinates = new FloatList();
  YCoordinates = new FloatList();

  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, "/dev/cu.usbmodem14101", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CW, 1);
  widgetOne.add_actuator(2, CW, 2);
 
  widgetOne.add_encoder(1, CW, 180, 13824, 1);
  widgetOne.add_encoder(2, CW, 0, 13824, 2);
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
   /*Sound Setup*/
  sineWaves = new SinOsc[numSines]; // Initialize the oscillators
  sineFreq = new float[numSines]; // Initialize array for Frequencies
  for (int i = 0; i < numSines; i++) {
    // Calculate the amplitude for each oscillator
    float sineVolume = (1.0 / numSines) / (i + 1);
    // Create the oscillators
    sineWaves[i] = new SinOsc(this);
    // Start Oscillators
    sineWaves[i].play();
    // Set the amplitudes for all oscillators
    sineWaves[i].amp(sineVolume);
  }
  /*Pan*/
  sine = new SinOsc(this);
  sine.play();
  
  /* Beep */
  beep = new  SoundFile(this, "beep.mp3"); // Relate beep mp3 to beep variable

  

  /*Matlab Bezier Points File*/
  Table Bezier_Points = loadTable("BezierPoints10.csv");
  
  /* X and Y coordinates of Bezier Coordinates from csv fil*/
  BZ[0] = new PVector(Bezier_Points.getFloat(0, 0), Bezier_Points.getFloat(1, 0));
  BZ[1] = new PVector(Bezier_Points.getFloat(0, 1), Bezier_Points.getFloat(1, 1));
  BZ[2] = new PVector(Bezier_Points.getFloat(0, 2), Bezier_Points.getFloat(1, 2));
  BZ[3] = new PVector(Bezier_Points.getFloat(0, 3), Bezier_Points.getFloat(1, 3));
  BZ[4] = new PVector(Bezier_Points.getFloat(0, 4), Bezier_Points.getFloat(1, 4));
  BZ[5] = new PVector(Bezier_Points.getFloat(0, 5), Bezier_Points.getFloat(1, 5));
  BZ[6] = new PVector(Bezier_Points.getFloat(0, 6), Bezier_Points.getFloat(1, 6));
  BZ[7] = new PVector(Bezier_Points.getFloat(0, 7), Bezier_Points.getFloat(1, 7));
  BZ[8] = new PVector(Bezier_Points.getFloat(0, 8), Bezier_Points.getFloat(1, 8));
  BZ[9] = new PVector(Bezier_Points.getFloat(0, 9), Bezier_Points.getFloat(1, 9));
  BZ[10] = new PVector(Bezier_Points.getFloat(0, 10), Bezier_Points.getFloat(1, 10));
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75));  // Avatar size
  s.h_avatar.setDensity(5);
  s.h_avatar.setFill(255,0,0); // Avatar colour e.g red
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); // Initialise avatar in position e.g top centre
  
  /* World conditions setup */
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); //Sets edge walls
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  world.draw();
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
  
  /*BezierCurve Setup */
  //Visual representation of Bezier coordinates and corresponding lines to for trace on Processing sketch window
  //uncomment to view. Modified from: https://forum.processing.org/two/discussion/19164/make-a-bezier-curve-with-more-than-2-control-points
  
  /*translate(400, 150); // Translate to position within Haply Fisica world
  scale(2); // Scale to view trace created by points
   //draw control points
  stroke(255, 255, 0); // Line colour (R,G,B)
  fill(0, 255, 0); // Colour fill
  for (int i = 0 ;  i < POINTS ; i++) {
    ellipse(BZ[i].x, BZ[i].y, 4, 4); // Create elipses for Bezier coordinates from csv file. size 5x5
    if (i != 0) {
      line(BZ[i].x, BZ[i].y, BZ[i - 1].x, BZ[i - 1].y); // Create lines for Bezier coordinates
    }
  }
  */
  
  // draw bezier points
  // Number at end determines how many points plotted
  stroke(0, 0, 255); // Line Colour
  for (float i = 0 ; i <= 1 ; i += 1 / 25.0) { // Change last number to determine how many points to recreate trace e.g 25.0 = 25 points
    bezPoint(i);
  } 
  
}


/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
    /*Sound*/
    //Map mouseY from 0 to 1
  float yoffset = map(s.h_avatar.getY(), 0, worldHeight, 1, 0);
  //Map mouseY logarithmically to 150 - 1150 to create a base frequency range
  float frequency = pow(1000, yoffset) + 350;
  //Use mouseX mapped from -0.5 to 0.5 as a detune argument
  float detune = map(s.h_avatar.getX(), 0, worldWidth, -0.5, 0.5);

  for (int i = 0; i < numSines; i++) { 
    sineFreq[i] = frequency * (i + 1 * detune);
    // Set the frequencies for all oscillators
    sineWaves[i].freq(sineFreq[i]);
    
  }
  
  // Map mouseX from -1.0 to 1.0 for left to right
  sine.freq(150); //pan frequency
  sine.pan(map(s.h_avatar.getX(), 0, worldWidth, -1.0, 1.0)); //Pan function
   
  world.draw(); 
}
/* end draw section ****************************************************************************************************/

/* 10th Order Bezier equation utilised to convert Bezier coordinates into trace */
/*Kn10=@(s) [(1.*(s.^0).*((1-s).^10)); (10.*(s.^1).*((1-s).^9)); (45.*(s.^2).*((1-s).^8)); ... 
            (120.*(s.^3).*((1-s).^7)); (210.*(s.^4).*((1-s).^6)); (252.*(s.^5).*((1-s).^5)); ...
            (210.*(s.^6).*((1-s).^4)); (120.*(s.^7).*((1-s).^3)); (45.*(s.^8).*((1-s).^2)); ...
            (10.*(s.^9).*((1-s).^1)); (1.*(s.^10).*((1-s).^0))];
*/
//modified from: https://forum.processing.org/two/discussion/19164/make-a-bezier-curve-with-more-than-2-control-points

void bezPoint(float s) {
  // First set of indices from 10th order Bezier equation
  float p0 = 1;      //s^0
  float p1 = p0 * s; //s^1
  float p2 = p1 * s; //s^2
  float p3 = p2 * s; //s^3
  float p4 = p3 * s; //s^4
  float p5 = p4 * s; //s^5
  float p6 = p5 * s; //s^6
  float p7 = p6 * s; //s^7
  float p8 = p7 * s; //s^8
  float p9 = p8 * s; //s^9
  float p10 = p9 * s; //s^10
  
  // Second Set of Indices from 10th order Bezier Equation
  float pp0 = 1;             //(1-s)^0
  float pp1 = pp0 * (1 - s); //(1-s)^1
  float pp2 = pp1 * (1 - s); //(1-s)^2
  float pp3 = pp2 * (1 - s); //(1-s)^3
  float pp4 = pp3 * (1 - s); //(1-s)^4
  float pp5 = pp4 * (1 - s); //(1-s)^5
  float pp6 = pp5 * (1 - s); //(1-s)^6
  float pp7 = pp6 * (1 - s); //(1-s)^7
  float pp8 = pp7 * (1 - s); //(1-s)^8
  float pp9 = pp8 * (1 - s); //(1-s)^9
  float pp10 = pp9 * (1 - s); //(1-s)^10
  
  /* Bezier X Coordinates */
  float BZx = pp10 * BZ[0].x   // Binomial coefficient equation 1 for 10th order bezier X coordinate 1
    + 10 * p1 * pp9 * BZ[1].x  // Binomial coefficient equation 2 for 10th order bezier X coordinate 2
    + 45 * p2 * pp8 * BZ[2].x  // Binomial coefficient equation 3 for 10th order bezier X coordinate 3
    + 120 * p3 * pp7 * BZ[3].x // Binomial coefficient equation 4 for 10th order bezier X coordinate 4
    + 210 * p4 * pp6 * BZ[4].x // Binomial coefficient equation 5 for 10th order bezier X coordinate 5
    + 252 * p5 * pp5 * BZ[5].x // Binomial coefficient equation 6 for 10th order bezier X coordinate 6 
    + 210 * p6 * pp4 * BZ[6].x // Binomial coefficient equation 7 for 10th order bezier X coordinate 7
    + 120 * p7 * pp3 * BZ[7].x // Binomial coefficient equation 8 for 10th order bezier X coordinate 8
    + 45 * p8 * pp2 * BZ[8].x  // Binomial coefficient equation 9 for 10th order bezier X coordinate 9 
    + 10 * p9 * pp1 * BZ[9].x  // Binomial coefficient equation 10 for 10th order bezier X coordinate 10
    + p10 * BZ[10].x;          // Binomial coefficient equation 11 for 10th order bezier X coordinate 11
    
  /* Bezier Y Coordinates*/
  float BZy = pp10 * BZ[0].y   // Binomial coefficient equation 1 for 10th order bezier Y coordinate 1
    + 10 * p1 * pp9 * BZ[1].y  // Binomial coefficient equation 2 for 10th order bezier Y coordinate 2
    + 45 * p2 * pp8 * BZ[2].y  // Binomial coefficient equation 3 for 10th order bezier Y coordinate 3
    + 120 * p3 * pp7 * BZ[3].y // Binomial coefficient equation 4 for 10th order bezier Y coordinate 4
    + 210 * p4 * pp6 * BZ[4].y // Binomial coefficient equation 5 for 10th order bezier Y coordinate 5
    + 252 * p5 * pp5 * BZ[5].y // Binomial coefficient equation 6 for 10th order bezier Y coordinate 6
    + 210 * p6 * pp4 * BZ[6].y // Binomial coefficient equation 7 for 10th order bezier Y coordinate 7
    + 120 * p7 * pp3 * BZ[7].y // Binomial coefficient equation 8 for 10th order bezier Y coordinate 8
    + 45 * p8 * pp2 * BZ[8].y  // Binomial coefficient equation 9 for 10th order bezier Y coordinate 9
    + 10 * p9 * pp1 * BZ[9].y  // Binomial coefficient equation 10 for 10th order bezier Y coordinate 10
    + p10 * BZ[10].y;          // Binomial coefficient equation 11 for 10th order bezier Y coordinate 11
    
    point(BZx, BZy); // Creates point from X coordinate and Y coordinate along recreated trace
    
    XCoordinates.append(BZx + 15); //Final translation to fit onto Haply workspace X Coordinates
    YCoordinates.append(BZy - 7); //Final translation to fit onto Haply workspace Y Coordinates
    
    /*Print first coordinate plots prior to final translation */
    print(" x coordinate is", BZx, " ");
    print(" y coordinate is", BZy, " ");
    
    /*Print first coordinate plot after final translation*/
    print(" X coordinate is", XCoordinates, " ");
    print(" Y coordinate is", YCoordinates, " ");
   
}
    
   
/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    rendering_force = true;
    
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
      angles.set(widgetOne.get_device_angles()); 
      pos_ee.set(widgetOne.get_device_position(angles.array())); //<>//
      pos_ee.set(pos_ee.copy().mult(200)); //<>//
      error.set(targetPosition.copy().sub(pos_ee)); // Set error between pos of end effector and target position
      error.set(error.copy().mult(k_p)); // Multiply copy of error by gain //<>//
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+2, edgeTopLeftY+(pos_ee).y-7); 
    s.updateCouplingForce();
    f_ee.set(error.array()); // Force of end effector to error array to move //<>//
    f_ee.div(20000); // Increase to reduce instability effects e.g 40000 //<>//
    
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();
  
    world.step(1.0f/1000.0f); // 1.0/1000 = 1ms
  
    rendering_force = false;
    
    /* Speed of end effector tool to follow positions*/
    simulationTimeStep++;
    if (simulationTimeStep == 600) // Speed of End Effector. E.g every 600Hz step to next point
    { //<>//
       targetPosition.set(XCoordinates.get(currentIndex), YCoordinates.get(currentIndex)); //<>//
       simulationTimeStep = 0; //<>//
       beep.play(); // Play beep sound at start of trace
       currentIndex++;
       
       /* Testing Coordinates */
       print("Position of end effector is ", pos_ee, " "); 
       print("Avatar Xcoordinate is ", s.getAvatarPositionX(), " "); 
       print("Avatar Ycoordinate is ", s.getAvatarPositionY(), " "); 
       print("End Effector Xcoordinate is ", s.getToolPositionX(), " "); 
       print("End Effector Ycoordinate is ", s.getToolPositionY(), " ");
        
    }
   
    if (currentIndex == 1)
    {
      beep.play();
    }
      else
      {
        beep.stop();
      }
    if (currentIndex == 25) // Play beep.mp3 at end of trace
    {
      beep.play();
    }
      else
      {
        beep.stop();
      }
    }
  
}
/* end simulation section **********************************************************************************************/
