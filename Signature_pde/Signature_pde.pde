 /**
 **********************************************************************************************************************
 * @file       Signature_pde.pde
 * @author     Paul Duncan. Modified from Base Template Created By Steve Ding and Colin Gallacher
 * @version    V2
 * @date       20 - March - 2021
 * @brief      Haply Haptic device for unguided and collaborative exploration
 *             
 **********************************************************************************************************************
 
 /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
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

/* task space */
PVector           pos_ee                              = new PVector(0, 0);
PVector           f_ee                                = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

/* list of contacts with avatar */
ArrayList         <FContact> touching;

/* joints to be created for avatar */
boolean           Created_Join                        = false;
FDistanceJoint    join;
FCircle           gA; // grab radius

/* define linkage state */
boolean           Linkable                             = false;

/* define number of guides total */
int               guide                               = 1;

/* Sound*/
SinOsc[]          sineWaves; // Array of sines
float[]           sineFreq; // Array of frequencies
int               numSines = 3; // Number of oscillators to use
SinOsc            sine; //Pan

/* Initialization of virtual tool */
HVirtualCoupling  s;

/* F_Letters*/

FPoly             T;
FPoly             H;
/* Background */
int               Background; //Setup colour number for background

/* end elements definition *********************************************************************************************/

/* Initialization of letters */
boolean           refresh                         = true;
PImage            letter_a;                          
PImage            letter_b; 
PImage            letter_c;
PImage            letter_d;
PImage            letter_h;
PImage            letter_v;
PImage            letter_s; 
PImage            letter_is;
PImage            letter_bp;
PImage            letter_Join;
PImage            letter_Overtop;
PImage            letter_Alphabet;
/* setup section *******************************************************************************************************/
void setup()
{
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 400); // Processing sketch screen size
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem14101", 0);
   */
  haplyBoard          = new Board(this, "/dev/cu.usbmodem14101", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  println(Serial.list());
  
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
  
  /* Create Grabbable Guide */ 
  FCircle guide;
  guide              = new FCircle(0.5); // Number coorellates to size 
  guide.setPosition(2, 4); // Sets guide position inHaply Fisica world
  guide.setFill(0, 256, 0); // Sets colour (R,G,B)
  guide.setFriction(20); // Needs to be of a value for link
  guide.setDensity(20); // Needs to be of a value for link
  guide.setHaptic(true);
  guide.setName("Guide"); //Sets guide variable name to Guide
  world.add(guide); // Adds guide to worlds
  
  /* grabArea */
  gA                  = new FCircle(0.75);
  gA.setDensity(0);
  gA.setNoStroke();
  gA.setSensor(true);
  gA.setFill(0, 0, 0, 50); //Opacity
  gA.setPosition(3, 3);
  world.add(gA);
  
  /* Background Refresh*/
  Background = 255; //background white
  
  //Images of letters used from Rosemary Sassoon, 2003, Handwriting - The way to teach it. SAGE Publications Inc. Available at: http://psulibrary.palawan.edu.ph/wtbooks/resources/pdf/908615.pdf 
 /*Load Letters */
  letter_a = loadImage("Handwriting/a.JPEG");
  letter_b = loadImage("Handwriting/b.jpg");
  letter_c = loadImage("Handwriting/c.jpg");
  letter_d = loadImage("Handwriting/d.jpg");
  letter_h = loadImage("Handwriting/h.jpg");
  letter_v = loadImage("Handwriting/v.jpg");
  letter_s = loadImage("Handwriting/s.jpg");
  letter_is = loadImage("Handwriting/is.jpg");
  letter_bp = loadImage("Handwriting/bp.jpg");
  letter_Join     = loadImage("Handwriting/Join.jpg");
  letter_Overtop  = loadImage("Handwriting/OvertopJoin.jpg");
  letter_Alphabet = loadImage("Handwriting/Alphabet.jpg");

  /* creation of T shape */ //Used from Steve Ding and Colin Gallacher in SHAPES.pde
  T                   = new FPoly(); 
  T.vertex(-1.5, -1.0);
  T.vertex( 1.5, -1.0);
  T.vertex( 3.0/2.0, 0);
  T.vertex( 1.0/2.0, 0);
  T.vertex( 1.0/2.0, 4.0/2.0);
  T.vertex(-1.0/2.0, 4.0/2.0);
  T.vertex(-1.0/2.0, 0);
  T.vertex(-3.0/2.0, 0);
  T.setPosition(edgeTopLeftX+12, edgeTopLeftY+5); 
  T.setStaticBody(true);
  T.setSensor(true);
  T.setFill(150,150,0);
  world.add(T);
 
   /* Creation of H shape */
  H                 = new FPoly();
  H.vertex(1, 1);
  H.vertex(1, 4);
  H.vertex(2, 4);
  H.vertex(2, 3);
  H.vertex(3, 3);
  H.vertex(3, 4);
  H.vertex(4, 4);
  H.vertex(4, 1);
  H.vertex(3, 1);
  H.vertex(3, 2);
  H.vertex(2, 2);
  H.vertex(2, 2);
  H.vertex(2, 1);
  H.setPosition(edgeTopLeftX+5, edgeTopLeftY+2);
  H.setStaticBody(true);
  H.setSensor(true);
  H.setFill(150,150,0);
  world.add(H);
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.3)); 
  s.h_avatar.setDensity(5);
  s.h_avatar.setFill(255,0,0); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  

  /* World conditions setup */
  world.setGravity((0.0), (000.0)); //1000 cm/(s^2). 0 = No gravity
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4); // Edge bounce
  world.setEdgesFriction(0.5); //Friction of edges
 
  /*Sound Setup*/
  sineWaves = new SinOsc[numSines]; // Initialize the oscillators
  sineFreq = new float[numSines]; // Initialize array for Frequencies

  for (int i = 0; i < numSines; i++) 
  {
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
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}



/* draw section ********************************************************************************************************/
void draw()
{
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
   
   /*Background refresh*/
   if (refresh == true)
   {
      background(Background);   
   }

  
  /*Sound*/
    //Map AvatarY from 0 to 1
  float yoffset = map(s.h_avatar.getY(), 0, worldHeight, 1, 0); // High pitch = top, low pitch = bottom
  //Map avatarY logarithmically to create a base frequency range
  float frequency = pow(1000, yoffset) + 350;
  //Use avatarX mapped from -0.5 to 0.5 as a detune argument
  float detune = map(s.h_avatar.getX(), 0, worldWidth, -0.5, 0.5);

  for (int i = 0; i < numSines; i++) 
  { 
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

/* Letter Setup */
public void keyPressed() 
{
  if (key == 'a') 
  {
     image(letter_a, 300, 0, 400, 400);
  }
  if (key == 'b') 
  {
     image(letter_b, 300, 0, 400, 400);
  }
  if (key == 'c') 
  {
     image(letter_c, 300, 0, 400, 400);
  }
  if (key == 'd') 
  {
     image(letter_d, 300, 0, 400, 400);
  }
  if (key == 'h') 
  {
     image(letter_h, 300, 0, 400, 400);
  }
  if (key == 'v') 
  {
     image(letter_v, 300, 0, 400, 400);
  }
  if (key == '0') 
  {
     image(letter_Alphabet, 100, 0, 850, 400);
  }
  if (key == '1') 
  {
     image(letter_Join, 100, 0, 800, 400);
  }
  if (key == '9') 
  {
     image(letter_is, 300, 0, 400, 400);
  }
  if (key == '8') 
  {
     image(letter_bp, 300, 0, 400, 400);
  }
  if (key == 'H') 
  {
     world.add(H);
  }
  if (key == 'T') 
  {
     world.add(T);
  }
  if (key == ']') 
  {
     world.remove(H);
     world.remove(T);
  }
  if(key == '-'){ //Clears Screen
    fill(255);
    rect(0,0,1000,400); 
  }
}

/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable
{
  
  public void run()
  {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    rendering_force = true;
    
    if(haplyBoard.data_available())
    {
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      pos_ee.set(widgetOne.get_device_position(angles.array()));
      pos_ee.set(pos_ee.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+2, edgeTopLeftY+(pos_ee).y-7); 
    s.updateCouplingForce();
    f_ee.set(-s.getVCforceX(), s.getVCforceY());
    f_ee.div(60000); // Increase if stability becomes an issure with end effector
    
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();
  
    gA.setPosition(s.h_avatar.getX(), s.h_avatar.getY()); //Position Grabbable Area over Haply Avatar
    
    joint_Formation();
    
    world.step(1.0f/1000.0f);
  
    rendering_force = false;
    
  }
}
/* end simulation section **********************************************************************************************/



/* Joint Creation */

void joint_Formation()
{
  
  boolean touchingCircle = false;
  boolean spaceBarPressed = false;
  
  touching = gA.getContacts(); // If touching GrabArea
  for(int i = 0; i < touching.size(); i++)
  {
    if((touching.get(i).getBody1() != s.h_tool) && (touching.get(i).getBody1().getName() == "Guide"))
    {
      touchingCircle = true;
    }
  }
  
   if (keyPressed)
   {
    if(key == ' ') // if spacebar is pressed boolean value buttonpressed = true
    {
      spaceBarPressed = true;
    }
    if(key == '.') // if . is pressed boolean value of refresh = true to refresh screen
    {
    refresh = true; // Screen Refresh  
    }
    else if (!spaceBarPressed)
    {
      refresh = false;
     }
   }
   
  if(Linkable && spaceBarPressed && touchingCircle) // Create link when space is pressed and touching ciricle
  {
    Linkable = false;
    for(int i = 0; i < touching.size(); i++)
    {
      if((touching.get(i).getBody1() != s.h_tool) && (touching.get(i).getBody1().getName() == "Guide"))
      {
        if(!Created_Join)
        {
          join = new FDistanceJoint(s.h_avatar, touching.get(i).getBody1());
          join.setDamping(1); // set damping of FDistanceJoint
          join.setFrequency(110); // set frequency of FDistanceJoint
          join.setLength(1); // // set Length of FDistanceJoint. Want a close connecion therefore 1 is used
          world.add(join); // Add joint to world
          Created_Join = true;
          
        }
      }
    }
  }
  else if(spaceBarPressed && !touchingCircle) //Remove join when the conditions are not met 
  {
    Linkable = false;
  }
  else if(!spaceBarPressed && touchingCircle) //Remove join when the conditions are not met 
  {
    Linkable = true;
    if(Created_Join)
    {
      world.remove(join);
      Created_Join = false;
    }
  }
  else if(!spaceBarPressed && !touchingCircle) //Remove join when the conditions are not met
  {
    Linkable = true;
    if(Created_Join)
    {
      world.remove(join);
      Created_Join = false;
    }
  }


/*Friction upon FBody*/
//Sets friction to avatar for unguided exploration
  if (s.h_avatar.isTouchingBody(T) || s.h_avatar.isTouchingBody(H))
  {
   s.h_avatar.setDamping(800);
  }
 
  else
  {
    s.h_avatar.setDamping(50);
  }

}
/* end helper functions section ****************************************************************************************/


/*Unused Code***********************************************************************************************************/
