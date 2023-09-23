
#include "simpletools.h"            
#include "servo.h"

//DECLARING VARIABLES, PINS OF COMPONENTS ON PROPELLER

static int servo_right_pin = 16; 
static int servo_left_pin = 17; 
static int ir_left_pin = 1; 
static int ir_right_pin = 3;

static int center_path = 1; 
static int red_light_obstacle = 1;
static int intersection = 0;
static int intersection_lane_a = 1;
static int intersection_atob = 1;
static int end = 0; 

static volatile int intersection_detection = 0;  
static volatile int pickup_indicator = 0; 
static volatile int drop_indicator = 0; 



int distance_counter = 0;
static int pappli = 0;
static int papplia = 0;


serial *lcd;

static int pickup_lane = 0; 
static int drop_lane = 0; 


static int straight_speed_rightservo = -100 ;
static int straight_speed_leftservo =  100; 

static int straight_speed_laneab_rightservo = -25; 
static int straight_speed_laneab_leftservo = 25; 

static int right_turn_rightservo = 50;
static int right_turn_leftservo = 50;

static int left_turn_rightservo = -50;
static int left_turn_leftservo = -50;


static volatile int counter;
static volatile int pickup_counter = 0;

static volatile int currentstateir; 
static volatile int laststateir;


static int trig_front = 8; 
static int echo_front = 9;
static int trig_left = 6;
static int echo_left = 5;
static int final_counter = 0; 


static volatile int left_ir;
static volatile int right_ir;
static volatile int last_left_ir;
static volatile int last_right_ir;

static volatile float distance = 100;
static volatile float distance_pickupdrop;
static volatile int intermediate_papplia = 0;
static volatile float duration;
static volatile float duration1;
static volatile int obstacle =0 ; 
static volatile int obstacle_center=0; 
static volatile int counting_final_ready = 0; 

static int red_pin = 13;
static int green_pin = 14; 
static int blue_pin = 15; 

const int ON = 22;
const int CLR = 12;
const int L0C0=128;


//COGSTART FUNCTION THAT READS THE STATUS OF THE IR CONSTANTLY
void ir_status_function(void *par)
{
  while(1)
  {
    left_ir = input(ir_left_pin);
    right_ir = input(ir_right_pin);
  }    
} 

////COGSTART FUNCTION THAT IS FOR CONSTANT INDICATION 
void led_control(void *par)
{
  while(1)
  { 
  //FOR INTERSECTION
    if (intersection_detection == 1)
    {
    low(red_pin);
    low(green_pin);
    high(blue_pin);
    pause(300);
    low(red_pin);
    low(green_pin);
    low(blue_pin);
    intersection_detection = 0; 
   }    

   //FOR OBSTACLES
   if (obstacle_center == 1 || distance < 10 )
   {
    printf("Distance print : %f \n", distance);
    high(red_pin);
    low(green_pin);
    low(blue_pin);
    pause(300);
    low(red_pin);
    low(green_pin);
    low(blue_pin);    
    }
      
    //FOR PICKUP
    if(pickup_indicator == 1)
    {
    low(red_pin);
    high(green_pin);
    high(blue_pin);
    pause(2000);
    low(red_pin);
    low(green_pin);
    low(blue_pin); 
    pickup_indicator = 0; 
    }   
    
    //FOR DROP OFF
    if(drop_indicator == 1)
    {
    low(red_pin);
    high(green_pin);
    low(blue_pin);
    pause(500);
    low(red_pin);
    low(green_pin);
    low(blue_pin);
    drop_indicator = 0;    
    }  
    
    //FOR FINAL DROP OFF DISTANCE 
   if(counting_final_ready == 1)
    {
    int k; 
    printf("End Print");
    printf("\n");
    //printf("%d ",distance_counter); 
    for(k = 1 ; k<=pappli; k++)
    {
    high(red_pin);
    low(green_pin);
    high(blue_pin);
    pause(300);
    low(red_pin);
    low(green_pin);
    low(blue_pin);
    pause(300);   
      }
      lcd = serial_open(12, 12, 0, 9600);
      writeChar(lcd, ON);
      writeChar(lcd, CLR);
      writeChar(lcd, 17);

       pause(5);
       writeChar(lcd, L0C0);
       dprint(lcd, "Distance Covered = %d cm",pappli*40); 
       
      break; 
         
     }  
  }
} 



//FUNCTION FOR LEFT TURN 
void left_turn()
{
  //int currentstateir, laststateir;
  currentstateir = left_ir;
  laststateir = currentstateir; 
  
  

  if(currentstateir == 0)
  {
    counter  = 2;
  }
  else
  {
    counter = 3; 
  }
  int i=1;

  while(i<=counter)
  { 
    left_tilt();
    currentstateir = left_ir;
    if(laststateir != currentstateir)
    {
      i++;
      laststateir = currentstateir; 

    }
  }
 
}  

//FUNCTION FOR RIGHT TURN
void right_turn()
{

  currentstateir = right_ir;
  laststateir = currentstateir; 
  
  

  if(currentstateir == 0)
  {
    counter  = 2;
  }
  else
  {
    counter = 3; 
  }
  int i=1;

  while(i<=counter)
  { 
    right_tilt();
    currentstateir = right_ir;
    if(laststateir != currentstateir)
    {
      i++;
      laststateir = currentstateir; 

    }
  }
 
} 


//FUNCTION FOR FRONT OBSTACLE DETECTION
void ultrasonic_status_fn(void *par)
{ 
  float l_cm;
  long duration;
  
  while (1)
  {
    //float l_cm_avg = 0.0;
   
    low(trig_front);
    pulse_out(trig_front,10);  
    long tEcho = pulse_in(echo_front,1);

    l_cm = tEcho/58.0;        
     
    distance = l_cm;
    
    if (distance < 40)
    {
     obstacle = 1;
    }
    else
    {
      obstacle = 0;     
    }             
      
  }    
}

//FUNCTION FOR DETECTING PICKUP AND DROP OFF WIDGETS
void ultrasonic_pickup_drop_fn(void *par)
{
  float l1_cm;
  long duration1;
  
  while (1)
  {
    //float l_cm_avg = 0.0;
   
    low(trig_left);
    pulse_out(trig_left,10);  
    long tEcho1 = pulse_in(echo_left,1);

    l1_cm = tEcho1/58.0;        
     
    distance_pickupdrop = l1_cm;
    pause(1000);
    //CHECK 
    
  }    

  
  
}
  
//FUNCTIONS FOR ROBOT MOTION CONTROL 
void straight()
{
  servo_speed(servo_right_pin, straight_speed_rightservo);
  servo_speed(servo_left_pin,straight_speed_leftservo);
}

void straight_lane_b()
{
  servo_speed(servo_right_pin, straight_speed_laneab_rightservo);
  servo_speed(servo_left_pin, straight_speed_laneab_leftservo);
}

void left_tilt()
{
  servo_speed(servo_right_pin, left_turn_rightservo);
  servo_speed(servo_left_pin, left_turn_leftservo);
}

void right_tilt()
{
  servo_speed(servo_right_pin, right_turn_rightservo);
  servo_speed(servo_left_pin, right_turn_leftservo);
}

void halt()
{
  servo_speed(servo_right_pin,0);
  servo_speed(servo_left_pin, 0); 
}  
    
    
//FUNCTION FOR LINE FOLLOWING 
void motion()
{
  if((left_ir == 0) && (right_ir == 0))
  {
   straight(); 
  }
  
  else if((left_ir == 1) && (right_ir == 0))
  {
    left_tilt();
  } 
  
  else if((left_ir == 0) && (right_ir == 1))
  {
    right_tilt();
  }
  
  else if((left_ir == 1) && (right_ir == 1))
  {

    intersection_detection = 1;
   if (center_path == 1)
    {
      intersection = intersection + 1; 
      
      if(obstacle == 1)
      {
        obstacle_center = 1; 
      }        
      
    }   
    

    straight();
    pause(300);
    
  }           
      
  
}

//FUNCTION FOR LINE FOLLOWING IN LANE B DURING MANEUVER 
void motion_lane_b_maneuver()
{
  if (distance < 10)
  {
    halt(); 
  }    
  
 else if((left_ir == 0) && (right_ir == 0))
  {
   straight_lane_b(); 
  }
  
  else if((left_ir == 1) && (right_ir == 0))
  {
    left_tilt();
  } 
  
  else if((left_ir == 0) && (right_ir == 1))
  {
    right_tilt();
  }
  
  else if((left_ir == 1) && (right_ir == 1))
  { 
    intersection_detection = 1;
    straight_lane_b();
    pause(600);

  }           
      
  
}

 
//FUNCTION FOR WIDGET PICKUP AND LINE FOLLOWING IN LANE A AND B
void motion_lane_a()
{

if(pickup_counter == 0)
{
  if(distance < 10)
  {
    halt();
  }
   else if(distance_pickupdrop < 21)
  {
    pickup_counter = 1;
    if (pickup_lane == 1)
    { 
      pickup_indicator = 1;
      final_counter = 1;
      papplia = 5 - intersection_lane_a;
      printf("Papplia matlab yaha aaya : \n");
      printf("%d \n",papplia);
    }
    else if(drop_lane == 1)
    {
      printf("Intersection count for lane B : ");
      printf("%d \n", intersection_lane_a); 
      drop_indicator = 1; 
      end = 1;
    }
    halt();
    pause(2000);      
  }    
  else if((left_ir == 0) &&(right_ir == 0))
  {
    straight_lane_b();
  }
  else if((left_ir == 1) && (right_ir == 0))
  {
    left_tilt();
  }
  else if((left_ir == 0) && (right_ir == 1))
  {
    right_tilt();
  }
    else if((left_ir == 1) && (right_ir == 1))
  { 
     intersection_detection = 1; 
     intersection_lane_a = intersection_lane_a +1;
    straight_lane_b();
    pause(700);
   }  
  
 } 
 
 else if(pickup_counter == 1)
 {
     if(distance < 10)
  {
    halt();
  }   
  else if((left_ir == 0) &&(right_ir == 0))
  {
    straight_lane_b();
  }
  else if((left_ir == 1) && (right_ir == 0))
  {
    left_tilt();
  }
  else if((left_ir == 0) && (right_ir == 1))
  {
    right_tilt();
  }
    else if((left_ir == 1) && (right_ir == 1))
  { 
     intersection_detection = 1; 
     intersection_lane_a = intersection_lane_a +1;
    straight_lane_b();
    pause(700);
    
   }  
 }   
}  
  


//MEMORY ALLOCATION FOR INDIVIDUAL COGS
unsigned int ir_stack[40+40];
unsigned int ultrasonic_stack[40+40];
unsigned int ultrasonic_pickupdrop_stack[40+40];
unsigned int led_control_stack[40+40]; 
unsigned int distance_measurement_stack[40+40];


//MAIN FUNCTION
int main()                                   
{

  cogstart(&ir_status_function, NULL,ir_stack, sizeof(ir_stack));
  cogstart(&ultrasonic_status_fn, NULL,ultrasonic_stack, sizeof(ultrasonic_stack));  
  cogstart(&ultrasonic_pickup_drop_fn, NULL,ultrasonic_pickupdrop_stack, sizeof(ultrasonic_pickupdrop_stack));  
  cogstart(&led_control, NULL,led_control_stack, sizeof(led_control_stack));  

//CENTER LANE TRAVEL AND OBSTACLE MANEUVERING 
  while(1)
 { 
 
  motion();
  
   //printf("%d", intersection);
   if (intersection>= 5) 
  {
    halt();
    break;
  }    


  if (obstacle_center == 1) 
  {
    obstacle_center = 0;
    center_path = 0;
    left_turn();
    while(1)
    {
     if((left_ir == 1) && (right_ir ==1))
    {
      motion();
      break;
      }
      motion();    
    }
    //motion();
    right_turn();
    while(1)
    {
     if((left_ir == 1) && (right_ir ==1))
    {
      motion_lane_b_maneuver();
      break;
      }
      motion_lane_b_maneuver();
    } 
     
    right_turn();
    //obstacle_center = 0;
    while(1)
    {
      if((left_ir == 1) && (right_ir ==1))
    {
      motion();
      break;
      }
      motion();
    }
    
 
  
    left_turn();
    if (obstacle == 1)
    {
     obstacle_center = 1;
    } 
        
   
    center_path = 1;
    intersection = intersection + 1;   
  }
       
  } //end of while loop   

red_light_obstacle = 0;

//ROBOT AT THE END OF CETER LANE 
//PART FOR LANE A 

obstacle_center = 0;

right_turn();
obstacle_center = 0;

while(1)
  {
   if((left_ir == 1) && (right_ir ==1))
    {
      motion();
      break;
    }
  motion(); 
 }  
right_turn();
red_light_obstacle = 1;
obstacle_center = 0;

pickup_lane = 1; 

while(1)
{
  
  if(intersection_lane_a == 5)
  {
    straight_lane_b();
    pause(600);
    halt();
    break;
  }    
  
  motion_lane_a();
 }
 
intermediate_papplia = papplia;
  
//END OF PART FOR LANE A
 red_light_obstacle = 0; 
  
//ROBOT GOING TO LANE B
 pickup_lane = 0; 
 drop_lane = 1;
 right_turn();
 intersection = 1;
 center_path = 1; 
 while(1)
 {
   motion();
   
   if (intersection >= 3)
   {
     motion();
     //pause(300);
     //halt();
     break;
    }
         
 }  
  
//REACHED LANE B
red_light_obstacle =  1;

//START CHECKING FOR DROP OFF AT LANE B

intersection = 0; 
pickup_counter = 0;
drop_lane = 1; 
pickup_lane =0 ;
intersection_lane_a = 1;

right_turn();

obstacle_center = 0; 
while(1)
{
  motion_lane_a();
  
  if (end == 1)
  {
    pappli = intermediate_papplia +  intersection_lane_a + 1;
    printf("%d pappli\n", pappli);
    printf("%d papplia \n", papplia);
    counting_final_ready = 1;

    halt();
    
    break;
  }    

 }  

 
}
