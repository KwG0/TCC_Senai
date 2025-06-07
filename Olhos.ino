#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define sensor A2
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 


#define OLED_RESET     -1 
#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int agua =0;

int demo_mode = 1;
static const int max_animation_index = 8;
int current_animation_index = 0;




int ref_eye_height = 40;
int ref_eye_width = 40;
int ref_space_between_eye = 10;
int ref_corner_radius = 10;

int left_eye_height = ref_eye_height;
int left_eye_width = ref_eye_width;
int left_eye_x = 32;
int left_eye_y = 32;
int right_eye_x = 32+ref_eye_width+ref_space_between_eye;
int right_eye_y = 32;
int right_eye_height = ref_eye_height;
int right_eye_width = ref_eye_width;




void draw_eyes(bool update=true)
{
    display.clearDisplay();        
    
    int x = int(left_eye_x-left_eye_width/2);
    int y = int(left_eye_y-left_eye_height/2);
    display.fillRoundRect(x,y,left_eye_width,left_eye_height,ref_corner_radius,SSD1306_WHITE);
    x = int(right_eye_x-right_eye_width/2);
    y = int(right_eye_y-right_eye_height/2);
    display.fillRoundRect(x,y,right_eye_width,right_eye_height,ref_corner_radius,SSD1306_WHITE);    
    if(update)
    {
      display.display();
    }
    
}


void center_eyes(bool update=true)
{
  
  left_eye_height = ref_eye_height;
  left_eye_width = ref_eye_width;
  right_eye_height = ref_eye_height;
  right_eye_width = ref_eye_width;
  
  left_eye_x = SCREEN_WIDTH/2-ref_eye_width/2-ref_space_between_eye/2;
  left_eye_y = SCREEN_HEIGHT/2;
  right_eye_x = SCREEN_WIDTH/2+ref_eye_width/2+ref_space_between_eye/2;
  right_eye_y = SCREEN_HEIGHT/2;
  
  draw_eyes(update);
}

void blink(int speed=12)
{
  draw_eyes();
  
  
  for(int i=0;i<3;i++)
  {
    left_eye_height = left_eye_height-speed;
    right_eye_height = right_eye_height-speed;    
    draw_eyes();
    delay(1);
  }
  for(int i=0;i<3;i++)
  {
    left_eye_height = left_eye_height+speed;
    right_eye_height = right_eye_height+speed;
    
    draw_eyes();
    delay(1);
  }
}


void sleep()
{
  left_eye_height = 2;
  right_eye_height = 2;
  draw_eyes(true);  
}
void wakeup()
{
  
  sleep();
  
  for(int h=0; h <= ref_eye_height; h+=2)
  {
    left_eye_height = h;
    right_eye_height = h;
    draw_eyes(true);
  }
  

}


void happy_eye()
{
  center_eyes(false);
  
  int offset = ref_eye_height/2;
  for(int i=0;i<10;i++)
  {
    display.fillTriangle(left_eye_x-left_eye_width/2-1, left_eye_y+offset, left_eye_x+left_eye_width/2+1, left_eye_y+5+offset, left_eye_x-left_eye_width/2-1,left_eye_y+left_eye_height+offset,SSD1306_BLACK);
    

    display.fillTriangle(right_eye_x+right_eye_width/2+1, right_eye_y+offset, right_eye_x-left_eye_width/2-1, right_eye_y+5+offset, right_eye_x+right_eye_width/2+1,right_eye_y+right_eye_height+offset,SSD1306_BLACK);
     
    offset -= 2;
    display.display();
    delay(1);
  }
  
  
  display.display();
  delay(1000);
}

void saccade(int direction_x, int direction_y)
{
  
  
  int direction_x_movement_amplitude = 8;
  int direction_y_movement_amplitude = 6;
  int blink_amplitude = 8;

  for(int i=0;i<1;i++)
  {
    left_eye_x+=direction_x_movement_amplitude*direction_x;
    right_eye_x+=direction_x_movement_amplitude*direction_x;    
    left_eye_y+=direction_y_movement_amplitude*direction_y;
    right_eye_y+=direction_y_movement_amplitude*direction_y;    
    
    right_eye_height-=blink_amplitude;
    left_eye_height-=blink_amplitude;
    draw_eyes();
    delay(1);
  }
  
  for(int i=0;i<1;i++)
  {
    left_eye_x+=direction_x_movement_amplitude*direction_x;
    right_eye_x+=direction_x_movement_amplitude*direction_x;    
    left_eye_y+=direction_y_movement_amplitude*direction_y;
    right_eye_y+=direction_y_movement_amplitude*direction_y;

    right_eye_height+=blink_amplitude;
    left_eye_height+=blink_amplitude;

    draw_eyes();
    delay(1);
  }
  
}

void move_right_big_eye()
{
  move_big_eye(1);
}
void move_left_big_eye()
{
  move_big_eye(-1);
}
void move_big_eye(int direction)
{
  

  int direction_oversize = 1;
  int direction_movement_amplitude = 2;
  int blink_amplitude = 5;

  for(int i=0;i<3;i++)
  {
    left_eye_x+=direction_movement_amplitude*direction;
    right_eye_x+=direction_movement_amplitude*direction;    
    right_eye_height-=blink_amplitude;
    left_eye_height-=blink_amplitude;
    if(direction>0)
    {
      right_eye_height+=direction_oversize;
      right_eye_width+=direction_oversize;
    }else
    {
      left_eye_height+=direction_oversize;
      left_eye_width+=direction_oversize;
    }

    draw_eyes();
    delay(1);
  }
  for(int i=0;i<3;i++)
  {
    left_eye_x+=direction_movement_amplitude*direction;
    right_eye_x+=direction_movement_amplitude*direction;
    right_eye_height+=blink_amplitude;
    left_eye_height+=blink_amplitude;
    if(direction>0)
    {
      right_eye_height+=direction_oversize;
      right_eye_width+=direction_oversize;
    }else
    {
      left_eye_height+=direction_oversize;
      left_eye_width+=direction_oversize;
    }
    draw_eyes();
    delay(1);
  }

  delay(1000);

  for(int i=0;i<3;i++)
  {
    left_eye_x-=direction_movement_amplitude*direction;
    right_eye_x-=direction_movement_amplitude*direction;    
    right_eye_height-=blink_amplitude;
    left_eye_height-=blink_amplitude;
    if(direction>0)
    {
      right_eye_height-=direction_oversize;
      right_eye_width-=direction_oversize;
    }else
    {
      left_eye_height-=direction_oversize;
      left_eye_width-=direction_oversize;
    }
    draw_eyes();
    delay(1);
  }
  for(int i=0;i<3;i++)
  {
    left_eye_x-=direction_movement_amplitude*direction;
    right_eye_x-=direction_movement_amplitude*direction;    
    right_eye_height+=blink_amplitude;
    left_eye_height+=blink_amplitude;
    if(direction>0)
    {
      right_eye_height-=direction_oversize;
      right_eye_width-=direction_oversize;
    }else
    {
      left_eye_height-=direction_oversize;
      left_eye_width-=direction_oversize;
    }
    draw_eyes();
    delay(1);
  }


  center_eyes();


}

void setup() {
  
  
  pinMode(sensor, INPUT);
  

  
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
   
  Serial.begin(115200);
  
  

  
  display.clearDisplay();
  
  display.setTextSize(1);             
  display.setTextColor(SSD1306_WHITE);        
  display.setCursor(0,0);             
  display.println(F("U.A.P.P"));  
  display.display();  
  delay(2000);
  sleep();
  delay(2000);

 
  
  
}



void launch_animation_with_index(int animation_index)
{

  

  if(animation_index>max_animation_index)
  {
    animation_index=8;
  }

  switch(animation_index)
      {
        case 0:
          wakeup();
          break;
        case 1:
          center_eyes(true);
          break;
        case 2:
          move_right_big_eye();
          break;
        case 3:
          move_left_big_eye();
          break;
        case 4:      
          blink(10);
          break;
        case 5:
          blink(20);
          break;
        case 6:
          happy_eye();      
          break;
        case 7:
          sleep();
          break;
        
          
      }
}



void loop() {
 
 agua = analogRead (sensor);

Serial.print(" agua: ");
Serial.println(agua);

 delay(300);

   if(agua < 200)
   {
    
 display.clearDisplay();
  
  display.setTextSize(2);             
  display.setTextColor(SSD1306_WHITE);        
  display.setCursor(12,12);             
  display.println(F("sem agua"));  
  display.display();  
  delay(10000);
  sleep();
  delay(100);
  


}

  if(demo_mode == 1)
  {
    // cycle animations
    launch_animation_with_index(current_animation_index++);
    if(current_animation_index > max_animation_index)
    {
      current_animation_index = 0;
    }
  }

  

  if(Serial.available()) {
    String data = Serial.readString();
    data.trim();
    char cmd = data[0];
    
    
    
    
    if(cmd == 'A')
    {
      demo_mode = 0;

      String arg = data.substring(1,data.length());
      int anim = arg.toInt();
      launch_animation_with_index(anim);
      Serial.print(cmd);
      Serial.print(arg);   
    }

 
}
}