/*
PiE Picky Stepper Controller Firmware v0.01
Accepts comma separated motor delays followed by a newline, in the syntax
> x_delay, y_delay, z_delay, t_dinner \n

For example:
> 1000, 1000, 1000, 1000 \n
commands each stepper motor to drive forward, with 1000 microsecond delays
between ticks.
*/

String inString = "";
byte current_channel = 0;
boolean sign = true;
int x_cmd, y_cmd, z_cmd, t_cmd = 0;

int x_interval = 0;
boolean x_ledstate = LOW;
boolean x_dirstate = LOW;
boolean x_last_pos = true;
long x_prev_time = 0;

int y_interval = 0;
boolean y_ledstate = LOW;
boolean y_dirstate = LOW;
boolean y_last_pos = true;
long y_prev_time = 0;

int z_interval = 0;
boolean z_ledstate = LOW;
boolean z_dirstate = LOW;
boolean z_last_pos = true;
long z_prev_time = 0;

int t_interval = 0;
boolean t_ledstate = LOW;
boolean t_dirstate = LOW;
boolean t_last_pos = true;
long t_prev_time = 0;

#define x_step 2
#define x_dir 5

#define y_step 3
#define y_dir 6

#define z_step 4
#define z_dir 7

#define t_step 12
#define t_dir 13

#define enable 8


void setup() {
  Serial.begin(115200);
  
  pinMode(x_step, OUTPUT);
  pinMode(x_dir, OUTPUT);
  
  pinMode(y_step, OUTPUT);
  pinMode(y_dir, OUTPUT);
  
  pinMode(z_step, OUTPUT);
  pinMode(z_dir, OUTPUT);
  
  pinMode(t_step, OUTPUT);
  pinMode(t_dir, OUTPUT);
  
  pinMode(enable, OUTPUT);
  digitalWrite(enable, LOW);
}

void loop() {
  unsigned long currentMillis = micros();
  
  if (x_interval != 0) {
    if(currentMillis - x_prev_time > x_interval) {
      x_prev_time = currentMillis;   
      if (x_ledstate == LOW)
        x_ledstate = HIGH;
      else
        x_ledstate = LOW;
      digitalWrite(x_step, x_ledstate);
    }
  }
  
  if (y_interval != 0) {
    if(currentMillis - y_prev_time > y_interval) {
      y_prev_time = currentMillis;   
      if (y_ledstate == LOW)
        y_ledstate = HIGH;
      else
        y_ledstate = LOW;
      digitalWrite(y_step, y_ledstate);
    }
  }
  
  if (z_interval != 0) {
    if(currentMillis - z_prev_time > z_interval) {
      z_prev_time = currentMillis;   
      if (z_ledstate == LOW)
        z_ledstate = HIGH;
      else
        z_ledstate = LOW;
      digitalWrite(z_step, z_ledstate);
    }
  }
  
  if (t_interval != 0) {
    if(currentMillis - t_prev_time > t_interval) {
      t_prev_time = currentMillis;
      t_ledstate = !t_ledstate;
      digitalWrite(t_step, t_ledstate);
    }
  }
  
  int inChar;
  if (Serial.available() > 0) {
    inChar = Serial.read();
    if (inChar == '-') {
      sign = false;
    }
  }

  if (isDigit(inChar)) {
    inString += (char)inChar; 
  }
  
  if (inChar == ',') {
    switch (current_channel) {
    case 0:    // 0 = x_cmd
      x_cmd = inString.toInt();
      x_interval = x_cmd/2;
      if (sign != x_last_pos) {
        x_dirstate = !x_dirstate;
        digitalWrite(x_dir, x_dirstate);
        x_last_pos = sign;
      }
      break;
    case 1:    // 1 = y_cmd:
      y_cmd = inString.toInt();
      y_interval = y_cmd/2;
      if (sign != y_last_pos) {
        y_dirstate = !y_dirstate;
        digitalWrite(y_dir, y_dirstate);
        y_last_pos = sign;
      }
      break;
    case 2:   // 2 = z_cmd:
      z_cmd = inString.toInt();
      z_interval = z_cmd/2;
      if (sign != z_last_pos) {
        z_dirstate = !z_dirstate;
        digitalWrite(z_dir, z_dirstate);
        z_last_pos = sign;
      }
      break;
    }
    sign = true;
    inString = "";
    current_channel++;
  }
  
  if (inChar == '\n') {
    t_cmd = inString.toInt();
    if (sign != t_last_pos) {
      t_dirstate = !t_dirstate;
      digitalWrite(t_dir, t_dirstate);
      t_last_pos = sign;
      sign = true;
    }
    t_interval = t_cmd/2;
    
    /*
    Serial.print(", x_cmd: ");
    Serial.print(x_cmd);
    Serial.print(", y_cmd: ");
    Serial.print(y_cmd);
    Serial.print(", z_cmd: ");
    Serial.print(z_cmd);
    Serial.print(", t_cmd: ");
    Serial.println(t_cmd);
    */
    

    inString = ""; 
    current_channel = 0;
  }
  
  

}









