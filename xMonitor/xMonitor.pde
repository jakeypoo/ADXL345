//ONLY BT_DAEMON NEEDS TO BE RUNNING FOR THIS TO WORK

import processing.serial.*;
import oscP5.*;

Serial myPort;
OscP5 oscP5;

Runnable runnable = new SerialThread();
Thread thread = new Thread(runnable);



//======= SAVING DATA STUFF =========
boolean save_file = false;

String timestamp = String.valueOf(day()) + '-' + String.valueOf(month()) + '-' + String.valueOf(year()) + '@' + String.valueOf(hour()) + String.valueOf(minute()) + ".txt";
String filename = timestamp; //if you want to call your file something else, name it here

PrintWriter outputter;

int sampleRate=0, ch1=0, ch2=0, ch3=0;
int[] heart_buffer = new int[1];
int[] touch_buffer = new int[1];
int[] movement_buffer = new int[1];
int[] raw_buffer = new int[1];

MiniGraph ch1_graph;
MiniGraph ch2_graph;
MiniGraph ch3_graph;
MiniGraph raw_graph;

int CHUNK_SIZE = 42;  //need to balance this against FPS
//                    FPS*CHUNK_SIZE ~= 512
//  ^dont push the fps too high and adjuct chunk to try to get constant frame rate
float ANALOG_SCALE = 0.003; //scales the ADC graphs
float RAW_SCALE = 0.001; //scales the RAW graph
boolean print_en = true; //set to 'false' to turn off text
int adxl_mag;

void setup(){
  
  if(save_file){
    println ("SAVING TO FILE : " + filename);
    outputter = createWriter(filename);
  }
  size(900,810);
  
  raw_buffer[0] = 0;
  
   /* start oscP5, listening for incoming messages at port 51040 */
  oscP5 = new OscP5(this,7776);
  
 // --------- Serial Prt initialization --------------------------
  String portName = Serial.list()[0]; //change the index if you're having problems connecting to Arduino
  println("Serial available:");
  println(Serial.list());
  print("Running on : ");
  println(Serial.list()[0]);
  myPort = new Serial(this, portName, 19200);
  
  thread.start();
  
 //----------- Graphs Sizes (x,y,width,height) ------------------- 
  ch1_graph= new MiniGraph(5,5,width-10,190);
  ch2_graph= new MiniGraph(5,205,width-10,190);
  ch3_graph= new MiniGraph(5,405,width-10,190);
  raw_graph= new MiniGraph(5,600,width-10,95);
  ch1_graph.fgColor = color(255,200,200);
  ch2_graph.fgColor = color(255,255,255);
  ch3_graph.fgColor = color(100,255,200);
  raw_graph.fgColor = color(200,50,200);
  ch1_graph.fgWeight = 3;
  ch2_graph.fgWeight = 2;
  ch3_graph.fgWeight = 2;
  raw_graph.fgWeight = 3;
  ch1_graph.label = "<3 HEART RATE <3";
  ch2_graph.label = "TOUCH";
  ch3_graph.label = "MOVEMENT";
  raw_graph.label = "BRAINWAVE";
}

void draw() {
   String[] fileout = new String[CHUNK_SIZE];
   background(0x000000);
   int add_length, bt_length;
   
      //---------RETRIEVE FROM OSC/RAW BUFFER-------- 
//  for(int i = CHUNK_SIZE; i > 0; i--) {
//    raw_graph.addValue(raw_buffer[0]*RAW_SCALE);
//    fileout[CHUNK_SIZE-i] += raw_buffer[0];
//    raw_buffer = subset(raw_buffer,1);
//   }
//   if(raw_buffer.length > 50){
//     raw_buffer = subset(raw_buffer, raw_buffer.length-2); //if there is more than 0.5 secs delay, dump the buffer
//   } 
   
   //---------ADD DATA TO GRAPHS-----------
  add_length = (movement_buffer.length)-2;
  bt_length  = (raw_buffer.length)-1;
  
  if(bt_length < 1) bt_length = 0;
  if(add_length < 1) add_length = 0;
 
 
  for(int a = 0; a< add_length; a++){
     ch1_graph.addValue(heart_buffer[a]*0.75);  //heart rate 
     ch2_graph.addValue(touch_buffer[a]*.004);  //pressure strip
     ch3_graph.addValue(movement_buffer[a] * .001);
   }
   
   heart_buffer = subset(heart_buffer,add_length);
   touch_buffer = subset(touch_buffer,add_length);
   movement_buffer = subset(movement_buffer,add_length);
   
   println(add_length);
   
  //-------- PRINT SOME WORDS -----------
//  if(save_file) {
//   for(int i = 0; i < CHUNK_SIZE; i++){outputter.println(fileout[i]);}
//  }

 // ------- DRAW GRAPH ------------
 ch1_graph.render();
 ch2_graph.render();
 ch3_graph.render();
 raw_graph.render();
   
}

class SerialThread implements Runnable {
  
  public void run(){
   char[] packet = new char[6];
   while(true){
   // ------- READ SERIAL DATA --------- 
     packet[0] = 0;
     while(myPort.available() < 8) continue; //if you try to read an empty serial buffer monsters will come and eat you   
     while(packet[0] != '\n') packet[0] = myPort.readChar();
     
     for(int k = 0; k<6; k++){
         packet[k] = myPort.readChar();
     }
     for(int m = 0; m<2; m++){
       heart_buffer = append(heart_buffer, packet[0]);  
      }
     for(int m = 0; m<2; m++){
       touch_buffer = append(touch_buffer, packet[1]);
     }
     
     adxl_mag = ((packet[5])|(packet[4]<<8)); 
     adxl_mag = int(sqrt(float(adxl_mag)) * 7.5) -1000;
     
     for(int m = 0; m<2; m++){
        movement_buffer = append(movement_buffer, adxl_mag); 
     }
  //        fileout[CHUNK_SIZE-(2*i)] = String.valueOf(int(packet[0])) + '\t' + String.valueOf(int(packet[1])) + '\t' + String.valueOf(adxl_mag) + '\t';
 //         fileout[CHUNK_SIZE-(2*i)+1] = String.valueOf(int(packet[0])) + '\t' + String.valueOf(int(packet[1])) + '\t' + String.valueOf(adxl_mag) + '\t';
   }
  }

}

void oscEvent(OscMessage msg) {
  /* print the address pattern and the typetag of the received OscMessage */
  
    if(msg.checkAddrPattern("/mindset/raw")==true) {
    int val = msg.get(0).intValue();
    raw_graph.addValue(val*RAW_SCALE);    
    //if(print_e n) println(" raw: " + val + " " + val*RAW_SCALE);
    raw_buffer = append(raw_buffer, val);
  } 

}
