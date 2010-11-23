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

int serial_dump_count = 0;
int[] heart_buffer = new int[1];
int[] touch_buffer = new int[1];
byte[] x_buffer = new byte[1];
byte[] y_buffer = new byte[1];
byte[] z_buffer = new byte[1];
int[] raw_buffer = new int[1];

MiniGraph ch1_graph;
MiniGraph ch2_graph;
MiniGraph ch3_graph;
MiniGraph ch4_graph;
MiniGraph ch5_graph;
MiniGraph raw_graph;



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
  myPort = new Serial(this, portName, 57600);
  
  
 //----------- Graphs Sizes (x,y,width,height) ------------------- 
  ch1_graph= new MiniGraph(5,5,width-10,190);
  ch2_graph= new MiniGraph(5,205,width-10,190);
  ch3_graph= new MiniGraph(5,355,width-10,190);
  ch4_graph= new MiniGraph(5,355,width-10,190);
  ch5_graph= new MiniGraph(5,355,width-10,190);
  raw_graph= new MiniGraph(5,600,width-10,95);
  ch1_graph.fgColor = color(255,200,200);
  ch2_graph.fgColor = color(255,255,255);
  ch3_graph.fgColor = color(100,255,200);
  ch4_graph.fgColor = color(255,51,204);
  ch5_graph.fgColor = color(51,204,255);
  raw_graph.fgColor = color(200,50,200);
  ch1_graph.fgWeight = 3;
  ch2_graph.fgWeight = 2;
  ch3_graph.fgWeight = 2;
  ch4_graph.fgWeight = 2;
  ch5_graph.fgWeight = 2;
  raw_graph.fgWeight = 3;
  ch1_graph.label = "<3 HEART RATE <3";
  ch2_graph.label = "TOUCH";
  ch3_graph.label = "MOVEMENT";
  raw_graph.label = "BRAINWAVE";

  myPort.clear();
  thread.start();
}

void draw() {
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
 
  while(serial_dump_count >0) continue; 

  add_length = (z_buffer.length)-2;
  bt_length  = (raw_buffer.length)-1;
  
  if(bt_length < 1) bt_length = 0;
  if(add_length < 1) add_length = 0;
 
  for(int a = 0; a< add_length; a++){
     ch1_graph.addValue(heart_buffer[a]*0.75);  //heart rate 
     ch2_graph.addValue(touch_buffer[a]*.01);  //pressure strip
     ch3_graph.addValue(x_buffer[a] * .05);
     ch4_graph.addValue(y_buffer[a] * .05);
     ch5_graph.addValue(z_buffer[a] * .05);
   }
  
  serial_dump_count=add_length;  
     
   println(add_length+ "  " + frameRate);
   
  //-------- PRINT SOME WORDS -----------
//  if(save_file) {
//   for(int i = 0; i < CHUNK_SIZE; i++){outputter.println(fileout[i]);}
//  }

 // ------- DRAW GRAPH ------------
 ch1_graph.render();
 ch2_graph.render();
 ch3_graph.render();
 ch3_graph.render();
 ch4_graph.render();
 ch5_graph.render();
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
     
     for(int k = 0; k<4; k++){
         packet[k] = myPort.readChar();
     }
     for(int m = 0; m<1; m++){
       heart_buffer = append(heart_buffer, packet[0]>>7);  
      }
     for(int m = 0; m<1; m++){
       touch_buffer = append(touch_buffer, packet[0]&0x7F);
     }
     
     for(int m = 0; m<1; m++){
        x_buffer = append(x_buffer,byte(packet[1])); 
     }

     for(int m = 0; m<1; m++){
        y_buffer = append(y_buffer,byte(packet[2])); 
     }


     for(int m = 0; m<1; m++){
        z_buffer = append(z_buffer,byte(packet[3])); 
     }


     if(serial_dump_count >0) 
     {
       heart_buffer = subset(heart_buffer, serial_dump_count);
       x_buffer = subset(x_buffer, serial_dump_count);
       y_buffer = subset(y_buffer, serial_dump_count);
       z_buffer = subset(z_buffer, serial_dump_count);
       touch_buffer = subset(touch_buffer, serial_dump_count);
       serial_dump_count=0;
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
