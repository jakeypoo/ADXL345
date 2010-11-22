// This class represents one of the mini graphs on the left
class MiniGraph {

  int x;
  int y;
  int w;
  int h;

  float[] history;
  int index;

  boolean drawRect = false;
  boolean fillCurve = false;
  boolean drawLevel = false;
  boolean drawThreshold = false;
  
  float threshold = 0;

  color fgColor = color(255);
  int fgWeight = 1;
  color mdColor = color(255, 64);
  color bgColor = color(60);
  color labelColor = color(200);

  String label = "";
  int label_size = 12;
  PFont font;

  MiniGraph(int x, int y, int w, int h) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;

    history = new float[w];
    
    font = createFont("Arial", label_size);
  }

  // newValue -> 0..1
  void addValue(float newValue) {
    history[index] = newValue;
    index = (index+1) % history.length;    
  }

  void render() {
    int c = prevValue(index);
    if ( c < 0 ) c = history.length - 1;
    for (int i=history.length-1; i >= 0; i--) {      

      int x1 = x + i;
      int y1 = int( (y+h) - h * history[c] );      
      c = prevValue(c);

      if (fillCurve) {
        stroke(mdColor);
        line(x1, y1, x1, (y+h));  
      }

      if (drawLevel && i == history.length -1) {
        // current value
        noStroke();
        fill(bgColor);
        rect(x, y1, w, (y+h) - y1);
      }

      stroke(fgColor);
      strokeWeight(fgWeight);
      if (i == 0) {
        point(x1, y1);
      }
      else {
        int x2 = x + i-1;
        int y2 = int ( (y+h) - h * history[c] );                
        line(x1, y1, x2, y2);
      }
    }
    if (drawRect) {
      noFill();
      stroke(fgColor);
      rect(x, y, w, h);  
    }
    
    if (label != "") {
      fill(labelColor);
      textFont(font);
      text( label, x, y+h+label_size+2);  
    }
    
    if (drawThreshold) {
      stroke(100, 255, 255, 128);
      line(x-1, (1-threshold)*h + y, x+w-1, (1-threshold)*h + y);  
    }
  }

  int prevValue(int index) {
    int i = index - 1;
    if (i < 0) i = history.length - 1;
    return i;  
  }
}

class GraphRowLayout {
  int x;
  int y;
  int w;
  int h;

  int between_spacing = 10;

  ArrayList graphs;
  int num_graphs;
  int can_hold;

  boolean drawRect = false;
  
  GraphRowLayout(int x, int y, int w, int h, int can_hold) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;    
    this.can_hold = can_hold;
    graphs = new ArrayList();
  }

  void addGraph(MiniGraph new_graph) {
      //println("old lenght: " + graphs.length);
      graphs.add(new_graph);
//      num_graphs++;
      recalcLayout();
  }
  
  void recalcLayout() {
    int x_spacing = w/graphs.size();
    int x_height = x_spacing;//-between_spacing/2;
      
    for(int i=0; i<graphs.size(); i++) {
      MiniGraph cur_graph = (MiniGraph) graphs.get(i);
      cur_graph.x = x_spacing*i+x;
      cur_graph.w = x_height-between_spacing;
      cur_graph.y = y;
      cur_graph.h = h;
      // We resize the history of the graph to match the new width
      cur_graph.history = new float[cur_graph.w];
    }  
  }
  
  void render() {
    if (drawRect) {
        noFill();
        stroke(255);
        rect(x, y, w, h);  
    }
    for(int i=0; i<graphs.size(); i++) {
      MiniGraph cur_graph = (MiniGraph) graphs.get(i);
      cur_graph.render();  
    }
  }
}  

class GraphColumnLayout {
  int x;
  int y;
  int w;
  int h;

  int between_spacing = 10;

  ArrayList graphs;
  int num_graphs;
  int can_hold;

  boolean drawRect = false;
  
  GraphColumnLayout(int x, int y, int w, int h, int can_hold) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;    
    this.can_hold = can_hold;
    graphs = new ArrayList();
  }

  void addGraph(MiniGraph new_graph) {
      //println("old lenght: " + graphs.length);
      graphs.add(new_graph);
//      num_graphs++;
      recalcLayout();
  }
  
  void recalcLayout() {
    int y_spacing = h/graphs.size();
    int y_height = y_spacing;//-between_spacing/2;
      
    for(int i=0; i<graphs.size(); i++) {
      MiniGraph cur_graph = (MiniGraph) graphs.get(i);
      cur_graph.x = x;
      cur_graph.w = w;
      cur_graph.y = y_spacing*i+y;
      cur_graph.h = y_height-between_spacing;
      // We resize the history of the graph to match the new width
      cur_graph.history = new float[cur_graph.w];
    }  
  }
  
  void render() {
    if (drawRect) {
        noFill();
        stroke(255);
        rect(x, y, w, h);  
    }
    for(int i=0; i<graphs.size(); i++) {
      MiniGraph cur_graph = (MiniGraph) graphs.get(i);
      cur_graph.render();  
    }
  }
}
