#include <QTRSensors.h>
#define NUM_SENSORS 8
#define TIMEOUT 2500 //microseconds

QTRSensorsRC my_sensor(() {, , , , , , , },
  NUM_SENSORS, TIMEOUT, );


typedef struct{
    int x,y,point,type,explored;
}maze;
maze track[10];

bool start = false;
int node = 0;
int current_x = 0; 
int current_y = 0;
int j = 0; 
int visited[j];
int visit_length = visit.length()/4;

void setup()
{
    for (int i = 0; i < 400; i++)  // make the calibration 
  {
    qtrrc.calibrate();       
    digitalWrite(LED_BUILTIN, HIGH);     
  }

    digitalWrite(LED_BUILTIN,LOW);
}

void loop()
{
    //when started

    if(start==false && /*straiht line*/){
        track[node].x = 0; track[node].y = 0; track[node].type = 0; track[node].explored = 1; track[node].point = 0;
        start = true;
        digitalWrite(LED_BUILTIN,HIGH);
        //move_forward();
    }

    if(/*any junction*/)
    {
         current_x = current_x + encoder_x_distance; current_y = current_y + encoder_y_distance;

                    if(search(current_x,current_y)==false)
                    {
                        node++;
                        track[node].x = current_x + encoder_x_distance; track[node].y = current_y + encoder_y_distance;
                        track[node].explored = 1; track[node].point = node;

                         switch(/*type of juction*/)
                         {
                            case : /*right point*/
                                track[node].type = 1;
                                //move right
                                break;
                            
                            case : /*left turn*/
                                track[node].type = 1;
                                //move left
                                break;

                            case : /*T point*/
                                track[node].type = 2;
                                //turn right
                                break;

                            case : /*+ point*/
                                track[node].type = 3;
                                //turn right
                                break;
                         }
                    }
                    else
                    {
                        track[node].explored++ ;

                        switch (track[node].explored)
                        {
                        case 2:
                            /*move forward*/
                            break;
                        case 3:
                            /*move left*/
                            break;
                        }

                        if(track[node].explored == track[node].type)
                        {
                            //about turn.
                        }
                        else if(track[node].explored > track[node].type)
                        {
                            actual_run();
                        }
                    }

                    visited[j] = node;
                    j++;
    }
}

int search(int x, int y, int node) 
{ 
    int i; 
    for (i = 0; i <= node; i++) {
        if (track[i].x == current_x && track[i].y == current_y) 
            return true; 

    }
    return false; 
} 

void PIDcontrol(){ 
  int error = position - 1000;  

  int motorSpeed= Kp*error + Kd*(Error-lastError);

  leftMotor = initialSpeed+motorSpeed;
  rightMotor = initalSpeed-motorSpeed;

  lastError = error;

  if(rightMotor<0)
    rightMotor = 0;

  if(leftMotor<0)
    leftMotor = o;  
}

void actual_run()
{
    unsigned int graph[node][node];

    for(int i = 0; i<node ; i++)
    {
        for(int k = 0; k<node ; k++) //forming adjacency matrix
        {
            if(i==k)
            {
                graph[i][k] = 0;
            }

            if(adjacency(i,k) == true)
            {
                graph[i][k] = (track[i].x - track[k].x) + (track[i].y - track[k].y);
                graph[k][i] = (track[i].x - track[k].x) + (track[i].y - track[k].y); //symmtrical matrix;
            }
            else
            {
                graph[i][k] = 0;
                graph[k][i] = 0;
            }
            
        }
    }
}

void adjacency(int i,int k)
{
    for(int d = 1; d<visit_length ; d++)
    {
        if(visit[d] == i &&(visit[d+1] == k || visit[d-1] == k))
        {
            return true;
        }

        else
        {
            return false;
        }
        
    }
}