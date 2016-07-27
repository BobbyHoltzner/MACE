#include <GL/glew.h> // Include the GLEW header file
#include <GL/glut.h> // Include the GLUT header file
#include <iostream>
#include <math.h>
#include <algorithm>
#include <time.h>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <cstring>

using namespace std;

#include "cvt.cpp"
#include "Voronoi.h"
#include "VPoint.h"
#include "Voronoi.cpp"
#include "VParabola.cpp"

void test1();
void test_cvt(int dim_num, int n, int batch, int init, int sample, int sample_num, 
  int it_max, int it_fixed, int *seed, double r[],int *it_num, double *it_diff, 
  double *energy );
void display (void);
void onEF (int n);
void reshape (int width, int height);

vor::Voronoi * v;
vor::Vertices * ver; 
vor::Vertices * ver1;  
vor::Vertices * dir; 
vor::Edges * edg;	
	 

double w = 10000;
int t = 10;
int main (int argc, char **argv) 
{
	using namespace vor;

	v = new Voronoi();
	ver = new Vertices();
	ver1 = new Vertices();
	dir = new Vertices();
	
	srand ( time(NULL) );

	for(int i=0; i<3; i++) 
	{
		
		ver->push_back(new VPoint( w * (double)rand()/(double)RAND_MAX , w * (double)rand()/(double)RAND_MAX )); //initialize generators
		ver1->push_back(new VPoint(0, 0)); //initialize centroids
	
	}
	
	for(int k = 0; k<10; k++)
	{
		dir->push_back(new VPoint(t,t)); 
		t+=10;	
	}

	std::cout << "voronois done!\n";
	for(vor::Vertices::iterator i = ver->begin(); i!= ver->end(); ++i)
	{
		std::cout << (*i)->x << "x\n";
		std::cout << (*i)->y << "y\n";
	}
	
	test1(); //function to generate centroids
	
	
	glutInit(&argc, argv); // Initialize GLUT
	glutInitDisplayMode (GLUT_SINGLE); // Set up a basic display buffer (only single buffered for now)
	glutInitWindowSize (600, 600); // Set the width and height of the window
	glutInitWindowPosition (100, 100); // Set the position of the window
	glutCreateWindow ("Your first OpenGL Window"); // Set the title for the window
	
	glutTimerFunc(50, onEF, 0);
	glutDisplayFunc(display); // Tell GLUT to use the method "display" for rendering


	glutReshapeFunc(reshape); // Tell GLUT to use the method "reshape" for reshaping
	glutMainLoop(); // Enter GLUT's main loop
	
	return 0;
}

void drawVoronoi()
{	
	/*Update the generators with the centroid values as they move closer
	 * to them using formula x1 = x1+ (x2-x1)*t and y1 = y1+ (y2-y1)*t
	 * where (x1, y1) are initial generators, (x2,y2) are centroids
	 * and t is used as an iterator to move towards the centroids.
	 * */
	  vor::Vertices::iterator k = dir->begin(); 
	  vor::Vertices::iterator j = ver1->begin();
	  for(vor::Vertices::iterator i = ver->begin(); i!= ver->end(); ++i)
	  {
	  (*i)->x = (*i)->x + (((*j)->x-(*i)->x)*(*k)->x)/100;
	  (*i)->y = (*i)->y + (((*j)->y-(*i)->y)*(*k)->y)/100;
	  j++;
	  }
	  k++;
	  edg = v->GetEdges(ver, w, w);
	  
	for(vor::Edges::iterator i = edg->begin(); i!= edg->end(); ++i)
	{
		glBegin(GL_LINES);
		glVertex2f( -1+2*(*i)->start->x/w,  -1+2*(*i)->start->y/w);
		glVertex2f( -1+2*(*i)->end->x/w, -1+2*(*i)->end->y/w);
		glEnd();
		
	}
	
	for(vor::Vertices::iterator i = ver->begin(); i!= ver->end(); ++i)
	{
		glBegin(GL_QUADS);
		glColor3f(0,0,1);
		glVertex2f( -1+2*(*i)->x/w -0.01,  -1+2*(*i)->y/w - 0.01);
		glVertex2f( -1+2*(*i)->x/w +0.01,  -1+2*(*i)->y/w - 0.01);
		glVertex2f( -1+2*(*i)->x/w +0.01,  -1+2*(*i)->y/w + 0.01);
		glVertex2f( -1+2*(*i)->x/w -0.01,  -1+2*(*i)->y/w + 0.01);
		glEnd();
	}
	  

}



void test1()
{

# define N 3
# define DIM_NUM 2

  int batch;
  double energy;
  int init;
  char init_string[80];
  double it_diff;
  int it_fixed;
  int it_max;
  int it_num;
  double r[DIM_NUM*N];
  int sample;
  int sample_num;
  char sample_string[80];
  int seed;
  int seed_init;


  batch = 1000;
  init = 0;
  strcpy ( init_string, "uniform" );
  it_max = 10000;
  it_fixed = 1;
  sample = -1;
  sample_num = 10000;
  strcpy ( sample_string, "uniform" );
  seed = 123456789;

  seed_init = seed;

  test_cvt ( DIM_NUM, N, batch, init, sample, sample_num, it_max, it_fixed, 
    &seed, r, &it_num, &it_diff, &energy );

      
  int k = 0;
  for(vor::Vertices::iterator i = ver1->begin(); i != ver1->end(); ++i)
    {
		if(k%2==0)
		{
			(*i)->x = r[2*k];
			(*i)->y = r[2*k+1]; 
		}
		else
		{
			(*i)->x = r[2*k];
			(*i)->y = r[2*k+1]; 
		}
		k++;
	}
	
	
	
  return;

}

void display (void) 
{
	std::cout << "display\n";
	glLoadIdentity(); // Load the Identity Matrix to reset our drawing locations  
	glTranslatef(0.0f, 0.0f, -5.0f); 
	glFlush();
}



void onEF(int n)
{
	
	glutTimerFunc(100, onEF, 0);
	glClear(GL_COLOR_BUFFER_BIT);//Clear the screen
	glClearColor(0.0f, 0.0f, 0.2f, 1.0f); // Clear the background of our window to red  
	
	drawVoronoi();
	glutSwapBuffers();

	//Draw everything to the screen
}

void reshape (int width, int height) 
{
	
	glViewport(0, 0, (GLsizei)width, (GLsizei)height); // Set our viewport to the size of our window
	glMatrixMode(GL_PROJECTION); // Switch to the projection matrix so that we can manipulate how our scene is viewed
	glLoadIdentity(); // Reset the projection matrix to the identity matrix so that we don't get any artifacts (cleaning up)
	gluPerspective(22.5, (GLfloat)width / (GLfloat)height, 1.0, 100.0); // Set the Field of view angle (in degrees), the aspect ratio of our window, and the new and far planes
	glMatrixMode(GL_MODELVIEW); // Switch back to the model view matrix, so that we can start drawing shapes correctly
}

void test_cvt ( int dim_num, int n, int batch, int init, int sample, int sample_num, 
  int it_max, int it_fixed, int *seed, double r[],int *it_num, double *it_diff, 
  double *energy )
{
  
  bool DEBUG = true;
  int i;
  bool initialize;
  int seed_base;
  int seed_init;

  *it_num = 0;
  *it_diff = 0.0;
  *energy = 0.0;
  seed_init = *seed;
//
//  Initialize the data, unless the user has already done that.
//
 
    initialize = true;
    if ( sample == -1 )
	{
    if ( initialize )
    {
      random_initialize ( *seed );
    }
    
    int k = 0;
    for(vor::Vertices::iterator i = ver->begin(); i != ver->end(); ++i)
    {
		if(k%2==0)
		{
			r[2*k] = (*i)->x;
			r[2*k+1] = (*i)->y; 
		}
		else
		{
			r[2*k] = (*i)->x;
			r[2*k+1] = (*i)->y; 
		}
		k++;
    *seed = ( *seed ) + n * dim_num;
    }
   }

  if ( init == sample )
  {
    initialize = false;
  }
  else
  {
    initialize = true;
  }
//
//  Carry out the iteration.
//
  while ( *it_num < it_max )
  {
    if ( ( (*it_num) % it_fixed ) == 0 )
    {
      seed_base = *seed;
    }
    else
    {
      *seed = seed_base;
    }

    *it_num = *it_num + 1;
    seed_init = *seed;
	
    cvt_iterate ( dim_num, n, batch, sample, initialize, sample_num, seed,
      r, it_diff, energy );

    initialize = false;
  }
  return;
}

