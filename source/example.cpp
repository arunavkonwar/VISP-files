#include <iostream>
#include <math.h>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>

#include <string>     // std::string, std::to_string


using namespace std ;



int main()
{
  int i,j;
  vpImage<unsigned char> I1(300,400,0);
  vpImage<unsigned char> I2(300,400,0);
  vpImage<vpRGBa> Iimage(800,1200);
  
 
  vpImageIo::read(Iimage,"../data/hollywood-triangle.jpg") ;
  
// Cette partie ne sert qu'a la simulation
  // on positionne un poster dans le repere Rw

// This part is only for simulation
  // we position a poster in the rep Rw

  double L = 0.600 ;
  double l = 0.400;
  // Initialise the 3D coordinates of the Iimage corners
  vpColVector X[4];
  for (int i = 0; i < 4; i++) X[i].resize(3);
  // Top left corner
  X[0][0] = -L;
  X[0][1] = -l;
  X[0][2] = 0;
  
  // Top right corner
  X[1][0] = L;
  X[1][1] = -l;
  X[1][2] = 0;
  
  // Bottom right corner
  X[2][0] = L;
  X[2][1] = l;
  X[2][2] = 0;
  
  //Bottom left corner
  X[3][0] = -L;
  X[3][1] = l;
  X[3][2] = 0;
  


  vpImageSimulator sim;
  sim.init(Iimage, X);

  // On définit une camera avec certain parametre u0 = 200, v0 = 150; px = py = 800
  vpCameraParameters cam(800.0, 800.0, 200, 150);
  cam.printParameters() ;

  vpMatrix K = cam.get_K() ;

//Matrix of intrinsic parameters 
 
  cout << "Matrice des paramètres intrinsèques" << endl ;
  cout << K << endl ;

/*

  // On positionne une camera c1 à la position c1Tw (ici le repere repère Rw est 2m devant Rc1 
  //We position a camera c1 at position c1Tw (here the reference mark Rw is 2m in front of Rc1
  vpHomogeneousMatrix  c1Tw(0,0,2.5,  vpMath::rad(0),vpMath::rad(0),0) ;
  //on simule l'image vue par c1 //we simulate the image seen by c1
  sim.setCameraPosition(c1Tw);
  // on recupère l'image //we recover the image
  sim.getImage(I1,cam);
  cout << "Image I1g " <<endl ;
  cout << c1Tw << endl ;
*/

  // On positionne une camera c2 à la position c2Tw //Positioning a camera c2 at position c2Tw
  for(float i=-1;i<=1;i=i+0.1){
    for(float j=-0.6;j<=0.6;j=j+0.1){
      vpHomogeneousMatrix c2Tw(i,j,2.5,
        vpMath::rad(0),vpMath::rad(0),0) ; //0.1,0,2, vpMath::rad(0),vpMath::rad(0),0) ;
        //on simule l'image vue par c2 //we simulate the image seen by c2
        sim.setCameraPosition(c2Tw);
        // on recupère l'image I2 //we recover the image I2
        sim.getImage(I2,cam);
        cout << "Image I1d " <<endl ;
        cout << c2Tw << endl ;
        float io=floorf(i * 100) / 100;
        float jo=floorf(j * 100) / 100;
        string loli = to_string(io);
        string lolj = to_string(jo);
        vpImageIo::write(I2,loli+"BY"+lolj+".pgm") ;
        vpDisplay::flush(I2);

    }  
  }
  
 
 /*

  // On affiche l'image I1 //We display image I1
  vpDisplayX d1(I1,10,10,"I1") ;
  vpDisplay::display(I1) ;
  vpDisplay::flush(I1) ;

  

  // On affiche l'image I2 //We display image I2
  vpDisplayX d2(I2,10,400,"I2") ;
  vpDisplay::display(I2) ;
  vpDisplay::flush(I2) ;

  */


  // sauvegarde des images resultats (en jpg et ppm) //save images results (in jpg and ppm)
//  vpImageIo::write(I1,"I1.jpg") ;
//  vpImageIo::write(I1,"I1.pgm") ;

  //vpImageIo::write(I2,"I2.jpg") ;
  //vpImageIo::write(I2,"I2.pgm") ;

/*

  vpDisplay::getClick(I2) ;
  cout << "OK " << endl ;

  vpDisplay::close(I2) ;
  vpDisplay::close(I1) ;

 */ 


  

  return 0;
}