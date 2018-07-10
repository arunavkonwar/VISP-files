#include <iostream>
#include <math.h>
#include <iomanip>
#include <fstream>
#include <ctime>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>
#include <visp/vpDisplayX.h>

#include <string>     // std::string, std::to_string


using namespace std ;



void
computeError3D(vpHomogeneousMatrix &cdTc, vpColVector &cdrc)
{
    vpPoseVector _cdrc(cdTc) ;
    cdrc = (vpColVector)_cdrc ;
}

void
computeInteractionMatrix3D(vpHomogeneousMatrix &cdTc,  vpMatrix &Lx)
{

    vpRotationMatrix cdRc(cdTc) ;
    vpThetaUVector tu(cdTc) ;

    vpColVector u ;
    double theta ;

    tu.extract(theta,u);
    vpMatrix Lw(3,3) ;
    Lw[0][0] = 1 ;
    Lw[1][1] = 1 ;
    Lw[2][2] = 1 ;

    vpMatrix sku = vpColVector::skew(u) ;
    Lw += (theta/2.0)*sku ;
    Lw += (1-vpMath::sinc(theta)/vpMath::sqr(vpMath::sinc(theta/2.0)))*sku*sku ;

    Lx.resize(6,6) ;
    Lx = 0 ;
    for (int i=0 ; i < 3 ; i++){   // bloc translation
      for (int j=0 ; j < 3 ; j++)
      {
          Lx[i][j] = cdRc[i][j] ;
          Lx[i+3][j+3] = Lw[i][j] ;
      }
    }    
}



double generator() {
  double temp1;
  double temp2, temp3;
  double result;
  int p;
  p = 1;
  time_t t;
  double PI=3.14;
  
  while( p > 0 ){
    //srand (time(NULL));
    temp2 = rand() / ( (double)RAND_MAX ); // rand() function generates an integer between 0 and  RAND_MAX, which is defined in stdlib.h.
    //cout << "tmp2 = " << temp2 << endl;
    if ( temp2 == 0 ){// temp2 is >= (RAND_MAX / 2)
      p = 1;
    }else{
      p = -1;
    }
  }
  temp1 = cos( ( 2.0 * (double)PI ) * rand() / ( (double)RAND_MAX ) );
  result = sqrt( -2.0 * log( temp2 ) ) * temp1;
  /*if( rand() / ( (double)RAND_MAX )  > 0.5){
    result *= -1;
    }*/
  //cout << "resultRAND = " << result << endl;
  return result;  // return the generated random sample to the caller

}



int main()
{
  clock_t begin = clock();
  int i,j;
  //vpImage<unsigned char> I1(300,400,0);
  vpImage<unsigned char> I2(224,224,0); //<unsigned char> for greyscale images
  //vpImage<vpRGBa> I2(300,400,0); // <vpRGBa> for color images
  vpImage<vpRGBa> Iimage(800,1200);
  
 
  vpImageIo::read(Iimage,"../data/hollywood-triangle.jpg") ;
  
// Cette partie ne sert qu'a la simulation
  // on positionne un poster dans le repere Rw

// This part is only for simulation
  // we position a poster in the rep Rw

//  double L = 0.400 ;
//  double l = 0.300;

  double L = 0.400 ;
  double l = 0.300;

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
  //vpCameraParameters cam(1110.0, 1110.0, 333, 227);
  //old parameters
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
  //-----------------------------
  vpColVector v ;
  double lambda = 0.1 ;
  vpMatrix Lx ;
  vpColVector e(6) ;
  //-----------------------------

   vpHomogeneousMatrix cdTw(0,0,1,
        vpMath::rad(0),vpMath::rad(0),0) ; //0.1,0,2, vpMath::rad(0),vpMath::rad(0),0) ;
    long k=0;
    
    double currentRelativeTx, currentRelativeTy, currentRelativeTz, currentRelativeRx, currentRelativeRy, currentRelativeRz;
    // p is for precision of the generated samples.
    double p = 0.01;
    
	for(int damn=0;damn<400;damn++){	
	
	currentRelativeTx = p*generator();
	currentRelativeTy = p*generator(); 
	currentRelativeTz = p*generator(); 
	currentRelativeRx = generator(); 
	currentRelativeRy = generator(); 
	currentRelativeRz = generator();

	vpHomogeneousMatrix cTw(currentRelativeTx,currentRelativeTy,currentRelativeTz,
	vpMath::rad(0),vpMath::rad(currentRelativeRy),vpMath::rad(currentRelativeRz)) ; //0.1,0,2, vpMath::rad(0),vpMath::rad(0),0) ;
	//on simule l'image vue par c2 //we simulate the image seen by c2
	sim.setCameraPosition(cTw);
	sim.setCleanPreviousImage(true, vpColor::black); //set color, default is black
	// on recupère l'image I2 //we recover the image I2
	sim.getImage(I2,cam);
	cout << "Image I1d " <<endl ;
	cout << cTw << endl ;

	//float io=floorf(i * 100) / 100;
	//float jo=floorf(j * 100) / 100;

	//float io=(float)(((int)(i*10))/10.0);;
	//float jo=(float)(((int)(j*10))/10.0);

	//string loli = to_string(io);
	//string lolj = to_string(jo);  
	//cout<<fixed;
	//cout<<setprecision(2);
	k++;
	string lolk = to_string(k);
	//vpImageIo::write(I2,k+".jpg") ; //write to filename
	vpImageIo::write(I2,"generated_images_high_precision/"+lolk + ".jpg");
	vpHomogeneousMatrix  cdTc = cdTw * cTw.inverse() ;
	//vpPoseVector deltaT(c2Tc1) ; //  vector 6 (t,theta U)

	//vpPoseVector deltaT(cdTc) ; //  vector 6 (t,theta U)
	//cout << vpPoseVector << endl;
	//cout<<c2Tc1[0];

	//-----------------------------------------------

	vpPoseVector cdrc ;
	cdrc.buildFrom(cdTc) ;			

	computeError3D(cdTc, e) ;
	// Calcul de la matrice d'interaction
	computeInteractionMatrix3D(cdTc, Lx) ;
	//        Calcul de la loi de commande
	vpMatrix Lp ;
	Lp = Lx.pseudoInverse() ;

	v = - lambda * Lp * e ;

	//------------------------------------------------

	// c1 <-- cd
	// c2 <-- vpColor
	ofstream outfile;
	outfile.open("data_high_precision.txt", ios_base::app);
	//outfile << loli+" "+lolj<<endl;
	//outfile << deltaT.t() << endl;
	outfile << v.t() << endl;
	//cout << deltaT.t() << endl ;
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout<<elapsed_secs<<endl;
}
    
   /* 
  // On positionne une camera c2 à la position c2Tw //Positioning a camera c2 at position c2Tw
  for(float i=-0.01;i<=0.01;i=i+0.001){
    for(float j=-0.01;j<=0.01;j=j+0.001){
    	for(float l=0.99;l<=1.01;l=l+0.001){
    		for(int m=-1;m<=1;m=m+0.2){
    			for(int n=-1;n<=1;n=n+0.2){
    				for(int o=-1;o<=1;o=o+0.2){
				      	vpHomogeneousMatrix cTw(i,j,l,
					vpMath::rad(n),vpMath::rad(o),vpMath::rad(m)) ; //0.1,0,2, vpMath::rad(0),vpMath::rad(0),0) ;
					//on simule l'image vue par c2 //we simulate the image seen by c2
					sim.setCameraPosition(cTw);
					sim.setCleanPreviousImage(true, vpColor::black); //set color, default is black
					// on recupère l'image I2 //we recover the image I2
					sim.getImage(I2,cam);
					cout << "Image I1d " <<endl ;
					cout << cTw << endl ;
		
					float io=floorf(i * 100) / 100;
					float jo=floorf(j * 100) / 100;

					//float io=(float)(((int)(i*10))/10.0);;
					//float jo=(float)(((int)(j*10))/10.0);

					//string loli = to_string(io);
					//string lolj = to_string(jo);  
					//cout<<fixed;
					//cout<<setprecision(2);
					k++;
					string lolk = to_string(k);
					//vpImageIo::write(I2,k+".jpg") ; //write to filename
					vpImageIo::write(I2,"generated_images_high_preision/"+lolk + ".jpg");
					vpHomogeneousMatrix  cdTc = cdTw * cTw.inverse() ;
					//vpPoseVector deltaT(c2Tc1) ; //  vector 6 (t,theta U)

					//vpPoseVector deltaT(cdTc) ; //  vector 6 (t,theta U)
					//cout << vpPoseVector << endl;
					//cout<<c2Tc1[0];

					//-----------------------------------------------

					vpPoseVector cdrc ;
					cdrc.buildFrom(cdTc) ;			

					computeError3D(cdTc, e) ;
					// Calcul de la matrice d'interaction
					computeInteractionMatrix3D(cdTc, Lx) ;
					//        Calcul de la loi de commande
					vpMatrix Lp ;
					Lp = Lx.pseudoInverse() ;

					v = - lambda * Lp * e ;

					//------------------------------------------------

			// c1 <-- cd
			// c2 <-- vpColor
					ofstream outfile;
					outfile.open("data_high_precision.txt", ios_base::app);
					//outfile << loli+" "+lolj<<endl;
					//outfile << deltaT.t() << endl;
					outfile << v.t() << endl;
					//cout << deltaT.t() << endl ;
					clock_t end = clock();
					double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
					cout<<elapsed_secs<<endl;
				}
			}
		}
		}
    }  
  }
  */
 
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


  
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout<<elapsed_secs<<endl;
  return 0;
}
