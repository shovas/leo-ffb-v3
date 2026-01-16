//‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹‹
//›                                                                         ﬁ
//› Module: Internals Example Source File                                   ﬁ
//›                                                                         ﬁ
//› Description: Declarations for the Internals Example Plugin              ﬁ
//›                                                                         ﬁ
//›                                                                         ﬁ
//› This source code module, and all information, data, and algorithms      ﬁ
//› associated with it, are part of CUBE technology (tm).                   ﬁ
//›                 PROPRIETARY AND CONFIDENTIAL                            ﬁ
//› Copyright (c) 1996-2007 Image Space Incorporated.  All rights reserved. ﬁ
//›                                                                         ﬁ
//›                                                                         ﬁ
//› Change history:                                                         ﬁ
//›   tag.2005.11.30: created                                               ﬁ
//›                                                                         ﬁ
//ﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂﬂ

#include "Example.hpp"          // corresponding header file
#include <math.h>               // for atan2, sqrt
#include <stdio.h>              // for sample output
#include <stdlib.h>              // for sample output

double s;	// current normalised steering position
double s_old;// previous steering position to detect state chage (dS/dT)
double b;	// current normalised wheel balance position (where FFB=0)
double y;	// current FFB output (normalised -1..0..+1)
double b_old;	// dynamically reclculated balance point on the previous step
float Ks;	// s scale 
float Kr;	// speed of force decay with wheel rotation
float Kf;	// FFB scale
float A, D;	// curve parameters
int	  fresh;// state: 1=need init values, 0=running


double FFB;

float WheelLock;
float Zacc;
float Caster;
float Camber;
float MaxFFB;
float Lx,Lz;		//rfactor coord system


float	Bt;	// pneumatic trail Pacejka params
float	Dt;
float	Ya;
float	Xo;
double  Ct, Et;


//Pacejka coefficients
//Pure Slip Condition
//Lateral Force
float	pCy1,pDy1,pDy2,pDy3,pEy1,pEy2,pEy3,pEy4,pKy1,pKy2,pKy3,pHy1,pHy2,pHy3,pVy1,pVy2,pVy3,pVy4;
//Aligning Torque
float	qBz1,qBz2,qBz3,qBz4,qBz5,qBz9,qBz10,qCz1,qDz1,qDz2,qDz3,qDz4,qDz6,qDz7,qDz8,qDz9,qEz1,qEz2,qEz3,qEz4,qEz5,qHz1,qHz2,qHz3,qHz4;
//Basic tire parameters
float	Fz0,R0;
float   Mz0_Right, Mz0_Left;
float   Noise;

double Steering, SinSteering, CosSteering;		// wheels angle
double LVel, RVel;
double VLrad, VRrad;
double LFz, RFz;
//double TyreSpeed;
double LocalRotAccelZ;
//double kL, kR;			// brake slip
double ptL, ptR;		// pneumatic trail

double GripL, GripR;


double VxL, VxR; // tyre forward speed projection
double VyL, VyR; // tyre side speed projection

double WRotL, WRotR;
double FyL, FyR;	// lateral forces


// plugin information
unsigned g_uPluginID          = 0;
char     g_szPluginName[]     = "Leo Bodnar FFB plugin";
unsigned g_uPluginVersion     = 001;
unsigned g_uPluginObjectCount = 1;
InternalsPluginInfo g_PluginInfo;

// interface to plugin information
extern "C" __declspec(dllexport)
const char* __cdecl GetPluginName() { return g_szPluginName; }

extern "C" __declspec(dllexport)
unsigned __cdecl GetPluginVersion() { return g_uPluginVersion; }

extern "C" __declspec(dllexport)
unsigned __cdecl GetPluginObjectCount() { return g_uPluginObjectCount; }

// get the plugin-info object used to create the plugin.
extern "C" __declspec(dllexport)
PluginObjectInfo* __cdecl GetPluginObjectInfo( const unsigned uIndex )
{
  switch(uIndex)
  {
    case 0:
      return  &g_PluginInfo;
    default:
      return 0;
  }
}


// InternalsPluginInfo class

InternalsPluginInfo::InternalsPluginInfo()
{
  // put together a name for this plugin
  sprintf( m_szFullName, "%s - %s", g_szPluginName, InternalsPluginInfo::GetName() );
}

const char*    InternalsPluginInfo::GetName()     const { return ExampleInternalsPlugin::GetName(); }
const char*    InternalsPluginInfo::GetFullName() const { return m_szFullName; }
const char*    InternalsPluginInfo::GetDesc()     const { return "Example Internals Plugin"; }
const unsigned InternalsPluginInfo::GetType()     const { return ExampleInternalsPlugin::GetType(); }
const char*    InternalsPluginInfo::GetSubType()  const { return ExampleInternalsPlugin::GetSubType(); }
const unsigned InternalsPluginInfo::GetVersion()  const { return ExampleInternalsPlugin::GetVersion(); }
void*          InternalsPluginInfo::Create()      const { return new ExampleInternalsPlugin(); }


// InternalsPlugin class

const char ExampleInternalsPlugin::m_szName[] = "Leo Bodnar FFB plugin";
const char ExampleInternalsPlugin::m_szSubType[] = "Internals";
const unsigned ExampleInternalsPlugin::m_uID = 1;
const unsigned ExampleInternalsPlugin::m_uVersion = 3;  // set to 3 for InternalsPluginV3 functionality and added graphical and vehicle info


PluginObjectInfo *ExampleInternalsPlugin::GetInfo(){  return &g_PluginInfo;}
void ExampleInternalsPlugin::WriteToAllExampleOutputFiles( const char * const openStr, const char * const msg ){}
void ExampleInternalsPlugin::Startup(){}
void ExampleInternalsPlugin::Shutdown(){}
void ExampleInternalsPlugin::StartSession(){}
void ExampleInternalsPlugin::EndSession(){}


void ExampleInternalsPlugin::EnterRealtime()
{

	WheelLock = 15;	// degrees
	Lx = 0.707;		// 1/2 wheel track
	Lz = 1.65;		// 1/2 wheel base
	MaxFFB = 2.5;
	Bt = 5;		// shape
	Ya = 0.99;	// ratio of below/above peaks, positive
	Xo = 12.5;	//degrees intersect
	Caster = 0.0;
	Camber = 0.0;
	Zacc = 1.0;

	Dt = 1;		// amplitude


	//ParkingLot
	Kf = 11500;					// FFB scale
	Ks = 10;					// s scale 
	A =  1.5;					// curve shape (1..+oo)
	Kr = 8.0;					// speed of tension dissipation

	fresh = 1;					// triggers init data in telemetry;
	y = 0;						// init FFB




	FILE *fi = fopen( "Plugins\\LeoFFB_v3.ini", "r" );
    if( fi != NULL )
	{

		char temp[256];	// temporary string variable

		fscanf (fi, "%f", &WheelLock);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &Lx);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &Lz);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &MaxFFB);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &Bt);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &Ya);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &Xo);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &Caster);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &Camber);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &Zacc);
		fgets (temp, 256, fi);


		fscanf (fi, "%f", &Kf);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &Ks);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &A);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &Kr);
		fgets (temp, 256, fi);


		fgets (temp, 256, fi);	//pure slip
		fgets (temp, 256, fi);	//lateral force

		fscanf (fi, "%f", &pCy1);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pDy1);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pDy2);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pDy3);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pEy1);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pEy2);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pEy3);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pEy4);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pKy1);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pKy2);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pKy3);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pHy1);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pHy2);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pHy3);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pVy1);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pVy2);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pVy3);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &pVy4);
		fgets (temp, 256, fi);

		fgets (temp, 256, fi);	//Aligning torque

		fscanf (fi, "%f", &qBz1);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qBz2);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qBz3);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qBz4);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qBz5);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qBz9);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qBz10);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qCz1);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qDz1);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qDz2);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qDz3);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qDz4);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qDz6);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qDz7);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qDz8);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qDz9);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qEz1);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qEz2);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qEz3);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qEz4);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qEz5);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qHz1);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qHz2);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qHz3);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &qHz4);
		fgets (temp, 256, fi);

		//Basic tire parameters
		fgets (temp, 256, fi);

		fscanf (fi, "%f", &Fz0);
		fgets (temp, 256, fi);
		fscanf (fi, "%f", &R0);
		fgets (temp, 256, fi);

		//Misc parameters
		fgets (temp, 256, fi);

		fscanf (fi, "%f", &Noise);
		fgets (temp, 256, fi);


		fclose( fi );				// close the parameters file
	}

	WheelLock	/= 57.3;		// convert to radians

	Zacc *= -10000;

	Xo = tan(Xo/57.29578) ;	//tan of degrees intersect

	Ct = 0.63662 * acos(-Ya);
	Et = (Bt*Xo - tan(1.5708/Ct)) / (Bt*Xo-atan(Bt*Xo));

	//parking

	if (A < 1) A = 1;
//	Kf *= 115.0;
	D = (2.0 * A - 1.0) / A;	// curve slope near zero


  FILE *fo = fopen( "Plugins\\LeoDataTest.txt", "a" );
  if( fo != NULL )
  {
    fprintf( fo, "%f %f %f %f \n", Fz0, R0, pCy1, WheelLock);
    fclose( fo );
  }

}


void ExampleInternalsPlugin::UpdateTelemetry( const TelemInfoV2 &info )
{

    const TelemWheelV2 &wheelL = info.mWheel[0];
    const TelemWheelV2 &wheelR = info.mWheel[1];
	LFz = wheelL.mTireLoad;
	RFz = wheelR.mTireLoad;

	WRotL = -wheelL.mRotation;
	WRotR = -wheelR.mRotation;

	
	s = info.mUnfilteredSteering;	// -1..0..+1
	LocalRotAccelZ = info.mLocalRotAccel.z;

	double	slipL, slipR;
	double	Btat;	// Bt*alpha t

	Steering = info.mUnfilteredSteering * WheelLock;	// radian wheels are turned
	SinSteering = sin(Steering);
	CosSteering = cos(Steering);

	//projections of local velocity onto the wheels coordinates x-longitude/brake, y-latitude/side force
	VxL = (-info.mLocalVel.z+info.mLocalRot.y*Lx) * CosSteering + (-info.mLocalVel.x+info.mLocalRot.y*Lz) * SinSteering;
	VyL = (-info.mLocalVel.z+info.mLocalRot.y*Lx) * SinSteering - (-info.mLocalVel.x+info.mLocalRot.y*Lz) * CosSteering;

//	double alpha = - atan(VyL / VxL);
	double alpha = - VyL / (abs(VxL) + 0.01);
 double a_Left = alpha;
	double gamma = 0; //camber
	double Fz = info.mWheel[0].mTireLoad;
	double dfz = (Fz - Fz0) / Fz0;
	// Left wheel
	//Lateral Force (pure)
	double SVy = Fz * ((pVy1 + pVy2 * dfz) + (pVy3 + pVy4 * dfz) * gamma);
	double SHy = (pHy1 + pHy2 * dfz) + pHy3 * gamma;
	double Ky = pKy1 * Fz0 * sin(2 * atan(Fz / (pKy2 * Fz0))) * (1 - pKy3 * abs(gamma));
	double ay = alpha + SHy;
	double signa = 1;
	if (ay < 0) signa = -1;
	double Ey = (pEy1 + pEy2 * dfz) * (1 - (pEy3 + pEy4 * gamma) * signa);
	double my = (pDy1 + pDy2 * dfz) * (1 - pDy3 * gamma * gamma);
	double Dy = my * Fz;
	double Cy = pCy1;
	double By = Ky / (Cy * Dy);
	double Fy0 = Dy * sin(Cy * atan(By * ay - Ey * (By * ay - atan(By * ay)))) + SVy;

	if (wheelL.mSurfaceType == 2)  Fy0 *= 0.65;
	if (wheelL.mSurfaceType == 3)  Fy0 *= 0.60;
	if (wheelL.mSurfaceType == 4)  Fy0 *= 0.52;
	if (wheelL.mSurfaceType == 5)  Fy0 *= 0.95;

 if (Noise > 1) Fy0 = - wheelL.mLateralForce; //// REPLACEMENT

// combined slip hack
//	double Fxmax = 2.2 * Fz * cos(3.141/180.0 * alpha);
//	double Fymax = Fy0 * sin(3.141/180.0 * alpha);
//	double Fbrk = info.mUnfilteredBrake * 0.96 * 3980 / 0.327   * 0.55;  //??? Brk torque (Nm) / Radius (m) * ratio	
//	if (Fbrk > Fxmax) Fbrk = Fxmax;
//	Fy0 *= 1 - Fbrk/Fxmax * 0.8;

 double Fy_Left = Fy0;	

	//Aligning Torque (pure)
	double Dr = Fz * ((qDz6 + qDz7 * dfz) + (qDz8 + qDz9 * dfz) * gamma) * R0;
	double Br = qBz9 + qBz10 * By * Cy;
	double SHt = qHz1 + qHz2 * dfz + (qHz3 + qHz4 * dfz) * gamma;
	double Bt  = (qBz1 + qBz2 * dfz + qBz3 * dfz * dfz) * (1 + qBz4 * gamma + qBz5 * abs(gamma));
	double Ct  = qCz1;
	double Dt = Fz * (qDz1 + qDz2 * dfz) * (1 + qDz3 * gamma + qDz4 * gamma * gamma) * (R0/Fz0);
	double at = alpha + SHt;
	double Et = (qEz1 + qEz2 * dfz + qEz3 * dfz * dfz) * (1 + (qEz4 + qEz5 * gamma) * 0.63662 * atan(Bt * Ct * at));
	double SHf = SHy + SVy / Ky;
	double ar = alpha + SHf;
	double Mzr = Dr * cos(atan(Br * ar)) * cos(alpha);
	double t = Dt * cos(Ct * atan(Bt * at - Et * (Bt * at - atan(Bt * at)))) * cos(alpha);
	Mz0_Left = -t * Fy0 + Mzr;

	if (Fz <= 0.0)
		{
		Mz0_Left = 0;
		Fy_Left = 0;
		}

	VxR = (-info.mLocalVel.z-info.mLocalRot.y*Lx) * CosSteering + (-info.mLocalVel.x+info.mLocalRot.y*Lz) * SinSteering;
	VyR = (+info.mLocalVel.z+info.mLocalRot.y*Lx) * SinSteering - ( info.mLocalVel.x-info.mLocalRot.y*Lz) * CosSteering;

//	alpha = - atan(VyR / VxR);		//right
	alpha = - VyR / (abs(VxR) + 0.01);
double a_Right = alpha;
	gamma = 0; //camber
	Fz = info.mWheel[1].mTireLoad;	//right
	dfz = (Fz - Fz0) / Fz0;
	// Left wheel
	//Lateral Force (pure)
	SVy = Fz * ((pVy1 + pVy2 * dfz) + (pVy3 + pVy4 * dfz) * gamma);
	SHy = (pHy1 + pHy2 * dfz) + pHy3 * gamma;
	Ky = pKy1 * Fz0 * sin(2 * atan(Fz / (pKy2 * Fz0))) * (1 - pKy3 * abs(gamma));
	ay = alpha + SHy;
	signa = 1;
	if (ay < 0) signa = -1;
	Ey = (pEy1 + pEy2 * dfz) * (1 - (pEy3 + pEy4 * gamma) * signa);
	my = (pDy1 + pDy2 * dfz) * (1 - pDy3 * gamma * gamma);
	Dy = my * Fz;
	Cy = pCy1;
	By = Ky / (Cy * Dy);
	Fy0 = Dy * sin(Cy * atan(By * ay - Ey * (By * ay - atan(By * ay)))) + SVy;

	if (wheelR.mSurfaceType == 2)  Fy0 *= 0.65;
	if (wheelR.mSurfaceType == 3)  Fy0 *= 0.60;
	if (wheelR.mSurfaceType == 4)  Fy0 *= 0.52;
	if (wheelR.mSurfaceType == 5)  Fy0 *= 0.95;

 if (Noise > 1) Fy0 = + wheelR.mLateralForce; //// REPLACEMENT
	
// combined slip hack
//	Fxmax = 2.2 * Fz * cos(3.141/180.0 * alpha);
//	double Fymax = Fy0 * sin(3.141/180.0 * alpha);
//	Fbrk = info.mUnfilteredBrake * 0.96 * 3980 / 0.327   * 0.55;  //??? Brk torque (Nm) / Radius (m) * ratio	
//	if (Fbrk > Fxmax) Fbrk = Fxmax;
//	Fy0 *= 1 - Fbrk/Fxmax * 0.8;

	double Fy_Right = Fy0;
	//Aligning Torque (pure)
	Dr = Fz * ((qDz6 + qDz7 * dfz) + (qDz8 + qDz9 * dfz) * gamma) * R0;
	Br = qBz9 + qBz10 * By * Cy;
	SHt = qHz1 + qHz2 * dfz + (qHz3 + qHz4 * dfz) * gamma;
	Bt  = (qBz1 + qBz2 * dfz + qBz3 * dfz * dfz) * (1 + qBz4 * gamma + qBz5 * abs(gamma));
	Ct  = qCz1;
	Dt = Fz * (qDz1 + qDz2 * dfz) * (1 + qDz3 * gamma + qDz4 * gamma * gamma) * (R0/Fz0);
	at = alpha + SHt;
	Et = (qEz1 + qEz2 * dfz + qEz3 * dfz * dfz) * (1 + (qEz4 + qEz5 * gamma) * 0.63662 * atan(Bt * Ct * at));
	SHf = SHy + SVy / Ky;
	ar = alpha + SHf;
	Mzr = Dr * cos(atan(Br * ar)) * cos(alpha);
	t = Dt * cos(Ct * atan(Bt * at - Et * (Bt * at - atan(Bt * at)))) * cos(alpha);
	Mz0_Right = -t * Fy0 + Mzr;

	if (Fz <= 0.0) 
		{
		Mz0_Right = 0;
		Fy_Right = 0;
		}

/*	ptL = 0;
	if (VxL !=0){
		Btat = Bt * (VyL / fabs(VxL) - Camber);
		ptL = Dt * cos(Ct * atan(Btat - Et * (Btat - atan(Btat))));
		ptL /= (1 + Btat * Btat);
		}
	ptR = 0;
	if (VxR !=0){
		Btat = Bt * (VyR / fabs(VxR) + Camber);
		ptR = Dt * cos(Ct * atan(Btat - Et * (Btat - atan(Btat))));
		ptR /= (1 + Btat * Btat);
		}

*/

/*
	
	FILE *fo = fopen( "Plugins\\LeoStat.txt", "a" );
  if( fo != NULL )
  {
    fprintf( fo, "%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e\n",
		SVy, SHy, Ky, ay, Ey, my, Dy, Cy, By, Fy0, VyL, VxL, VyR, VxR, a_Left*180/3.141, a_Right*180/3.141, info.mLocalVel.x, info.mLocalVel.z, info.mLocalRot.y, Steering, info.mWheel[0].mTireLoad, info.mWheel[1].mTireLoad, Mz0_Left, Mz0_Right, Fy_Left, Fy_Right, info.mWheel[0].mLateralForce, info.mWheel[1].mLateralForce, info.mLocalVel.z, info.mLocalRot.y);
    fclose( fo );
  }

*/
/*
	FILE *fo = fopen( "Plugins\\LeoStat.txt", "a" );
	if( fo != NULL )
		{
		fprintf( fo, "%f,",info.mClutchRPM);
		fprintf( fo, "%f,",info.mDeltaTime);
	//	fprintf( fo, "%d,",info.mDentSeverity[0]);
	//	fprintf( fo, "%d,",info.mDentSeverity[7]);
		fprintf( fo, "%i,",info.mDetached);
		fprintf( fo, "%f,",info.mEngineMaxRPM);
		fprintf( fo, "%f,",info.mEngineOilTemp);
		fprintf( fo, "%f,",info.mEngineRPM);
		fprintf( fo, "%f,",info.mEngineWaterTemp);
	//	fprintf( fo, "%,",info.mExpansion);
		fprintf( fo, "%f,",info.mFuel);
		fprintf( fo, "%d,",info.mGear);
		fprintf( fo, "%d,",info.mLapNumber);
		fprintf( fo, "%f,",info.mLapStartET);
		fprintf( fo, "%f,",info.mLastImpactET);
		fprintf( fo, "%f,",info.mLastImpactMagnitude);
		fprintf( fo, "%f,",info.mLastImpactPos.x);
		fprintf( fo, "%f,",info.mLastImpactPos.y);
		fprintf( fo, "%f,",info.mLastImpactPos.z);
		fprintf( fo, "%f,",info.mLocalAccel.x);
		fprintf( fo, "%f,",info.mLocalAccel.y);
		fprintf( fo, "%f,",info.mLocalAccel.z);
		fprintf( fo, "%f,",info.mLocalRot.x);
		fprintf( fo, "%f,",info.mLocalRot.y);
		fprintf( fo, "%f,",info.mLocalRot.z);
		fprintf( fo, "%f,",info.mLocalRotAccel.x);
		fprintf( fo, "%f,",info.mLocalRotAccel.y);
		fprintf( fo, "%f,",info.mLocalRotAccel.z);
		fprintf( fo, "%f,",info.mLocalVel.x);
		fprintf( fo, "%f,",info.mLocalVel.y);
		fprintf( fo, "%f,",info.mLocalVel.z);
		fprintf( fo, "%f,",info.mOriX.x);
		fprintf( fo, "%f,",info.mOriX.y);
		fprintf( fo, "%f,",info.mOriX.z);
		fprintf( fo, "%f,",info.mOriY.x);
		fprintf( fo, "%f,",info.mOriY.y);
		fprintf( fo, "%f,",info.mOriY.z);
		fprintf( fo, "%f,",info.mOriZ.x);
		fprintf( fo, "%f,",info.mOriZ.y);
		fprintf( fo, "%f,",info.mOriZ.z);
		fprintf( fo, "%i,",info.mOverheating);
		fprintf( fo, "%f,",info.mPos.x);
		fprintf( fo, "%f,",info.mPos.y);
		fprintf( fo, "%f,",info.mPos.z);
		fprintf( fo, "%d,",info.mScheduledStops);
		fprintf( fo, "%f,",info.mSteeringArmForce);
	//	fprintf( fo, "%s,",info.mTrackName);
		fprintf( fo, "%f,",info.mUnfilteredBrake);
		fprintf( fo, "%f,",info.mUnfilteredClutch);
		fprintf( fo, "%f,",info.mUnfilteredSteering);
		fprintf( fo, "%f,",info.mUnfilteredThrottle);
	//	fprintf( fo, "%s,",info.mVehicleName);

		int	w;
		for (w = 0; w < 4; w++){
			fprintf( fo, "%f,",info.mWheel[w].mBrakeTemp);
			fprintf( fo, "%i,",info.mWheel[w].mDetached);
		//	fprintf( fo, "%,",info.mWheel[w].mExpansion);
			fprintf( fo, "%i,",info.mWheel[w].mFlat);
			fprintf( fo, "%f,",info.mWheel[w].mGripFract);
			fprintf( fo, "%f,",info.mWheel[w].mLateralForce);
			fprintf( fo, "%f,",info.mWheel[w].mPressure);
			fprintf( fo, "%f,",info.mWheel[w].mRideHeight);
			fprintf( fo, "%f,",info.mWheel[w].mRotation);
			fprintf( fo, "%i,",info.mWheel[w].mSurfaceType);
			fprintf( fo, "%f,",info.mWheel[w].mSuspensionDeflection);
			fprintf( fo, "%f,",info.mWheel[w].mTemperature[0]);
			fprintf( fo, "%f,",info.mWheel[w].mTemperature[1]);
			fprintf( fo, "%f,",info.mWheel[w].mTemperature[2]);
		//	fprintf( fo, "%s,",info.mWheel[w].mTerrainName);
			fprintf( fo, "%f,",info.mWheel[w].mTireLoad);
			fprintf( fo, "%f,",info.mWheel[w].mWear);
			}

		fprintf( fo, "\n");
		fclose( fo );
		}
*/

//	FyL = wheelL.mLateralForce;
//	FyR = wheelR.mLateralForce;

	FyL = Fy_Left;
	FyR = Fy_Right;

	// mSurfaceType;    // 0=dry, 1=wet, 2=grass, 3=dirt, 4=gravel, 5=rumblestrip


	// distance travelled since last update (seconds)
	double	deltaC; // part of circumference travelled from last update
	deltaC = Kr * info.mDeltaTime * sqrt((-info.mLocalVel.x+info.mLocalRot.y*Lz)*(-info.mLocalVel.x+info.mLocalRot.y*Lz) + info.mLocalVel.z*info.mLocalVel.z);
	if(deltaC < 0.5){ // otherwise rotating too fast
		double x; // temp

		if (fresh == 1){	// needs initing
			s_old = Ks * info.mUnfilteredSteering;	// previous wheel position
			b_old = s_old;							// balance wheel position
			b = b_old;
			fresh = 0;	// we are done
			}

		s = Ks * info.mUnfilteredSteering;	// -Ks..0..+Ks actual wheel position

		if(s >= b){ // positive slope

			if( s >= s_old){
				// we are moving away and pulling the wheel behind
				y = 1.0;	// saturate
				if ((s - b) < 1){	// inside the curve -> calculate
					x = s - b;
					y = 1.0 - A * (x-1.0)*(x-1.0) / (A - x);
					}
				b_old = s - y / D;
				}
			if( s < s_old){
				// we are moving back linearly to a new b, b_old remains the same
				b = b_old;
				y = D * (s - b);
				}

			s_old = s;	// update the previous to detect movement direction
			} // positive slope

		if(s < b){ // negative slope

			if( s <= s_old){
				// we are moving away and pulling the wheel behind
				y = -1.0;	// saturate
				if ((b - s) < 1){	// inside the curve -> calculate
					x = b - s;		// positive
					y = 1.0 - A * (x-1.0)*(x-1.0) / (A - x);
					y = -y;			//negative slope
					}
				b_old = s - y / D;
				}
			if( s > s_old){
				// we are moving back linearly to a new b, b_old remains the same
				b = b_old;
				y = D * (s - b);	//negative
				}

			s_old = s;	// update the previous to detect movement direction
			} // negative slope
		} // otherwise rotating too fast


	if(deltaC >= 0.5){ // rotating too fast, reset when slowed down
		fresh = 1;
		y = 0;
		}

	// if we are rolling, dissipate the difference between b, b_old and s with travelled distance

	// dissipate the difference
	b  += (s-b) * deltaC;
	b_old += (s-b_old) * deltaC;

	if (wheelL.mSurfaceType == 2)  y *= 0.65;
	if (wheelR.mSurfaceType == 2)  y *= 0.65;

	if (wheelL.mSurfaceType == 3)  y *= 0.60;
	if (wheelR.mSurfaceType == 3)  y *= 0.60;

	if (wheelL.mSurfaceType == 4)  y *= 0.52;
	if (wheelR.mSurfaceType == 4)  y *= 0.52;

	if (wheelL.mSurfaceType == 5)  y *= 0.95;
	if (wheelR.mSurfaceType == 5)  y *= 0.95;
	// mSurfaceType;    // 0=dry, 1=wet, 2=grass, 3=dirt, 4=gravel, 5=rumblestrip

}

bool ExampleInternalsPlugin::ForceFeedback( float &forceValue )
{
	FFB = forceValue;

	forceValue = MaxFFB * (  -(Mz0_Left-Mz0_Right) + Caster*(FyL-FyR));

	if ((abs(VxL)+abs(VxR)) < 5.0)
		forceValue *= (abs(VxL)+abs(VxR)) / 5.0;
	
	
//	forceValue = - MaxFFB * (ptL*FyL + ptR*FyR + Caster*(FyL+FyR));

//	FILE *fo = fopen( "Plugins\\LeoStat.txt", "a" );
//	if( fo != NULL ){
//		fprintf( fo, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", VxL, VyL, VxR, VyR, ptL, ptR, FyL, FyR, forceValue);
//		fclose( fo );
//	  }

	forceValue += LocalRotAccelZ * Zacc;
//	forceValue += (abs(FyL) + abs(FyR)) * rand() / RAND_MAX * Noise / 10000.0;

	forceValue += y * Kf;

	return( true );
}


void ExampleInternalsPlugin::UpdateScoring( const ScoringInfoV2 &info ){}
bool ExampleInternalsPlugin::RequestCommentary( CommentaryRequestInfo &info ){  return( false );}
void ExampleInternalsPlugin::UpdateGraphics( const GraphicsInfoV2 &info ){}
bool ExampleInternalsPlugin::CheckHWControl( const char * const controlName, float &fRetVal ){return( false );}
void ExampleInternalsPlugin::ExitRealtime(){}
