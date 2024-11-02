#include <iostream>
#include <cmath>
#include <string>
#include <fstream>
using namespace std;

//This is the declaration of the readFile Void function
void readFile(string, double[], double[], double[], double[], double[], double[], int);


//Declaration for the yaw, pitch and roll transformation functions
void yawMatFunc(double, double[][3]);
void pitchMatFunc(double, double[][3]);
void rollMatFunc(double, double[][3]);

//This calls the function that converts the NED values to XYZ values that can be used
void Rot_NED_to_Body(double[], double[], double[], double[], double[], double[], double[], double[], double[], int);

//This is the function that fineds the attack angle, side slip, and the airspeed V_t
void velParams(double [], double[], double[], double[], double[], double[], int);

//Function that calcultats the STD
double calculateSTD(double[], int);

//This will write all of the data found into a CSV file

double writeCSV(string, double[], double[], double[], int);


int main(){
    //int variable that determines the size of the 1d arrays
    int size = 10;
    
    //Name of the file that will be read from
    string inputFilename = "aircraftdata.txt";
    
    //The velocity vector in the north, east and down direction
    double V_N[size] = {0};
    double V_E[size] = {0};
    double V_D[size] = {0};
    
    //Euler angle vectors for yaw, pitch and roll
    double yaw[size] = {0};
    double pitch[size] = {0};
    double roll[size] = {0};
    
    //calling function that will read the values fromt the aircraft document into arrays
    readFile(inputFilename, V_N, V_E, V_D, yaw, pitch, roll, size);
   
    //New transformed vectors of NED to xyz declaration
    
    double V_x[size] = {0};
    double V_y[size] = {0};
    double V_z[size] = {0};
    
    // calling function that transforms the NED values to XYZ values
    Rot_NED_to_Body(V_N, V_E, V_D, yaw, pitch, roll, V_x, V_y, V_z, size);
    
    //Intializes V_t, Attack Angle, and Side Slip arrarys
    double V_t[size] = {0};
    double Attack_ang[size] = {0};
    double SideSlip_ang[size] = {0};
    //Function that defines V_t, Attack Ang, and Side Slip
    velParams(V_x, V_y, V_z, V_t, Attack_ang, SideSlip_ang, size);
    
    //Assigns standard of deviation to the caluclateSTD function
    double standard_deviation = calculateSTD(V_t, size);
    cout << standard_deviation;
    
    //Writes V_t, AttackAng, and Sideslip to arrays in output.csv function
    writeCSV("output.csv", V_t, Attack_ang, SideSlip_ang, size);
    
    return 0;
}


//This function will open the file of aircraft data and will read the values into arrays
void readFile(string inputFilename, double V_N[], double V_E[], double V_D[], double yaw[], double pitch[], double roll[], int size){
//This opens the aircraft data file
	ifstream inputStream;
	inputStream.open(inputFilename);  
	
	//Reads the headers into arbitrary values of a-f
    string a, b, c, d, e, f;
	inputStream >> a >> b >> c >> d >> e >> f;
	
	//this will continue to read until the array size of 10 is met
	for (int j = 0; j < size; j++)
	{
	    inputStream >> V_N[j] >> V_E[j] >> V_D[j] >> yaw[j] >> pitch[j] >> roll[j];
	    //this converts the values of yaw, pitch, and roll to radians
	    yaw[j] = yaw[j] * (M_PI/180);
	    pitch[j] = pitch[j] * (M_PI/180);
	    roll[j] = roll[j] * (M_PI/180);
	    
	}
    
    //Closes the fstream
    inputStream.close();

}


void yawMatFunc(double yaw_A, double yawMat[][3]){
    //This accesses the individual values of the matrix yawMat and does the transformation from NED to xyz
    yawMat[0][0] = cos(yaw_A);
    yawMat[0][1] = sin(yaw_A);
    yawMat[1][0] = -sin(yaw_A);
    yawMat[1][1] = cos(yaw_A);
    yawMat[2][2] = 1;

}


void pitchMatFunc(double pitch_A, double pitchMat[][3]){
     //This accesses the individual values of the matrix pitchMat and does the transformation from NED to xyz
    pitchMat[0][0] = cos(pitch_A);
    pitchMat[0][2] = -sin(pitch_A);
    pitchMat[1][1] = 1;
    pitchMat[2][0] = sin(pitch_A);
    pitchMat[2][2] = cos(pitch_A);
    
}



void rollMatFunc(double roll_A, double rollMat[][3]){
     //This accesses the individual values of the matrix rollMat and does the transformation from NED to xyz
    rollMat[0][0] = 1;
    rollMat[1][1] = cos(roll_A);
    rollMat[1][2] = sin(roll_A);
    rollMat[2][1] = -sin(roll_A);
    rollMat[2][2] = cos(roll_A);
}


void Rot_NED_to_Body(double V_N[], double V_E[], double V_D[], double yaw[], double pitch[], double roll[], double V_x[], double V_y[], double V_z[], int size){
   
    
    for (int i = 0; i < size; i++){
        //This is calls the yawMatFunc and is also the declaration for yawMat
        double yaw_A = yaw[i];
        double yawMat[3][3] = {0};
        yawMatFunc(yaw_A, yawMat);
        
        //This is calls the pitchMatFunc and is also the declaration for pitchMat
        double pitch_A = pitch[i];
        double pitchMat[3][3] = {0};
        pitchMatFunc(pitch_A, pitchMat);
        
        //This is calls the rollMatFunc and is also the declaration for rollMat
        double roll_A = roll[i];
        double rollMat[3][3] = {0};
        rollMatFunc(roll_A, rollMat);
        
        
        //this is the first 3 by 3 multiplication which multiplies the roll and the pitch matricies
        double roll_times_pitch[3][3] = {0};
        
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                for(int l = 0; l < 3; l++){
                    roll_times_pitch[j][k] += rollMat[j][l] * pitchMat[l][k];
                    
                }
            } 
        }

        //this is the second 3 by 3 muliplication that muliplies the result of the roll_times_pitch by the yaw matriciy
        double nat[3][3] = {0};
        
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                for(int l = 0; l < 3; l++){
                    nat[j][k] += roll_times_pitch[j][l] * yawMat[l][k];
                }
            } 
        }
        
        //this now does the matrix multiplication for V_x, V_y, and V_z and directly assigns it to V_x, V_y, and V_z
        V_x[i] = nat[0][0] * V_N[i] + nat[0][1] * V_E[i] + nat[0][2] * V_D[i];
        
        V_y[i] = nat[1][0] * V_N[i] + nat[1][1] * V_E[i] + nat[1][2] * V_D[i];
        
        V_z[i] = nat[2][0] * V_N[i] + nat[2][1] * V_E[i] + nat[2][2] * V_D[i];
        
    }
    
    
   
    
}

void velParams(double V_x[], double V_y[], double V_z[], double V_t[], double Attack_ang[], double SideSlip_ang[], int size){
    for (int i = 0; i < size; i++){
        //this calculates V_t
        V_t[i] = sqrt(pow(V_x[i],2) + pow(V_y[i],2) + pow(V_z[i],2) );
       
        
        //This calculates the attack angle in radians and then converts it to degrees
        Attack_ang[i] = atan2(V_z[i], V_x[i]) * 180/M_PI;
        
        //This calculates the side slip in radians and then converts it to degrees
        SideSlip_ang[i] = asin(V_y[i]/V_t[i]) * 180/M_PI;
 
    }
}

//This function calculates the standard of deviation
double calculateSTD(double V_t[], int size){
    double sum = 0;
    double avg = 0;
    double meandiff = 0;
    double meansum = 0;
    double std = 0;
    
    //Calculates the sum
    for(int i = 0; i < size; i++){
        sum = sum + V_t[i];
    }
    
    //Finds the average of the sum
    avg = sum/static_cast<double>(size);
    
    //This will now sum the difference of the squaresq
    for(int i = 0; i < size; i++){
        meandiff = pow(V_t[i] - avg,2);
        meansum = meansum + meandiff;
    }
    
    //This determines the Standard of Deviation of the V_t
    std = sqrt((1.0/(static_cast<double>(size)-1)) * meansum);
 
    return std;
}

double writeCSV(string outFilename, double V_t[], double Attack_ang[], double SideSlip_ang[], int size){


	ofstream outputStream;
	// Create an output stream and open the file 
	
	outputStream.open(outFilename);  
	
	// Test if stream operatiopn failed
   if (outputStream.fail()) 
	{
		cout << "Error opening the output file."; 

	}
	
	//This creates the header for the file
	outputStream << "V_t" << "," << "Attack_ang" << "," <<"SideSlip_ang" << endl;

    //This reads the V_t, Attack ang, and side slip into the file
	for (int i = 0; i < size; i++){
	    outputStream << V_t[i] << "," << Attack_ang[i] << "," << SideSlip_ang[i] << endl;
	    
	}

    //closes the file
    outputStream.close();
    
}