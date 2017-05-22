/****************************************************************************************
 * Filename 	: main.cpp
 * Path			: models_evaluation/src
 * Institute	: Hochschule Bonn-Rhein-Sieg
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 	: main file to compute the evaluation and identification process
 **************************************************************************************/
#include "ModelOptimizer.hpp"
#include <fstream>
#include <stdlib.h>
#include <ros/package.h>
#include <luh_youbot_msgs/JointVector.h>

using namespace std;

string jointFiles[5] = {"inputs/Joint1/staticj1.dat", "inputs/Joint2/staticj2.dat", "inputs/Joint3/staticj3.dat", "inputs/Joint4/staticj4.dat", "inputs/Joint5/staticj5.dat"};
///string jointFiles_mov[5] = {"inputs/Joint1/r_joint1.dat", "inputs/Joint2/r_joint2.dat", "inputs/Joint3/r_joint3.dat", "inputs/Joint4/r_joint4.dat", "inputs/Joint5/r_joint5.dat"};
string jointFiles_mov[5] = {"inputs/Joint1/r_joint1.dat", "inputs/Joint2/r_joint2_new_3t.dat", "inputs/Joint3/r_joint3_new_3t.dat", "inputs/Joint4/r_joint4_new_3t.dat", "inputs/Joint5/r_joint5.dat"};
string outFiles[5] = {"/data/outputs/o_joint1.dat", "/data/outputs/o_joint2.dat", "/data/outputs/o_joint3.dat", "/data/outputs/o_joint4.dat", "/data/outputs/o_joint5.dat"};
string outFiles_trq[5] = {"/data/outputs/o_j_t1.dat", "/data/outputs/o_j_t2.dat", "/data/outputs/o_j_t3.dat", "/data/outputs/o_j_t4.dat", "/data/outputs/o_j_t5.dat"};
string outFiles_mov[5] = {"/data/outputs/o_mov_joint1.dat", "/data/outputs/o_mov_joint2.dat", "/data/outputs/o_mov_joint3.dat", "/data/outputs/o_mov_joint4.dat", "/data/outputs/o_mov_joint5.dat"};
string outFiles_mov_trq[5] = {"/data/outputs/o_mov_j_t1.dat", "/data/outputs/o_mov_j_t2.dat", "/data/outputs/o_mov_j_t3.dat", "/data/outputs/o_mov_j_t4.dat", "/data/outputs/o_mov_j_t5.dat"};
string outTrqError_sta[5] = {"/data/outputs/o_err_j_t1.dat", "/data/outputs/o_err_j_t2.dat", "/data/outputs/o_err_j_t3.dat", "/data/outputs/o_err_j_t4.dat", "/data/outputs/o_err_j_t5.dat"};
string outTrqError_mov[5] = {"/data/outputs/o_m_err_j_t1.dat", "/data/outputs/o_m_err_j_t2.dat", "/data/outputs/o_m_err_j_t3.dat", "/data/outputs/o_m_err_j_t4.dat", "/data/outputs/o_m_err_j_t5.dat"};
string outValTrqErr[5] = {"/data/outputs/o_v_err_j_t1.dat", "/data/outputs/o_v_err_j_t2.dat", "/data/outputs/o_v_err_j_t3.dat", "/data/outputs/o_v_err_j_t4.dat", "/data/outputs/o_v_err_j_t5.dat"};
static unsigned int poses_mov[5] ={1503, 1450, 1419, 1640, 1297};
/// Poses for the particular trajectories
///static unsigned int poses_mov[5] ={1503, 28, 34, 42, 1297};

static unsigned int poses[5] ={41, 43, 42, 50, 50};

unsigned int no_of_poses;
bool flagLoad, flagId;
/// Structure to hold the experimental data from the youBot manipulator
struct robotData
{
	double pos[5];
	double vel[5];
	double trq[5];
};
robotData *jtdata;

/****************************************************************************************
 * Functionname		: getInputs
 * Input			: Input file with Joint angles, velocities, torques
 * Output			: Joint angles, velocities, torques
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: To obtain joint angles, velocities, torques
 **************************************************************************************/
void getInputs(string packagePath)
{
	string str;
	double doublevalues[5];
	unsigned int count_pos = 0, count_vel = 0, count_trq = 0;
	const char *filePath = packagePath.c_str();
	ifstream inQ1read(filePath, ios::in);
	/// Check to ensure the file could be opened or not
	if(!inQ1read)
	{
		cerr << "File could not be opened" << endl;
		exit(ONE);
	}
	while(inQ1read >> str >> doublevalues[0] >> doublevalues[1] >> doublevalues[2] >>doublevalues[3] >>doublevalues[4])
	{		
		if(str == "Position")
		{
			jtdata[count_pos].pos[0] = doublevalues[0];
			jtdata[count_pos].pos[1] = doublevalues[1];
			jtdata[count_pos].pos[2] = doublevalues[2];
			jtdata[count_pos].pos[3] = doublevalues[3];
			jtdata[count_pos].pos[4] = doublevalues[4];
			count_pos++;
		}
		
		if(str == "Velocity")
		{
			jtdata[count_vel].vel[0] = doublevalues[0];
			jtdata[count_vel].vel[1] = doublevalues[1];
			jtdata[count_vel].vel[2] = doublevalues[2];
			jtdata[count_vel].vel[3] = doublevalues[3];
			jtdata[count_vel].vel[4] = doublevalues[4];
			count_vel++;
		}		
		
		if(str == "Torque")
		{
			jtdata[count_trq].trq[0] = doublevalues[0];
			jtdata[count_trq].trq[1] = doublevalues[1];
			jtdata[count_trq].trq[2] = doublevalues[2];
			jtdata[count_trq].trq[3] = doublevalues[3];
			jtdata[count_trq].trq[4] = doublevalues[4];
			count_trq++;
		}
	}
	
	inQ1read.close();
}

/****************************************************************************************
 * Functionname		: Main
 * Institute		: Hochschule Bonn-Rhein-Sieg
 * Author 			: Jeyaprakash Rajagopal
 * Advisors			: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 		: Main function for the identification and 
 * 					  evaluation of the robot dynamic models
 **************************************************************************************/
int main(int argc, char **argv)
{
	/// Creating an object to the class models_evaluation
	Models_evaluation model_youbot;
	/// Creating an object for ModelOptimizer
	ModelOptimizer model_optimizer;
	ModelOptimizer::State *data;
	/// Joint angles, velocities, accelerations variable declarations
	JntArray q(5), qdot(5),qdotdot(5);

	int readValue = 0;
	ofstream outClientFile, outFileTrqError, outValFileTrqErr;
	ofstream outJt;
	
	//robotData dataJoint[];
	string packagePath = model_youbot.urdf_path;
	string dataPath;
	string statmov;
	string outFileFull, outFileTorque, outTrqError, outValTrqErr_str;
	
	while(true)
	{
		outFileFull = '\0';outFileTorque='\0';outTrqError='\0';outValTrqErr_str='\0';
		dataPath  = packagePath + "/data/";
		cout << "\n***************Identification and evaluation of dynamic robot models***************" << endl << endl;
		cout << "Please select a static or moving configuration for evaluation (s-static/m-moving/q-quit): ";
		cin >> statmov;
		
		if(statmov.empty() || (statmov[0] == '-') || (statmov[0] == '+'))
		{
			system("clear"); 
			cout << "Please enter a valid input" << endl;
			continue;
		}
		else if((statmov[0] != 'm') && (statmov[0] != 's') && (statmov[0] != 'q'))
		{
			system("clear"); 
			cout << "Please enter a valid character input " << endl;
			continue;			
		}
		else if(isdigit(statmov[0]))
		{
			system("clear"); 
			cout << "Please enter a valid character input " << endl;
			continue;
		}
		else if(statmov[0] == 'q')
		{
			system("clear");
			break;
		}
		
		switch(statmov[0])
		{
			case 'm':
				cout << "Please provide an arm joint between 1 and 5 for the moving configuration, to control joints 1 - 5 :"; 
				std::cin >> readValue;

				if((readValue >= 1)&&(readValue <= 5))
				{
					cout << "Selected arm joint " << readValue << endl;
				}
				dataPath += jointFiles_mov[readValue-1];
				outFileFull = packagePath + outFiles_mov[readValue-1];
				outFileTorque = packagePath + outFiles_mov_trq[readValue-1];
				outTrqError = packagePath +  outTrqError_mov[readValue-1];
				outValTrqErr_str = packagePath +  outValTrqErr[readValue-1];
			
				no_of_poses = poses_mov[readValue-1];
				break;		
			case 's':
				cout << "Please provide an arm joint between 1 and 5  for the static configuration, to control joints 1 - 5 :"; 
				std::cin >> readValue;
		
				if((readValue >= 1)&&(readValue <= 5))
				{
					cout << "Selected arm joint " << readValue << endl;
				}
				dataPath += jointFiles[readValue-1];
				outFileFull = packagePath + outFiles[readValue-1];
				outFileTorque = packagePath + outFiles_trq[readValue-1];
				outTrqError = packagePath +  outTrqError_sta[readValue-1];
				outValTrqErr_str = packagePath +  outValTrqErr[readValue-1];
				no_of_poses = poses[readValue-1];
				break;
				
			default:
				dataPath += jointFiles[0];
				outFileFull = packagePath + outFiles[0];
				outFileTorque = packagePath + outFiles_trq[0];
				outTrqError = packagePath +  outTrqError_sta[0];
				outValTrqErr_str = packagePath +  outValTrqErr[readValue-1];
				no_of_poses = poses[0];
				break;
		}

		jtdata = new robotData[no_of_poses];

		system("clear");
		const char *out_filePath = outFileFull.c_str();
		const char *out_filePath_t = outFileTorque.c_str();
					std::vector<double> vec;
		const char *out_filePath_err = outTrqError.c_str();
		const char *out_val_err = outValTrqErr_str.c_str();

		/// Output file creation to store the torque and end-effector frame values
		outClientFile.open(out_filePath, ios::out);
		system("clear"); 
		outJt.open( out_filePath_t, ios::out);
		outFileTrqError.open(out_filePath_err, ios::out);
		outValFileTrqErr.open(out_val_err, ios::out);
		while(true)
		{
			cout << "\n***************Identification and evaluation of dynamic robot models***************" << endl;
			cout << "Please select one of the following inputs" << endl;
			cout << "d - Load robot's experiment data" << endl;
			cout << "e - Evaluate" << endl;
			cout << "i - Identify a model" << endl;
			cout << "s - Save params" << endl;
			cout << "q - Quit" << endl;
			cout << "Please enter the input (d/e/i/s/q) : ";
			cout << left;
			string input;
			cin >> input;

			if(input.empty() || (input[0] == '-') || (input[0] == '+'))
			{
				system("clear"); 
				cout << "Please enter a valid input." << endl;
				continue;
			}
			else if(isdigit(input[0]))
			{
				system("clear"); 
				cout << "Please enter a valid character input. " << endl;
				continue;
			}
			else if(input[0] == 'q')
			{
				system("clear");
				break;
			}
			switch(input[0])
			{
				case 'd':
					cout << "Loading data..." << endl;
					for(int in = 0; in<no_of_poses; in++)
					{
						/// Getting the inputs from a file 
						getInputs(dataPath);
					}
					cout << "Data loading is successful. " << endl;
					flagLoad = 1;
					break;
				/// Save the model output parameters 
				case 's':
					model_optimizer.saveParams();
					break;
				/// Evaluating the dynamic robot models	
				case 'e':
					if(!flagLoad)
					{
						cout << "Please load the data before evaluating. " << endl; 
						continue;
					}
					else
					{
						cout << "Evaluating..." << endl;
						
						for(int in = 0; in<no_of_poses; in++)
						{
							q(0) = jtdata[in].pos[0];
							q(1) = jtdata[in].pos[1];
							q(2) = jtdata[in].pos[2];
							q(3) = jtdata[in].pos[3];
							q(4) = jtdata[in].pos[4];
							/// check for the moving frames
							if(statmov[0] == 'm')
							{
								qdot(0) = jtdata[in].vel[0];
								qdot(1) = jtdata[in].vel[1];
								qdot(2) = jtdata[in].vel[2];
								qdot(3) = jtdata[in].vel[3];
								qdot(4) = jtdata[in].vel[4];
								if(no_of_poses != (in+1))
								{
									qdotdot(0) = ((jtdata[in+1].vel[0]-jtdata[in].vel[0])/0.11);
									qdotdot(1) = ((jtdata[in+1].vel[1]-jtdata[in].vel[1])/0.11);
									qdotdot(2) = ((jtdata[in+1].vel[2]-jtdata[in].vel[2])/0.11);
									qdotdot(3) = ((jtdata[in+1].vel[3]-jtdata[in].vel[3])/0.11);
									qdotdot(4) = ((jtdata[in+1].vel[4]-jtdata[in].vel[4])/0.11);										
								}
							}
							std::vector<double> roboError, keiserError, luhbotsError;
							roboError.resize(5);keiserError.resize(5);luhbotsError.resize(5);
							
							/// Robocup-at-work's model torque calculation 
							roboError = model_youbot.robocupModel(q,qdot,qdotdot,in, outClientFile, outJt);
							
							if(statmov[0] == 'm')
							{
								qdot(2) = -qdot(2); 
								qdotdot(2) = -qdotdot(2); 
							}
							/// Keiser's model torque calculation 
							keiserError = model_youbot.keiserModel(q,qdot,qdotdot,in, outClientFile);
							/// Luhbot's model torque calculation 
							luhbotsError = model_youbot.luhbotsModel(q,qdot,qdotdot,in, outClientFile);
							
							outFileTrqError << jtdata[in].pos[readValue - 1] << "," << jtdata[in].trq[readValue - 1] << "," << roboError[readValue - 1] << ","<<keiserError[readValue - 1] << ","<< luhbotsError[readValue - 1]  <<"," << (jtdata[in].trq[readValue - 1] - roboError[readValue - 1]) << "," << (jtdata[in].trq[readValue - 1] - keiserError[readValue - 1])<< "," << (jtdata[in].trq[readValue - 1] - luhbotsError[readValue - 1]) << endl;
						}
						cout << "Evaluation is completed and the torque outputs are stored in models_evaluation/data/outputs. " << endl;
					}
					break;
				/// Identify the model through optimization procedure
				case 'i':
					cout << "Identifier is running..." << endl;
					data = new ModelOptimizer::State[no_of_poses];
					memcpy(&data, &jtdata, sizeof(jtdata));
					model_optimizer.loadData(statmov);
					if(statmov[0] == 's')
						model_optimizer.getOptimizedStaticParams();
					else if(statmov[0] == 'm')
						model_optimizer.getOptimizedDynamicParams();
							
					cout << "Identification is completed and save the params by pressing 's'. " << endl;
					break;
				/// Quit the program
				case 'v':
					
					for(unsigned int sta=0;sta<no_of_poses;sta++)
					{
						vector<double> idVal, inpData, inpVel,inpAcc;
						idVal.resize(5);inpData.resize(5);inpVel.resize(5);inpAcc.resize(5);
						inpData[0] = jtdata[sta].pos[0];
						inpData[1] = jtdata[sta].pos[1];
						inpData[2] = jtdata[sta].pos[2];
						inpData[3] = jtdata[sta].pos[3];
						inpData[4] = jtdata[sta].pos[4];
						if(statmov[0] == 'm')
						{
							model_optimizer.loadParams(statmov);

							inpVel[0] = jtdata[sta].vel[0];
							inpVel[1] = jtdata[sta].vel[1];
							inpVel[2] = jtdata[sta].vel[2];
							inpVel[3] = jtdata[sta].vel[3];
							inpVel[4] = jtdata[sta].vel[4];

							if(no_of_poses != (sta+1))
							{
								inpAcc[0] = ((jtdata[sta+1].vel[0]-jtdata[sta].vel[0])/0.11);
								inpAcc[1] = ((jtdata[sta+1].vel[1]-jtdata[sta].vel[1])/0.11);
								inpAcc[2] = ((jtdata[sta+1].vel[2]-jtdata[sta].vel[2])/0.11);
								inpAcc[3] = ((jtdata[sta+1].vel[3]-jtdata[sta].vel[3])/0.11);
								inpAcc[4] = ((jtdata[sta+1].vel[4]-jtdata[sta].vel[4])/0.11);										
							}
							idVal = model_optimizer.getDynamicEffort(inpData,inpVel,inpAcc);		
							outValFileTrqErr << jtdata[sta].pos[readValue - 1] << "," << jtdata[sta].trq[readValue - 1] << "," << idVal[readValue - 1] << "," << (jtdata[sta].trq[readValue - 1] - idVal[readValue - 1]) << endl;
						}
						else if(statmov[0] == 's')
						{
							model_optimizer.loadParams(statmov);
							idVal = model_optimizer.getStaticEffort(inpData,inpVel,inpAcc);
								
							outValFileTrqErr << jtdata[sta].pos[readValue - 1] << "," << jtdata[sta].trq[readValue - 1] << "," << idVal[readValue - 1] << "," << (jtdata[sta].trq[readValue - 1] - idVal[readValue - 1]) << endl;
						}
					}

					break;
				case 'q':
					break;
				default:
					break;
			}
		}
		flagLoad = 0;
		flagId =0;
		/// Closing the output file that has been created
		outClientFile.close();
		outJt.close();
		outFileTrqError.close();
		outValFileTrqErr.close();
	}

	return 0;
}
