//FuzzyControllerV1.0
//by socoolzjm
#include "iostream"
#include "math.h"
#include "fstream"
#include "FuzzyController.h"
using namespace std;
const unsigned int FuzzyController::fuzzySetNum = 7;
const unsigned int FuzzyController::fuzzyRuler[7][7] = {	{ 0, 0, 1, 1, 2, 2, 3 },
															{ 0, 1, 1, 2, 2, 3, 4 },
															{ 1, 1, 2, 2, 3, 4, 4 },
															{ 1, 2, 2, 3, 4, 4, 5 },	
															{ 2, 2, 3, 4, 4, 5, 5 },
															{ 2, 3, 4, 4, 5, 5, 6 },
															{ 3, 4, 4, 5, 5, 6, 6 } };

FuzzyController::FuzzyController()
{	
}
//模糊控制器初始化函数
void FuzzyController::FuzzyControllerInitialize(int discreteExMinValue=-3, int discreteExMaxValue=3, int inputMagnificationValue=50,//误差（误差变化率也共用这个值）离散最小值（和隶属度函数有关，要改一起改，否则用默认值），误差（误差变化率也共用这个值）离散最大值（和隶属度函数有关，要改一起改，否则用默认值），离散值的取值倍数（越大越描述越精细，初始化速度越慢）
	int discreteUMinValue=-5, int discreteUMaxValue=5, int outputMagnificationValue=50,//输出（误差变化率也共用这个值）离散最小值（和隶属度函数有关，要改一起改，否则用默认值），输出（误差变化率也共用这个值）离散最大值（和隶属度函数有关，要改一起改，否则用默认值），离散值的取值倍数（越大越描述越精细，初始化速度越慢）
	int inputEMinValue=-3, int inputEMaxValue=-3,//实际误差（实际误差变化率也共用这个值）输入的最小值（大概范围），实际误（实际误差变化率也共用这个值）差输入的最大值（大概范围）
	float KeValue=1, float KecValue=1, float KuValue=1)//误差量化因子，误差变化率量化因子，输出的比例因子
{
	discreteExMin = discreteExMinValue;
	discreteExMax = discreteExMaxValue;
	inputMagnification = inputMagnificationValue;
	discreteUMin = discreteUMinValue;
	discreteUMax = discreteUMaxValue;
	outputMagnification = outputMagnificationValue;
	inputEMin = inputEMinValue;
	inputEMax = inputEMaxValue;
	Ke = KeValue;
	Kec = KecValue;
	Ku = KuValue;
	InputFuzzySubsetGenerate(discreteExMin, discreteExMax, inputMagnification);
	OutputFuzzySubsetGenerate(discreteUMin, discreteUMax, outputMagnification);
	FuzzyResponseSheetGenerate(inputDiscreteVariableNum, exfuzzyVariableSheet,
		inputDiscreteVariableNum, ecxfuzzyVariableSheet,
		outputDiscreteVariableNum, uxfuzzyVariableSheet,
		fuzzySetNum);
	
}

void FuzzyController::InputFuzzySubsetGenerate(int min, int max, int magnification)
{
	inputDiscreteVariableNum = (unsigned int)magnification*(max - min) + 1;
	exfuzzyVariableSheet = new fuzzyVariableSheet*[fuzzySetNum];
	ecxfuzzyVariableSheet = new fuzzyVariableSheet*[fuzzySetNum];
	for (unsigned int i = 0; i < fuzzySetNum; i++)
	{
		exfuzzyVariableSheet[i] = new fuzzyVariableSheet[inputDiscreteVariableNum];
		ecxfuzzyVariableSheet[i] = exfuzzyVariableSheet[i];
	}
	for (unsigned int temp1 = 0; temp1 < fuzzySetNum; temp1++)
	{
		for (unsigned int temp2 = 0; temp2 < inputDiscreteVariableNum; temp2++)
		{
			exfuzzyVariableSheet[temp1][temp2].fuzzySet = temp1;
			exfuzzyVariableSheet[temp1][temp2].discreteVariable = (float)min + (float)temp2 / magnification;
			exfuzzyVariableSheet[temp1][temp2].memberShipDegree = InputMemberShipFunction(temp1, exfuzzyVariableSheet[temp1][temp2].discreteVariable);
		}
	}
}

void FuzzyController::OutputFuzzySubsetGenerate(int min, int max, int magnification)
{
	outputDiscreteVariableNum = (unsigned int)magnification*(max - min) + 1;
	uxfuzzyVariableSheet = new fuzzyVariableSheet*[fuzzySetNum];
	for (unsigned int i = 0; i < fuzzySetNum; i++)
	{
		uxfuzzyVariableSheet[i] = new fuzzyVariableSheet[outputDiscreteVariableNum];
	}
	for (unsigned int temp1 = 0; temp1 < fuzzySetNum; temp1++)
	{
		for (unsigned int temp2 = 0; temp2 < outputDiscreteVariableNum; temp2++)
		{
			uxfuzzyVariableSheet[temp1][temp2].fuzzySet = temp1;
			uxfuzzyVariableSheet[temp1][temp2].discreteVariable = (float)min + (float)temp2 / magnification;
			uxfuzzyVariableSheet[temp1][temp2].memberShipDegree = OutputMemberShipFunction(temp1,
				uxfuzzyVariableSheet[temp1][temp2].discreteVariable);
		}
	}
}

float FuzzyController::TriangularMembershipFunction(float variable, float a, float b, float c)
{
	if ((a < b) && (b<c))
	{
		if (variable <= a)
		{
			return 0;
		}
		else if ((variable > a) && (variable <= b))
		{
			return (variable - a) / (b - a);
		}
		else if ((variable > b) && (variable < c))
		{
			return (c - variable) / (c - b);
		}
		else
		{
			return 0;
		}
	}
	else if ((a == b) && (b < c))
	{
		if (variable <= b)
		{
			return 1;
		}
		else if ((variable > b) && (variable < c))
		{
			return (c - variable) / (c - b);
		}
		else
		{
			return 0;
		}
	}
	else if ((a < b) && (b == c))
	{
		if (variable <= a)
		{
			return 0;
		}
		else if ((variable > a) && (variable < b))
		{
			return (variable - a) / (b - a);
		}
		else
		{
			return 1;
		}
	}
	else
	{
		cout << "TriangularMembershipFunction Error" << endl;
		return -1;
	}
}

float FuzzyController::InputMemberShipFunction(unsigned int languageLevel, float subsetInput)
{
	switch (languageLevel)
	{
	case 0:
		return TriangularMembershipFunction(subsetInput, -3.0, -3.0, -1.0);
		break;
	case 1:
		return TriangularMembershipFunction(subsetInput, -3.0, -2.0, 0.0);
		break;
	case 2:
		return TriangularMembershipFunction(subsetInput, -3.0, -1.0, 1.0);
		break;
	case 3:
		return TriangularMembershipFunction(subsetInput, -2.0, 0.0, 2.0);
		break;
	case 4:
		return TriangularMembershipFunction(subsetInput, -1.0, 1.0, 3.0);
		break;
	case 5:
		return TriangularMembershipFunction(subsetInput, 0.0, 2.0, 3.0);
		break;
	case 6:
		return TriangularMembershipFunction(subsetInput, 1.0, 3.0, 3.0);
		break;
	default:
		cout << "InputMemberShipFunction Error" << endl;
		return -1;
		break;
	}
}

float FuzzyController::OutputMemberShipFunction(unsigned int languageLevel, float subsetInput)
{
	switch (languageLevel)
	{
	case 0:
		return TriangularMembershipFunction(subsetInput, -5.0, -5.0, -1.5);
		break;
	case 1:
		return TriangularMembershipFunction(subsetInput, -5.0, -3.0, 0.0);
		break;
	case 2:
		return TriangularMembershipFunction(subsetInput, -5.0, -1.5, 1.5);
		break;
	case 3:
		return TriangularMembershipFunction(subsetInput, -3.0, 0.0, 3.0);
		break;
	case 4:
		return TriangularMembershipFunction(subsetInput, -1.5, 1.5, 5.0);
		break;
	case 5:
		return TriangularMembershipFunction(subsetInput, 0.0, 3.0, 5.0);
		break;
	case 6:
		return TriangularMembershipFunction(subsetInput, 1.5, 5.0, 5.0);
		break;
	default:
		cout << "OutputMemberShipFunction Error" << endl;
		return -1;
		break;
	}
}

unsigned int FuzzyController::FuzzificationFunction(float input, int inputMin, int inputMax, float K, fuzzyVariableSheet **variableSheet)
{
	unsigned int temp1 = 0, temp2 = 0;
	float inputDiscreteVariable;
	input = K*input;
	//cout << "input=" << input << endl;
	//cout << "inputMin=" << inputMin << endl;
	//cout << "inputMax=" << inputMax << endl;
	if (input < inputMin)
	{
		inputDiscreteVariable = (float)discreteExMin;
		//cout << "1" << endl;
	}
	else if (input>inputMax)
	{
		inputDiscreteVariable = (float)discreteExMax;
		//cout << "2" << endl;
	}
	else
	{
		inputDiscreteVariable = ((float)(discreteExMax - discreteExMin) / (inputMax - inputMin))*floor((float)0.5+input*inputMagnification - (float)(inputMin + inputMax) / 2) / inputMagnification;
		//cout << "inputDiscreteVariable=" << inputDiscreteVariable << endl;
		//cout << "3" << endl;
	}
	unsigned int discreteVariableRowPosition = (inputDiscreteVariable - (float)discreteExMin)*inputMagnification;
	return discreteVariableRowPosition;
}

FuzzyController::fuzzyVariableSheet *FuzzyController::FuzzyInference(unsigned int discreteVariableERowPosition, unsigned int inputVariableENum, fuzzyVariableSheet **variableESheet,
	unsigned int discreteVariableEcRowPosition, unsigned int inputVariableEcNum, fuzzyVariableSheet **variableEcSheet,
	unsigned int outputVariableUNum, fuzzyVariableSheet **variableUSheet,
	unsigned int fuzzySetNum)
{
	fuzzyVariableSheet *fuzzyOutputVariable = new fuzzyVariableSheet[outputVariableUNum];
	unsigned int i;
	for (i = 0; i < outputVariableUNum; i++)
	{
		fuzzyOutputVariable[i].memberShipDegree = 0;
	}
	unsigned int k = 0;
	float tempMin;
	
	unsigned int j;
	for (i = 0; i < fuzzySetNum; i++)//eXec
	{
		for (j = 0; j < fuzzySetNum; j++)
		{
			if (variableESheet[i][discreteVariableERowPosition].memberShipDegree < variableEcSheet[j][discreteVariableEcRowPosition].memberShipDegree)
			{
				tempMin = variableESheet[i][discreteVariableERowPosition].memberShipDegree;
			}
			else
			{
				tempMin = variableEcSheet[j][discreteVariableEcRowPosition].memberShipDegree;
			}
			for (k = 0; k < outputVariableUNum; k++)//(eXec)Xu
			{
				if (tempMin < variableUSheet[fuzzyRuler[j][i]][k].memberShipDegree)
				{
					if (tempMin>fuzzyOutputVariable[k].memberShipDegree)
					{
						fuzzyOutputVariable[k].memberShipDegree = tempMin;
					}
				}
				else
				{
					if (variableUSheet[fuzzyRuler[j][i]][k].memberShipDegree > fuzzyOutputVariable[k].memberShipDegree)
					{
						fuzzyOutputVariable[k].memberShipDegree = variableUSheet[fuzzyRuler[j][i]][k].memberShipDegree;
					}
				}
				fuzzyOutputVariable[k].discreteVariable = variableUSheet[0][k].discreteVariable;
			}
		}
	}
	//cout << "outputVariableUNum=" << outputVariableUNum << endl;
	//cout << "fuzzyOutputVariable[outputVariableUNum-1].discreteVariable=" << fuzzyOutputVariable[outputVariableUNum - 1].discreteVariable << endl;
	return fuzzyOutputVariable;
}

void FuzzyController::FuzzyResponseSheetGenerate(unsigned int inputVariableENum, fuzzyVariableSheet **variableESheet,
	unsigned int inputVariableEcNum, fuzzyVariableSheet **variableEcSheet,
	unsigned int outputVariableUNum, fuzzyVariableSheet **variableUSheet,
	unsigned int fuzzySetNum)
{
	fuzzyResponseSheet = new float*[inputVariableEcNum];
	for (unsigned int i = 0; i < inputVariableENum; i++)
	{
		fuzzyResponseSheet[i] = new float[inputVariableENum];
	}
	for (unsigned int tempinputVariableEcPosition = 0; tempinputVariableEcPosition < inputVariableEcNum; tempinputVariableEcPosition++)
	{
		for (unsigned int tempinputVariableEPosition = 0; tempinputVariableEPosition < inputVariableENum; tempinputVariableEPosition++)
		{
			fuzzyResponseSheet[tempinputVariableEcPosition][tempinputVariableEPosition] = Defuzzification(
				FuzzyInference(tempinputVariableEPosition, inputVariableENum, exfuzzyVariableSheet,
				tempinputVariableEcPosition, inputVariableEcNum, ecxfuzzyVariableSheet,
				outputVariableUNum, uxfuzzyVariableSheet, fuzzySetNum),
				outputVariableUNum,
				1);
		}
	}


}

void FuzzyController::DisplayfuzzyVariableSheet(fuzzyVariableSheet **variableSheet, unsigned int SheetColNum, unsigned int SheetRowNum)
{
	for (unsigned int temp1 = 0; temp1 < SheetColNum; temp1++)
	{
		for (unsigned int temp2 = 0; temp2 < SheetRowNum; temp2++)
		{
			cout << variableSheet[temp1][temp2].memberShipDegree << " ";
		}
		cout << endl;
	}
	cout << endl;
}

float FuzzyController::Defuzzification(fuzzyVariableSheet *fuzzyOutputVariable, unsigned int outputVariableUNum, unsigned int method)
{
	/*
	for (unsigned int i = 0; i < outputVariableUNum; i++)
	{
	cout << fuzzyOutputVariable[i].memberShipDegree << " " << fuzzyOutputVariable[i].discreteVariable << endl;
	}
	cout << endl;
	*/
	fuzzyVariableSheet tempOutputVariable = fuzzyOutputVariable[0];
	/* 最大隶属度法（半成品）
	for (unsigned int i = 0; i < outputVariableUNum; i++)
	{
	if (fuzzyOutputVariable[i].memberShipDegree>tempMaxOutputVariable.memberShipDegree)
	{
	tempMaxOutputVariable = fuzzyOutputVariable[i];
	}
	}
	*/
	switch (method)
	{
	case 1://面积重心法
	{
			   float tempMemberShipDegreeSum = 0, tempDiscreteVariableSum = 0;
			   for (unsigned int i = 0; i < outputVariableUNum; i++)
			   {
				   tempDiscreteVariableSum = tempDiscreteVariableSum + fuzzyOutputVariable[i].discreteVariable*fuzzyOutputVariable[i].memberShipDegree;
				   tempMemberShipDegreeSum = tempMemberShipDegreeSum + fuzzyOutputVariable[i].memberShipDegree;
			   }
			   tempOutputVariable.discreteVariable = tempDiscreteVariableSum / tempMemberShipDegreeSum;
			   //cout << "tempOutputVariable.discreteVariable=" << tempOutputVariable.discreteVariable << endl;
			   return  tempOutputVariable.discreteVariable;
			   break;
	}
	default:
	{
			   cout << "Defuzzification Method Error" << endl;
			   return 0;
			   break;
	}
	}
}
//控制器输出控制量（float）
float FuzzyController::FuzzyControlOutput(float inputE, float inputEc)//误差，相邻两次误差的差值
{
	unsigned int discreteEcPosition = FuzzificationFunction(inputEc, inputEMin, inputEMax, Kec, exfuzzyVariableSheet);
	unsigned int discreteEPosition = FuzzificationFunction(inputE, inputEMin, inputEMax, Ke, ecxfuzzyVariableSheet);
	return fuzzyResponseSheet[discreteEcPosition][discreteEPosition] * Ku;
}