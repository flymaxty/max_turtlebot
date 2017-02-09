#ifndef FuzzyController_h
#define FuzzyController_h

//FuzzyControllerV1.0
//by socoolzjm
class FuzzyController
{
	private:
		int inputEMin, inputEMax;
		int discreteExMin, discreteExMax, discreteUMin, discreteUMax;
		float Ke;
		float Kec;
		float Ku;

		unsigned int inputDiscreteVariableNum;
		unsigned int outputDiscreteVariableNum;
		static const unsigned int fuzzySetNum;
		int inputMagnification;
		int outputMagnification;
		float **fuzzyRelationMatrix;

		static const unsigned int fuzzyRuler[7][7];
		float **fuzzyResponseSheet;

		struct fuzzyVariableSheet
		{
			float discreteVariable;
			unsigned int fuzzySet;
			float memberShipDegree;
		}**exfuzzyVariableSheet, **ecxfuzzyVariableSheet, **uxfuzzyVariableSheet;

		void InputFuzzySubsetGenerate(int min, int max, int magnification);
		void OutputFuzzySubsetGenerate(int min, int max, int magnification);
		float InputMemberShipFunction(unsigned int languageLevel, float subsetInput);
		float OutputMemberShipFunction(unsigned int languageLevel, float subsetInput);
		float TriangularMembershipFunction(float variable, float a, float b, float c);
		unsigned int FuzzificationFunction(float input, int inputMin, int inputMax, float K, fuzzyVariableSheet **variableSheet);
		fuzzyVariableSheet *FuzzyInference(unsigned int discreteVariableERowPosition, unsigned int inputVariableENum, fuzzyVariableSheet **variableESheet,
			unsigned int discreteVariableEcRowPosition, unsigned int inputVariableEcNum, fuzzyVariableSheet **variableEcSheet,
			unsigned int outputVariableUNum, fuzzyVariableSheet **variableUSheet,
			unsigned int fuzzySetNum);
		void FuzzyResponseSheetGenerate(unsigned int inputVariableENum, fuzzyVariableSheet **variableESheet,
			unsigned int inputVariableEcNum, fuzzyVariableSheet **variableEcSheet,
			unsigned int outputVariableUNum, fuzzyVariableSheet **variableUSheet,
			unsigned int fuzzySetNum);
		void DisplayfuzzyVariableSheet(fuzzyVariableSheet **variableSheet, unsigned int SheetColNum, unsigned int SheetRowNum);
		float Defuzzification(fuzzyVariableSheet *fuzzyOutputVariable, unsigned int outputVariableUNum, unsigned int method);

	public:
		//模糊控制器初始化函数
		FuzzyController(void);
		void FuzzyControllerInitialize(int discreteExMinValue, int discreteExMaxValue, int inputMagnificationValue,//误差（误差变化率也共用这个值）离散最小值（和隶属度函数有关，要改一起改，否则用默认值），误差（误差变化率也共用这个值）离散最大值（和隶属度函数有关，要改一起改，否则用默认值），离散值的取值倍数（越大越描述越精细，初始化速度越慢）
			int discreteUMinValue, int discreteUMaxValue, int outputMagnificationValue,//输出（误差变化率也共用这个值）离散最小值（和隶属度函数有关，要改一起改，否则用默认值），输出（误差变化率也共用这个值）离散最大值（和隶属度函数有关，要改一起改，否则用默认值），离散值的取值倍数（越大越描述越精细，初始化速度越慢）
			int inputEMinValue, int inputEMaxValue,//实际误差（实际误差变化率也共用这个值）输入的最小值（大概范围），实际误（实际误差变化率也共用这个值）差输入的最大值（大概范围）
			float KeValue, float KecValue, float KuValue); //误差量化因子，误差变化率量化因子，输出的比例因子
		//控制器输出控制量（float）
		float FuzzyControlOutput(float inputE, float inputEc);//误差，相邻两次误差的差值
};

#endif //Debug_Display_h
