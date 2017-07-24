#pragma once

#define MAX_GOAL_NUMBER 100

#include <windows.h>
#include <sys/timeb.h>
#include <mutex>
#include <map>
#include <shellapi.h>
#pragma comment(lib, "Shell32.lib")

struct function_cummunication
{
	/////////////////////////input////////////////////////////////

	// 목표 FPS. 
	int target_hz = 30;

	/////////////////////////output////////////////////////////////

	//exe path
	char exe_path[512];

	//exit 함수가 콜될때 마다 시간을 체크
	_timeb last_call;

	//exit 가 다시 호출되는데 걸리는 시간을 측정
	int ms = 0;

	//(ms + sleep time) 이용해서 FPS 를 계산
	//ms 가 아무리 빨라도 target_hz 만큼 조절된다.
	float actual_hz = 0;

	//navi engine 이 함수를 켜고 끌 수 있음. 
	//타의로 들어오는 함수구동 시그널:1, 자의로 구동되는 시그널:2
	int trigger = 0; // 1:triggering by eng, 2:triggering by itself

					 //함수 구동 시그널을 받았는지 체크됨. 
					 //ex) trigger = 1, fire = 0 -> 엔진이 구동 시그널을 줬지만 함수 구동 안됨.
					 //ex) trigger = 1, fire = 1 -> 엔진이 구동 시그널을 줬고, 함수 구동 됨.
	int fire = 0;
};


class IPC
{
private:
	struct function
	{
		function_cummunication *fc;

		int MMF_state = 0; // -1: disable, 0 : disconnected, 1: connected, 2: virtual MMF
		int data_size = 0;
		std::string IPC_name;
		HANDLE hMapFile;
		LPCTSTR pBuf;
	};
	struct ModuleInfo
	{
		_timeb last_call;
	};

	std::map<std::string, function> f_list;
	std::string own_name;

	int openMMF(std::map<std::string, function>::iterator &_func)
	{
		int data_size = (int)sizeof(function_cummunication) + _func->second.data_size;

		_func->second.hMapFile = CreateFileMapping(
			INVALID_HANDLE_VALUE,    // use paging file
			NULL,                    // default security
			PAGE_READWRITE,          // read/write access
			0,                       // maximum object size (high-order DWORD)
			data_size,                // maximum object size (low-order DWORD)
			_func->first.c_str());                 // name of mapping object

		if (_func->second.hMapFile == NULL)
		{
			printf("Could not create file mapping object (%s).\n", _func->first.c_str());
			_func->second.MMF_state = 0;
			return 0;
		}
		return 1;
	};
public:
	IPC::IPC()
	{

	};
	IPC::IPC(std::string _own_name)
	{
		own_name = _own_name;
	};
	IPC::~IPC()
	{
		std::map<std::string, function>::iterator finder = f_list.find(own_name);

		if (finder->second.MMF_state > 0)
		{
			finder->second.fc->trigger = 0;
			finder->second.fc->fire = 0;

			UnmapViewOfFile(finder->second.pBuf);
			CloseHandle(finder->second.hMapFile);
		}
	};

	///특정 프로세서와 연결을 만들고 data type 을 정해주는 경우 포인터를 연결시킴
	int connect(std::string f_name, int _data_size = 0)
	{
		int openMMF_flag = 0;
		std::map<std::string, function>::iterator finder = f_list.find(f_name);
		if (finder != f_list.end())
		{
			if (finder->second.MMF_state == 1) return 1;
		}
		else
		{
			f_list.insert(std::pair<std::string, function>(f_name, function()));
			finder = f_list.find(f_name);
		}

		finder->second.data_size = _data_size;
		int data_size = (int)sizeof(function_cummunication) + finder->second.data_size;

		finder->second.IPC_name = f_name;

		finder->second.hMapFile = OpenFileMapping(
			FILE_MAP_ALL_ACCESS,   // read/write access
			FALSE,                 // do not inherit the name
			finder->second.IPC_name.c_str());

		if (finder->second.hMapFile == NULL)
		{
			finder->second.MMF_state = -1;

			if (f_name == own_name)
			{
				printf("Open ipc (%s).\n", finder->second.IPC_name.c_str());
				if (openMMF(finder) == 0) return 0;
				else openMMF_flag = 1;
			}
			else
			{
				printf("Could not open ipc (%s).\n", finder->second.IPC_name.c_str());
				return 0;
			}
		}
		else printf("Connection success (%s).\n", finder->second.IPC_name.c_str());

		finder->second.pBuf = (LPTSTR)MapViewOfFile(finder->second.hMapFile, // handle to map object
			FILE_MAP_ALL_ACCESS,  // read/write permission
			0,
			0,
			data_size);

		if (finder->second.pBuf == NULL)
		{
			DWORD dw = GetLastError();
			TCHAR* message = nullptr;
			FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER,
				nullptr,
				dw,
				MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
				(TCHAR *)&message,
				0,
				nullptr);

			printf("Could not map view of file (%s).\n", finder->second.IPC_name.c_str());
			printf("%s\n", message);
			CloseHandle(finder->second.hMapFile);
			finder->second.MMF_state = -1;
			return 0;
		}

		finder->second.MMF_state = 1;

		if (openMMF_flag == 1)
		{// initialize itself
			function_cummunication temp;
			std::memcpy((void *)finder->second.pBuf, &temp, sizeof(function_cummunication));
			finder->second.MMF_state = 2;
		}

		finder->second.fc = (function_cummunication*)finder->second.pBuf;

		if (f_name == own_name)
		{
			_ftime64_s(&finder->second.fc->last_call);

			finder->second.fc->fire = 1;
			if (finder->second.fc->trigger == 0) finder->second.fc->trigger = 2;

			int bytes = GetModuleFileName(NULL, finder->second.fc->exe_path, 512);
			if (bytes >= 512)
				printf("Too long address\n -> %s\n", finder->second.fc->exe_path);
		}

		return 1;
	};
	template <typename T> T* connect(std::string f_name)
	{
		int customed_data_size = (int)sizeof(T);
		if (connect(f_name, customed_data_size) == 0) return nullptr;

		std::map<std::string, function>::iterator finder = f_list.find(f_name);

		if (own_name == f_name)
		{
			T* temp = new T;
			std::memcpy((void*)&finder->second.pBuf[sizeof(function_cummunication)], temp, sizeof(T));
			delete temp;
		}

		return (T*)&finder->second.pBuf[sizeof(function_cummunication)];
	};
	template <typename T> T* get_data(std::string f_name)
	{
		std::map<std::string, function>::iterator finder = f_list.find(_exe_name);
		if (finder == f_list.end()) return nullptr;

		std::map<std::string, function>::iterator finder = f_list.find(f_name);
		return (T*)&finder->second.pBuf[sizeof(function_cummunication)];
	};
	function_cummunication* get_state(std::string _exe_name)
	{
		std::map<std::string, function>::iterator finder = f_list.find(_exe_name);
		if (finder == f_list.end()) return nullptr;

		return finder->second.fc;
	};
	int execute(std::string exePath, std::string exeName, int console = 1)
	{
		SHELLEXECUTEINFOA p_info;

		ZeroMemory(&p_info, sizeof(SHELLEXECUTEINFOA)); //초기화
		p_info.cbSize = sizeof(SHELLEXECUTEINFOA);
		p_info.lpFile = exeName.c_str(); // 파일 이름
		p_info.lpDirectory = exePath.c_str(); //파일 위치

		p_info.nShow = console; //콘솔 show
		p_info.fMask = SEE_MASK_NOCLOSEPROCESS;

		return ShellExecuteEx(&p_info);
	};
	int isOn(std::string _exe_name)
	{
		function_cummunication *state = get_state(_exe_name);
		if (state == nullptr) return 0;
		else if (state->fire > 0) return 1;
		else return 0;
	}
	int isConnected(std::string f_name)
	{
		std::map<std::string, function>::iterator finder = f_list.find(f_name);
		if (finder == f_list.end()) return 0;

		if (finder->second.MMF_state == 1) return 1;
		else return 0;
	};

	///특정 프로세서 실행을 engine 에 요청함
	///폐기된 함수
	int start(std::string _exe_name)
	{
		//std::map<std::string, function>::iterator finder = f_list.find(_exe_name);
		//if (finder == f_list.end()) return 0;
		//if (finder->second.MMF_state <= 0) return 0;

		//if (_exe_name == own_name)
		//{
		//	finder->second.fc->fire = 1;
		//	if (finder->second.fc->trigger == 0) finder->second.fc->trigger = 2;
		//}
		//else
		//{
		//	finder->second.fc->trigger = 1;
		//}

		return 1;
	};

	///종료 요청. 해당 모듈은 exit 에서 종료 시그널을 받아 스스로 종료한다.
	int stop(std::string _exe_name)
	{
		std::map<std::string, function>::iterator finder = f_list.find(_exe_name);
		if (finder == f_list.end()) return 0;
		if (finder->second.MMF_state <= 0) return 0;

		finder->second.fc->trigger = 0;
		return 1;
	};

	/// 무조건 main loop 안에 한번 호출되어야 함. 
	///engine 이 프로세서 상태를 점검하고 종료할수 있게함.
	int exit(void)
	{
		std::map<std::string, function>::iterator finder = f_list.find(own_name);
		if (finder == f_list.end()) return 0;
		if (finder->second.MMF_state <= 0) return 0;

		if (finder->second.fc->trigger == 0)
		{
			finder->second.fc->fire = finder->second.fc->trigger;

			finder->second.MMF_state = 0;
			UnmapViewOfFile(finder->second.pBuf);
			CloseHandle(finder->second.hMapFile);
			return 1;
		}

		struct _timeb now_T;
		_ftime64_s(&now_T);
		int sec_gap = (int)(now_T.time - finder->second.fc->last_call.time);
		int mili_gap = (int)(now_T.millitm - finder->second.fc->last_call.millitm);
		float t_gap = (float)(1000 * sec_gap + mili_gap);
		finder->second.fc->ms = (int)t_gap;

		float designed_t_gap = 1000.0f / (float)finder->second.fc->target_hz;
		float laft_t_gap = designed_t_gap - t_gap;
		if (laft_t_gap>0 && t_gap >= 0) Sleep((DWORD)laft_t_gap);

		struct _timeb after_sleep;
		_ftime64_s(&after_sleep);
		sec_gap = (int)(after_sleep.time - finder->second.fc->last_call.time);
		mili_gap = (int)(after_sleep.millitm - finder->second.fc->last_call.millitm);
		t_gap = (float)(1000 * sec_gap + mili_gap);
		finder->second.fc->actual_hz = 1000.0f / t_gap;

		_ftime64_s(&finder->second.fc->last_call);

		return 0;
	};
};


struct NaviEngine
{
	int state = 0;
};

struct Cam_640480
{
	bool imshow = true;
	unsigned char data[640 * 480 * 3];
};

struct Robot
{
	//////////////////////////// input ////////////////////////////
	double set_vel = 0.0;
	double set_rotvel = 0.0;

	bool direct_on = false;
	bool navigate_on = false;
	bool near_goal = false;
	//double sub_goal_x = 0.0;
	//double sub_goal_y = 0.0;
	//double sub_goal_th = 0.0;
	int sub_goal_number = 0;
	double sub_goal[20][4]; // [order][{x, y, th, idx}]

							//////////////////////////// output ////////////////////////////

	int is_on = 0;

	double x = 0;
	double y = 0;
	double th = 0;

	double get_vel = 0.0;
	double get_rotvel = 0.0;
	bool arrived = false;

	double KeyFrame[3][3] = {}; // [idx][val] -> [val] : {Key, t, r}
};

struct ObservationData
{
	//////////////////////////// input ////////////////////////////
	int mode = 0; // 0:localize, 1:save, 2:saveDB, 3:loadDB, 9:reset
	bool always_on_flag = 0;
	bool draw_flag = 0;
	int row = 480;
	int col = 640;
	unsigned char RGB_data[640 * 480 * 3];

	//////////////////////////// output ////////////////////////////
	int num_detected_feature = 0;
	int num_DB_size = 0;
	float score[2048];
	float likelihood[2048];
};

struct Kinect1Data
{
	//////////////////////////// input ////////////////////////////
	bool get_color = 1;
	bool get_grid = 1;

	int        cgridWidth = 500;
	int        cgridHeight = 500;
	int		   cRobotCol = 250;
	int		   cRobotRow = 350;

	double mm2grid = 100.0 / 1000.0;
	int sampling_gap = 20;

	//////////////////////////// output ////////////////////////////
	unsigned char gridData[500 * 500];
	unsigned char freeData[500 * 500];
	unsigned char occupyData[500 * 500];
	unsigned char colorData[2][640 * 480 * 4];
	double RT[2][16];
};
struct Kinect2Data
{
	//////////////////////////// input ////////////////////////////
	double setYaw = 0.0;
	double setPitch = 0.0;
	bool draw_color = 0;

	//////////////////////////// output ////////////////////////////
	int        cColorWidth = 960; // 1920 / 2;
	int        cColorHeight = 540; // 1080 / 2;
	unsigned char data[960 * 540 * 3];

	///<Color>///
	bool BodyTracked[6];				//Ex) {0,0,0,0,1,0} or {1,0,0,0,1,0} or {0,0,1,0,1,1}
	double JointData[6][25][3];			//joints, ColorSpace(x,y, jointsState)
	double JointsWorldCoordinate[6][25][3];	//WorldCoordinate(x,y,z), 
											///<Texture>///
	unsigned char BodyIndexMat_data[424 * 512 * 3];		//BodyIndex Mat
	unsigned char body_img_data[424 * 512 * 1];			//body_img Mat
};

struct HumanReIdentificationData
{

	//////////////////////////// input ////////////////////////////
	int Mode = 0;	//	Mode 2: HumanTraining, Mode:3 Save, Mode 4: Load, Mode 5: Run, Mode 9: Reset
	bool draw_flag = 1;
	bool TakePicture = 1;
	int TargetLabel = 0;
	//////////////////////////// output ////////////////////////////
	float score[10 + 1];
	float Color_likelihood[10 + 1];
	float Texture_likelihood[10 + 1];
	int Color_result = 0;
	int Texture_result = 0;
	int people_count = 0;
	int frame_count_color = 0;
	int frame_count_texture = 0;
};

struct LocalizerData
{
	//////////////////////////// input ////////////////////////////
	bool received = true;

	int mode_in = 0;
	int keyFrame_in = 0;

	//-1:no commend
	int goal_size_in = 0;

	int goal_list_in[MAX_GOAL_NUMBER];

	char dataFilePath[256];

	//////////////////////////// output ////////////////////////////
	int DB_size = 0;

	int goal_size_out = 0;
	int goal_list_out[MAX_GOAL_NUMBER];

	bool drawMap = true;
	unsigned char Map[750000];
};

struct RGBDcamera {
	unsigned char colorData[4][640 * 480 * 3];
	unsigned short depthData[4][640 * 480 * 1];
	unsigned short irData[4][640 * 480 * 1];

	// cx, cy, fx, fy
	float colorK[4][4];
	float depthK[4][4];
	float colorCoeffs[4][5];
	float depthCoeffs[4][5];

	double colorTime[4];
	double depthTime[4];

	int colorWidth, colorHeight;
	int depthWidth, depthHeight;

	int num_of_senseor;
	int ref_cam_idx;

	float depth_to_color_R[4][9], depth_to_color_tvec[4][3];

	char camera_order[4][50];
	RGBDcamera()
	{
		memset(colorData, 0, sizeof(unsigned char) * 4 * 640 * 480 * 3);
		memset(depthData, 0, sizeof(unsigned short) * 4 * 640 * 480);
		memset(irData, 0, sizeof(unsigned short) * 4 * 640 * 480);

		memset(colorK, 0, sizeof(float) * 4 * 4);
		memset(depthK, 0, sizeof(float) * 4 * 4);
		memset(colorCoeffs, 0, sizeof(float) * 4 * 5);
		memset(depthCoeffs, 0, sizeof(float) * 4 * 5);
		
		memset(colorTime, 0, sizeof(double) * 4);
		memset(depthTime, 0, sizeof(double) * 4);
		
		colorWidth = 0; colorHeight = 0;
		depthWidth = 0; depthWidth = 0;

		num_of_senseor = 0;
		ref_cam_idx = 0;

		memset(depth_to_color_R, 0, sizeof(float) * 4 * 9);
		memset(depth_to_color_tvec, 0, sizeof(float) * 4 * 3);

		memset(camera_order, 0, sizeof(char) * 4 * 50);
	}
	RGBDcamera(const RGBDcamera& rgbd)
	{
		memcpy(colorData, rgbd.colorData, sizeof(unsigned char) * 4 * 640 * 480 * 3);
		memcpy(depthData, rgbd.depthData, sizeof(unsigned short) * 4 * 640 * 480);
		memcpy(irData, rgbd.irData, sizeof(unsigned short) * 4 * 640 * 480);

		memcpy(colorK, rgbd.colorK, sizeof(float) * 4 * 4);
		memcpy(depthK, rgbd.depthK, sizeof(float) * 4 * 4);
		memcpy(colorCoeffs, rgbd.colorCoeffs, sizeof(float) * 4 * 5);
		memcpy(depthCoeffs, rgbd.depthCoeffs, sizeof(float) * 4 * 5);

		memcpy(colorTime, rgbd.colorTime, sizeof(double) * 4);
		memcpy(depthTime, rgbd.depthTime, sizeof(double) * 4);

		colorWidth = rgbd.colorWidth; colorHeight = rgbd.colorHeight;
		depthWidth = rgbd.depthWidth; depthHeight = rgbd.depthHeight;

		num_of_senseor = rgbd.num_of_senseor;
		ref_cam_idx = rgbd.ref_cam_idx;

		memcpy(depth_to_color_R, rgbd.depth_to_color_R, sizeof(float) * 4 * 9);
		memcpy(depth_to_color_tvec, rgbd.depth_to_color_tvec, sizeof(float) * 4 * 3);

		memcpy(camera_order, rgbd.camera_order, sizeof(char) * 4 * 50);
	}
};

struct RealSenseData
{
	//////////////////////////// input_grid ////////////////////////////
	bool get_color = 1;
	bool get_grid = 1;

	int        cgridWidth = 500;
	int        cgridHeight = 500;
	int		   cRobotCol = 250;
	int		   cRobotRow = 350;

	double mm2grid = 100.0 / 1000.0;

	//////////////////////////// output_grid ////////////////////////////
	unsigned char gridData[500 * 500];
	unsigned char freeData[500 * 500];
	unsigned char occupyData[500 * 500];
	int sensorNumber = 3;
	unsigned char colorData[3][640 * 480 * 3];
	///////////////////////////////////////////////// bg
	int get_depth = 1;								////
	unsigned short depthData[3][320 * 240 * 1];		////
	float depthScale[3];							////
													//	unsigned short irData[3][320 * 240 * 1];		////
	int colorSize_width, colorSize_height;			////
	int depthSize_width, depthSize_height;			////
	float colorK[3][9], depthK[3][9];				////	fx fy ppx ppy
	float colorCoeffs[3][5], depthCoeffs[3][5];		////
	float depth_to_color_rot[3][9], depth_to_color_tran[3][3];		////

	int ref_cam_idx;
	////////////////////////////////////////////////////

	//////////////////////////// input_vision ////////////////////////////
	int mode = 0;
	bool always_on_flag = false;

	int querySize = 0;
	int queryidx[10];
	double queryIniPose[10][9];
	//////////////////////////// output_vision ////////////////////////////
	double matchingRate[10];
	double matchingResult[10][9];

	int ms;
};

struct ConaSlamData
{
	//////////////////////////// input ////////////////////////////

	unsigned char RGB[640 * 480 * 3];

	int mode = 0;

	bool near_mode = false;
	int near_idx = -1;

	bool bUseViewer = false;

	char strVocFile[256];

	char strSettingsFile[256];
};

struct OrbbecCameraData;

struct ExtrinsicCalibration
{
	// input
	bool bStart;
	int argc;
	char argv[100][50];

	// output
	bool isCalibrated;
	double cam_to_cam_R[3][9];
	double cam_to_cam_tvec[3][3];

	double tot_error[3];
	double avr_error[3];

	ExtrinsicCalibration()
		:bStart(false), argc(0), isCalibrated(false)
	{
		memset(argv, 0, sizeof(char) * 100 * 50);
		memset(cam_to_cam_R, 0, sizeof(double) * 3 * 9);
		memset(cam_to_cam_tvec, 0, sizeof(double) * 3 * 3);
		memset(tot_error, 0, sizeof(double) * 3);
		memset(avr_error, 0, sizeof(double) * 3);
	}
};

struct GroundDetectionData
{
	// input
	char camera_name[50];	// realsense, orbbec
	bool bStart;
	int iterator;

	// output
	double R[9];
	double tvec[3];
	bool isDone;

	GroundDetectionData()
		:bStart(false), iterator(20), isDone(false)
	{
		memset(camera_name, 0, sizeof(char) * 50);
		memset(R, 0, sizeof(double) * 9);
		memset(tvec, 0, sizeof(double) * 3);
	}
};
struct MultipleCalibration
	:public ExtrinsicCalibration,
	public GroundDetectionData
{
	void setExtrinsic(const ExtrinsicCalibration& ec)
	{
		this->ExtrinsicCalibration::bStart = ec.bStart;
		this->argc = ec.argc;
		this->isCalibrated = ec.isCalibrated;
		memcpy(this->argv, ec.argv, sizeof(char) * 100 * 50);
		memcpy(this->cam_to_cam_R, ec.cam_to_cam_R, sizeof(double) * 3 * 9);
		memcpy(this->cam_to_cam_tvec, ec.cam_to_cam_tvec, sizeof(double) * 3 * 3);
		memcpy(this->tot_error, ec.tot_error, sizeof(double) * 3);
		memcpy(this->avr_error, ec.avr_error, sizeof(double) * 3);
	}
	void setGroundDetection(const GroundDetectionData& gd)
	{
		this->GroundDetectionData::bStart = gd.bStart;
		this->iterator = gd.iterator;
		this->isDone = gd.isDone;

		memcpy(this->camera_name, gd.camera_name, sizeof(char) * 30);
		memcpy(this->R, gd.R, sizeof(double) * 9);
		memcpy(this->tvec, gd.tvec, sizeof(double) * 3);
	}
};

