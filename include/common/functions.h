#pragma once

#include <string>
#include <windows.h>
#include <sys/timeb.h>
#include <mutex>
#include <map>
#include <shellapi.h>
#include <direct.h>
#include <fstream>
#include <vector>

#include "opencv2\opencv.hpp"

#pragma comment(lib, "Shell32.lib")

class IPC_v2
{
public:
	std::string owner_name;

private:
	struct IPC_state
	{
		// target FPS. 
		float target_hz;

		//controled FPS.
		float actual_hz;

		//actaul computing time
		//it will be checked by waitIPC function
		int ms;

		//exe path
		char exe_path[512];

		//for checking owner
		_timeb last_owner_call;

		//for checking connection
		_timeb last_guest_call;

		//for controling hz
		_timeb iteration_call;

		bool exit_commend;

		IPC_state::IPC_state()
			: target_hz(30.0f), actual_hz(0.0f), ms(0),
			exit_commend(false)
		{};
	};
	struct IPC_connection_info
	{
		HANDLE hMapFile;
		IPC_state* state;
		LPCTSTR pBuf;
		unsigned int data_size;
		bool is_initialized = true;

		IPC_connection_info::IPC_connection_info()
			: hMapFile(NULL), state(nullptr), data_size(0)
		{};
		bool isOpen(void) { return hMapFile != NULL; };
		bool isConnected(void) { return pBuf != nullptr; };
	};

	std::mutex IPC_list_mutex;
	unsigned int sleep_ms_time;
	std::thread* ipc_thread;
	IPC_connection_info* own_IPC;

	std::map<std::string, IPC_connection_info*> IPC_list;

	typedef void(*CallBack_Function)(void* Userdata);
	std::vector<std::pair<CallBack_Function, void*>> callback_functions;

	void ipc_action(void)
	{
		//while (!own_IPC->state->exit_commend)
		while (1)
		{
			if (own_IPC)
			{
				if (own_IPC->state->exit_commend) break;
				_ftime64_s(&own_IPC->state->last_owner_call);
			}

			IPC_list_mutex.lock();
			std::map<std::string, IPC_connection_info*>::iterator i = IPC_list.begin();
			for (; i != IPC_list.end(); i++) 
				if (i->first != owner_name)
					_ftime64_s(&i->second->state->last_guest_call);
			IPC_list_mutex.unlock();

			Sleep(sleep_ms_time);
		}

		for (int i = 0; i < (int)callback_functions.size(); i++)
			callback_functions[i].first(callback_functions[i].second);

		own_IPC->state->exit_commend = false;

		exit(EXIT_SUCCESS);
	};

	//linking pbuf and IPC_state pointer 
	bool connectBuf(std::map<std::string, IPC_connection_info*>::iterator &_func)
	{
		if (_func->second->hMapFile == NULL) return false;

		_func->second->pBuf = (LPTSTR)MapViewOfFile(_func->second->hMapFile, // handle to map object
			FILE_MAP_ALL_ACCESS,  // read/write permission
			0,
			0,
			_func->second->data_size);

		if (_func->second->pBuf == NULL)
		{
			CloseHandle(_func->second->hMapFile);
			return false;
		}

		_func->second->state = (IPC_state*)_func->second->pBuf;
		return true;
	};

	//allocate MMF and connect data Buf
	bool createMMF(std::map<std::string, IPC_connection_info*>::iterator &_func)
	{
		unsigned int data_size = (unsigned int)sizeof(IPC_state) + _func->second->data_size;

		_func->second->hMapFile = CreateFileMapping(
			INVALID_HANDLE_VALUE,    // use paging file
			NULL,                    // default security
			PAGE_READWRITE,          // read/write access
			0,                       // maximum object size (high-order DWORD)
			data_size,                // maximum object size (low-order DWORD)
			_func->first.c_str());                 // name of mapping object

		if (_func->second->hMapFile == NULL) return false;

		if (!openMMF(_func)) return false;

		std::memcpy(_func->second->state, &IPC_state(), sizeof(IPC_state));

		_func->second->is_initialized = false;

		return true;
	};

	//accece MMF and connect data Buf
	bool openMMF(std::map<std::string, IPC_connection_info*>::iterator &_func)
	{
		_func->second->hMapFile = OpenFileMapping(
			FILE_MAP_ALL_ACCESS,   // read/write access
			FALSE,                 // do not inherit the name
			_func->first.c_str());

		if (_func->second->hMapFile == NULL) return false;

		if (!connectBuf(_func)) return false;

		return true;
	};

	unsigned int get_ms_duration(_timeb &to, _timeb &from)
	{
		unsigned int dt = 1000 * (unsigned int)(to.time - from.time);
		unsigned int dmt = (unsigned int)(to.millitm - from.millitm);

		return dt + dmt;
	};

	bool check_owner(std::map<std::string, IPC_connection_info*>::iterator finder)
	{
		_timeb now;
		_ftime64_s(&now);

		unsigned int dt = get_ms_duration(now, finder->second->state->last_owner_call);

		if (dt < sleep_ms_time + 50) return true;

		return false;
	}

public:

	IPC_v2::IPC_v2() :
		sleep_ms_time(500), ipc_thread(nullptr), own_IPC(nullptr)
	{
		ipc_thread = new std::thread(&IPC_v2::ipc_action, this);
	};

	IPC_v2::IPC_v2(std::string Owner_name) :
		sleep_ms_time(500), ipc_thread(nullptr), own_IPC(nullptr),
		owner_name(Owner_name)
	{
		ipc_thread = new std::thread(&IPC_v2::ipc_action, this);
	};

	bool connect(std::string IPC_name, unsigned int _data_size = 0)
	{
		std::map<std::string, IPC_connection_info*>::iterator finder = IPC_list.find(IPC_name);
		if (finder != IPC_list.end()) if (finder->second->isConnected())
			return true;

		IPC_list_mutex.lock();
		IPC_list.insert(std::pair<std::string, IPC_connection_info*>
			(IPC_name, new IPC_connection_info()));
		IPC_list_mutex.unlock();

		finder = IPC_list.find(IPC_name);

		finder->second->data_size = _data_size;

		if (!openMMF(finder))
			if (!createMMF(finder))
				return false;

		if (IPC_name == owner_name) set_owner(IPC_name);


		return true;
	};

	template <typename T> T* connect(std::string IPC_name)
	{
		if (!connect(IPC_name, (unsigned int)sizeof(T)))
			return nullptr;

		std::map<std::string, IPC_connection_info*>::iterator finder = IPC_list.find(IPC_name);
		if (finder == IPC_list.end()) return nullptr;

		if (!finder->second->is_initialized)
		{
			T* initilizer = new T;
			std::memcpy((void*)&finder->second->pBuf[sizeof(IPC_state)], initilizer, sizeof(T));
			finder->second->is_initialized = true;

			delete initilizer;
		}

		return (T*)&finder->second->pBuf[sizeof(IPC_state)];
	}

	bool set_owner(std::string IPC_name)
	{
		if (!owner_name.empty()) if (owner_name != IPC_name) return false;

		std::map<std::string, IPC_connection_info*>::iterator finder = IPC_list.find(IPC_name);
		if (finder == IPC_list.end()) return false;

		_timeb now;
		_ftime64_s(&now);

		unsigned int dt = get_ms_duration(now, finder->second->state->last_owner_call);
		if (dt < sleep_ms_time) return false;

		own_IPC = finder->second;
		owner_name = IPC_name;
		int bytes = GetModuleFileName(NULL, own_IPC->state->exe_path, 512);
		if (bytes >= 512)
			printf("Too long address\n -> %s\n", own_IPC->state->exe_path);

		return true;
	}

	std::string get_exe_path(std::string IPC_name)
	{
		std::map<std::string, IPC_connection_info*>::iterator finder = IPC_list.find(IPC_name);
		if (finder == IPC_list.end()) return std::string();

		if (!check_owner(finder)) return std::string();

		return std::string(finder->second->state->exe_path);
	}

	bool check_owner(std::string IPC_name)
	{
		std::map<std::string, IPC_connection_info*>::iterator finder = IPC_list.find(IPC_name);
		if (finder == IPC_list.end()) return false;

		return check_owner(finder);
	}

	bool check_guest(void)
	{
		if (!own_IPC) return false;

		_timeb now;
		_ftime64_s(&now);

		unsigned int dt = get_ms_duration(now, own_IPC->state->last_guest_call);

		if (dt < sleep_ms_time) return true;

		return false;
	}

	bool check_guest(std::string IPC_name)
	{
		std::map<std::string, IPC_connection_info*>::iterator finder = IPC_list.find(IPC_name);
		if (finder == IPC_list.end()) return false;

		_timeb now;
		_ftime64_s(&now);

		unsigned int dt = get_ms_duration(now, finder->second->state->last_guest_call);

		if (dt < sleep_ms_time) return true;

		return false;
	}

	bool set_target_hz(std::string IPC_name, float Target_hz)
	{
		std::map<std::string, IPC_connection_info*>::iterator finder = IPC_list.find(IPC_name);
		if (finder == IPC_list.end()) return false;

		finder->second->state->target_hz = Target_hz;

		return true;
	}

	float get_actual_hz(void)
	{
		if (own_IPC) return own_IPC->state->actual_hz;
		else return 0.0f;
	}

	float get_actual_hz(std::string IPC_name)
	{
		std::map<std::string, IPC_connection_info*>::iterator finder = IPC_list.find(IPC_name);
		if (finder == IPC_list.end()) return 0.0f;

		return 	finder->second->state->actual_hz;
	}

	int get_ms(void)
	{
		if (own_IPC) return own_IPC->state->ms;
		else return 0;
	}

	int get_ms(std::string IPC_name)
	{
		std::map<std::string, IPC_connection_info*>::iterator finder = IPC_list.find(IPC_name);
		if (finder == IPC_list.end()) return 0;

		return 	finder->second->state->ms;
	}

	//it's performed by wait_IPC
	int get_iteration_time(void)
	{
		if (own_IPC) return own_IPC->state->ms;

		return 0;
	}

	//it's performed by wait_IPC
	int get_iteration_time(std::string IPC_name)
	{
		std::map<std::string, IPC_connection_info*>::iterator finder = IPC_list.find(IPC_name);
		if (finder == IPC_list.end()) return 0;

		return finder->second->state->ms;
	}

	void wait_IPC(void)
	{
		if (!own_IPC)
		{
			Sleep(30);
			return;
		}

		_timeb now;
		_ftime64_s(&now);

		int dt = (int)get_ms_duration(now, own_IPC->state->iteration_call);

		own_IPC->state->ms = dt;

		float designed_t_gap = 1000.0f / own_IPC->state->target_hz;
		if (!isfinite(designed_t_gap)) designed_t_gap = 1000.0f / 30.0f;

		float t_gap_to_target_hz = designed_t_gap - (float)dt;
		if (t_gap_to_target_hz > 0.0f && dt >= 0)
			Sleep((int)t_gap_to_target_hz);

		//printf("%03f\t%03f\t%03f\n", (float)1000.0f / own_IPC->state->target_hz, (float)dt, t_gap_to_target_hz);

		_ftime64_s(&now);

		float actual_dt = (float)get_ms_duration(now, own_IPC->state->iteration_call);

		own_IPC->state->actual_hz = 1000.0f / actual_dt;

		own_IPC->state->iteration_call = now;
	}

	bool wait_val(int &target, int Val, int milli_sec = 1000)
	{
		int sllep_time = 10;
		int counter = milli_sec / sllep_time;

		while (counter-- > 0)
		{
			if (target == Val) return 1;
			Sleep(sllep_time);
		}

		return 0;
	}

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

	int exit_IPC(std::string IPC_name)
	{
		if (!check_owner(IPC_name)) return 0;

		std::map<std::string, IPC_connection_info*>::iterator finder = IPC_list.find(IPC_name);
		if (finder == IPC_list.end()) return 0;

		finder->second->state->exit_commend = true;

		return 1;
	};

	void registe_exit_callback(CallBack_Function callback_function, void* userdata = 0)
	{
		callback_functions.push_back(std::pair<CallBack_Function, void*>(callback_function, userdata));
	};
};

class Virtual_DB_mannager
{
private:
	std::ofstream saver;
	std::ifstream loader;

public:
	std::string DB_path = "..\\..\\simul_DB\\";

	void save(char* data, unsigned long long size, std::string File_name)
	{
		_mkdir(DB_path.c_str());

		saver.open(DB_path + File_name + ".sim", std::ios_base::out | std::ios_base::trunc | std::ios_base::binary);

		if (!saver.is_open()) return;

		saver.write(data, size);

		saver.close();
	}
	void load(char* data, unsigned long long size, std::string File_name)
	{
		loader.open(DB_path + File_name + ".sim", std::ios_base::in | std::ios_base::binary);

		if (!loader.is_open()) return;

		loader.read(data, size);

		loader.close();
	}
	void saveImg(cv::Mat& img, std::string File_name)
	{
		_mkdir(DB_path.c_str());

		cv::imwrite(DB_path + File_name + ".jpg", img);
	}
	void loadImg(cv::Mat& img, std::string File_name)
	{
		img = cv::imread(DB_path + File_name + ".jpg");
	}
};

struct Vurtiual_DB
{
	//0:none, 1:record, 2:play
	int simulation_mode = 0;
	int simulation_DB_idx = 0;

	bool simulation_DB_visualize = false;
};

struct MapData_Folder
{
	char mapData_folder[512] = "..\\..\\MapData\\";
};

struct Mode
{
private:
	int mode_flag;

public:
	Mode::Mode()
		: mode_flag(0)
	{};
	enum FLAG
	{
		querying,
		inserting,
		saveDB,
		loadDB,
		reset,
		always_on,
		draw1,
		draw2,
		draw3,
		draw4,
		option1
	};

	void clear_flag(void) { mode_flag = 0; }
	bool empty_flag(void) { return mode_flag == 0; };
	bool get_flag(FLAG idx) { return (mode_flag & 1 << idx) != 0; }
	bool get_and_off_flag(FLAG idx)
	{
		bool val = get_flag(idx);
		if (val) mode_flag -= 1 << idx;
		return val;
	}
	void set_flag_on(FLAG idx) { mode_flag |= 1 << idx; }
	void set_flag_off(FLAG idx) { get_and_off_flag(idx); }
	void set_flag(FLAG idx, bool val)
	{
		if (val) set_flag_on(idx);
		else set_flag_off(idx);
	}
	bool check_draw_flag(void)
	{
		if (get_flag(FLAG::draw1)) return true;
		if (get_flag(FLAG::draw2)) return true;
		if (get_flag(FLAG::draw3)) return true;
		if (get_flag(FLAG::draw4)) return true;

		return false;
	}
	bool wait_flag(FLAG idx, bool val = false, int ms = 1000)
	{
		int wait_time = 10;
		int counter = ms / wait_time;
		while (counter-- > 0)
		{
			if (get_flag(idx) == val) return true;
			Sleep(wait_time);
		}

		return false;
	}
};

struct Robot : Vurtiual_DB, Mode
{
	//////////////////////////// input ////////////////////////////
	int set_max_vel = 700;
	int set_max_rot_vel = 20;

	double set_vel = 0.0;
	double set_rotvel = 0.0;

	bool navigate_on = false;

	//mm
	//<0:not observed
	double dist_2_destination = -1.0;
	//bool near_goal = false;

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
struct RGBDcamera :Vurtiual_DB, Mode, MapData_Folder
{
	bool registration_enable;

	unsigned char colorData[4][1280 * 960 * 3];
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
		registration_enable = false;

		memset(colorData, 0, sizeof(unsigned char) * 4 * 1280 * 960 * 3);
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
		registration_enable = rgbd.registration_enable;
		memcpy(colorData, rgbd.colorData, sizeof(unsigned char) * 4 * 1280 * 960 * 3);
		memcpy(depthData, rgbd.depthData, sizeof(unsigned short) * 4 * 640 * 480);
		memcpy(irData, rgbd.irData, sizeof(unsigned short) * 4 * 640 * 480);

		memcpy(colorK, rgbd.colorK, sizeof(float) * 4 * 4);
		memcpy(depthK, rgbd.depthK, sizeof(float) * 4 * 4);
		memcpy(colorCoeffs, rgbd.colorCoeffs, sizeof(float) * 4 * 5);
		memcpy(depthCoeffs, rgbd.depthCoeffs, sizeof(float) * 4 * 5);

		colorWidth = rgbd.colorWidth; colorHeight = rgbd.colorHeight;
		depthWidth = rgbd.depthWidth; depthHeight = rgbd.depthHeight;

		num_of_senseor = rgbd.num_of_senseor;
		ref_cam_idx = rgbd.ref_cam_idx;

		memcpy(colorTime, rgbd.colorTime, sizeof(double) * 4);
		memcpy(depthTime, rgbd.depthTime, sizeof(double) * 4);

		memcpy(depth_to_color_R, rgbd.depth_to_color_R, sizeof(float) * 4 * 9);
		memcpy(depth_to_color_tvec, rgbd.depth_to_color_tvec, sizeof(float) * 4 * 3);

		memcpy(camera_order, rgbd.camera_order, sizeof(char) * 4 * 50);
	}
};

struct ExtrinsicCalibration
{
	// mode{ draw1: feature matching, draw2: camera pose }
	// input
	int* m_pMode;
	bool bStart;
	int argc;
	char argv[100][50];

	// for test
	float depth_slope[3];
	float depth_offset[3];

	// output
	bool isCalibrated;
	double cam_to_cam_R[3][9];
	double cam_to_cam_tvec[3][3];

	double tot_error[3];
	double avr_error[3];

	double gathering_data_hz;

	ExtrinsicCalibration()
		:bStart(false), argc(0), isCalibrated(false)
	{
		memset(argv, 0, sizeof(char) * 100 * 50);

		for (int i = 0; i < 3; i++) {
			depth_slope[i] = 1.f;
			depth_offset[i] = 0.f;
		}

		memset(cam_to_cam_R, 0, sizeof(double) * 3 * 9);
		memset(cam_to_cam_tvec, 0, sizeof(double) * 3 * 3);
		memset(tot_error, 0, sizeof(double) * 3);
		memset(avr_error, 0, sizeof(double) * 3);

		gathering_data_hz = 60;
	}
};

struct GroundDetectionData
{
	// input
	int *m_pMode;
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
	public GroundDetectionData,
	public Mode
{
	// Mode{ draw1: Extrinsic(feature matching),  draw2: Extrinsic(camera pose),
	//       draw3: GroundDetection(inlier img),  draw4: live 3D Visualize }
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

struct GridData
{
	//0:wait, 1:every iterator
	int get_grid_mode = 1;
	bool mamual_ground_detecte = false;

	double mm2grid = 100.0 / 1000.0;

	int cgridWidth = 500;
	int cgridHeight = 500;
	int cRobotCol = 250;
	int cRobotRow = 350;

	unsigned char gridData[500 * 500];
	unsigned char freeData[500 * 500];
	unsigned char occupyData[500 * 500];
};

struct MotionCalc : Mode
{
	//////////////////////////// input_vision ////////////////////////////
	//int num_of_senseor;
	//int cColorWidth = 640;
	//int cColorHeight = 480;
	//unsigned char colorData[3][640 * 480 * 3];

	//int mode = 0;
	//bool always_on_flag = false;

	int querySize = 0;
	int queryidx[10];
	double queryIniPose[10][9];

	//////////////////////////// output_vision ////////////////////////////
	double matchingRate[10];
	double matchingResult[10][9];
};